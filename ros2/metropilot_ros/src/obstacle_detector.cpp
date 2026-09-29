#include "metropilot/detector_pipeline.hpp"
#include "metropilot/precision_gate.hpp"
#include "metropilot/sensor_mount_config.hpp"
#include "metropilot/vehicle_geometry_config.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
using namespace metropilot;
PointFieldType point_field_type(std::uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return PointFieldType::Int8;
    case sensor_msgs::msg::PointField::UINT8:
      return PointFieldType::UInt8;
    case sensor_msgs::msg::PointField::INT16:
      return PointFieldType::Int16;
    case sensor_msgs::msg::PointField::UINT16:
      return PointFieldType::UInt16;
    case sensor_msgs::msg::PointField::INT32:
      return PointFieldType::Int32;
    case sensor_msgs::msg::PointField::UINT32:
      return PointFieldType::UInt32;
    case sensor_msgs::msg::PointField::FLOAT32:
      return PointFieldType::Float32;
    case sensor_msgs::msg::PointField::FLOAT64:
      return PointFieldType::Float64;
    default:
      throw std::runtime_error(
          "unsupported ROS PointField datatype: " +
          std::to_string(datatype)
      );
  }
}

struct PointCloudView {
  PointCloudBuffer buffer;
};

PointCloudView make_core_view(
    const sensor_msgs::msg::PointCloud2& message
) {
  PointCloudView out;
  out.buffer.height = message.height;
  out.buffer.width = message.width;
  out.buffer.fields.reserve(message.fields.size());
  for (const auto& field : message.fields) {
    out.buffer.fields.push_back(
        PointField{
            field.name,
            field.offset,
            point_field_type(field.datatype),
            field.count,
        }
    );
  }
  out.buffer.is_bigendian = message.is_bigendian;
  out.buffer.point_step = message.point_step;
  out.buffer.row_step = message.row_step;
  out.buffer.data =
      message.data.empty()
          ? nullptr
          : message.data.data();
  out.buffer.data_size = message.data.size();
  out.buffer.is_dense = message.is_dense;
  return out;
}

std::string schema_signature(
    const sensor_msgs::msg::PointCloud2& message
) {
  std::ostringstream out;
  out
      << message.height << '|'
      << message.point_step << '|'
      << (message.is_bigendian ? 1 : 0);
  for (const auto& field : message.fields) {
    out
        << ';' << field.name
        << ':' << field.offset
        << ':' << static_cast<int>(field.datatype)
        << ':' << field.count;
  }
  return out.str();
}

}

class ObstacleDetector final : public rclcpp::Node {
 public:
  ObstacleDetector() : Node("obstacle_detector") {
    using namespace metropilot;
    const auto share = ament_index_cpp::get_package_share_directory("metropilot_ros");
    calibration_ = load_sensor_calibration_yaml(setting<std::string>(
        "sensor_calibration_file", share + "/config/sensor_mount_challenge_assumed.yaml"));
    geometry_ = load_vehicle_sweep_geometry_yaml(setting<std::string>(
        "vehicle_geometry_file", share + "/config/vehicle_geometry.yaml"));
    pivot_offset_ = setting<double>("front_pivot_offset_m", -3.960);
    stale_timeout_ = setting<double>("stale_timeout_s", 0.5);
    verbose_ = setting<bool>("debug_log", false);
    if (!std::isfinite(pivot_offset_) || !std::isfinite(stale_timeout_) || stale_timeout_ <= 0)
      throw std::invalid_argument("invalid pivot offset or stale timeout");
    PrecisionGateConfig gate_config;
    auto count = [this](const char* name, std::size_t value) {
      const auto n = setting<int64_t>(name, value);
      if (n <= 0) throw std::invalid_argument(std::string(name) + " must be > 0");
      return static_cast<std::size_t>(n);
    };
    gate_config.min_firings = count("min_firings", gate_config.min_firings);
    gate_config.min_acquisition_groups = count("min_acquisition_groups", gate_config.min_acquisition_groups);
    gate_config.min_nominal_endpoints = count("min_nominal_endpoints", gate_config.min_nominal_endpoints);
    gate_config.confirmation_frames = count("confirmation_frames", gate_config.confirmation_frames);
    gate_config.min_xyzi_spatial_cells = count("min_xyzi_spatial_cells", gate_config.min_xyzi_spatial_cells);
    gate_config.min_xyzi_above_rail_cells = count("min_xyzi_above_rail_cells", gate_config.min_xyzi_above_rail_cells);
    gate_config.min_known_above_rail_cells = count("min_known_above_rail_cells", gate_config.min_known_above_rail_cells);
    gate_config.allow_xyzi = setting<bool>("allow_xyzi", gate_config.allow_xyzi);
    gate_config.min_non_structural_fraction = setting<double>(
        "min_non_structural_fraction", gate_config.min_non_structural_fraction);
    gate_config.max_source_gap_s = setting<double>("max_source_gap_s", gate_config.max_source_gap_s);
    gate_config.max_longitudinal_speed_mps = setting<double>(
        "max_longitudinal_speed_mps", gate_config.max_longitudinal_speed_mps);
    gate_ = PrecisionGate(gate_config);
    const auto reliability = setting<std::string>("input_reliability", "best_effort");
    if (reliability != "best_effort" && reliability != "reliable")
      throw std::invalid_argument("input_reliability must be best_effort or reliable");
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    if (reliability == "reliable") qos.reliable();
    output_ = create_publisher<std_msgs::msg::Bool>(
        setting<std::string>("output_topic", "/obstacle_detected"), rclcpp::QoS(1).reliable());
    input_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        setting<std::string>("input_topic", "/lidar_points"), qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { process(*cloud); });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      if (!stale_reported_ && last_received_ && std::chrono::duration<double>(Clock::now() - *last_received_).count() > stale_timeout_) {
        gate_.reset();
        if (last_flag_) publish(false);
        stale_reported_ = true;
        RCLCPP_WARN(get_logger(), "No fresh lidar data: obstacle flag reset");
      }
    });
    RCLCPP_INFO(get_logger(), "Precision-first detector ready; confirmation=%zu frames, assumed_calibration=%s",
        gate_config.confirmation_frames, calibration_.uses_assumed_calibration ? "true" : "false");
  }
 private:
  template<typename T>
  T setting(const std::string& name, const T& default_value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, default_value, descriptor);
  }
  using Clock = std::chrono::steady_clock;
  void publish(bool flag) {
    std_msgs::msg::Bool message;
    message.data = flag;
    output_->publish(message);
    last_flag_ = flag;
  }
  void process(const sensor_msgs::msg::PointCloud2& cloud) {
    const auto started = Clock::now();
    if (last_received_ && std::chrono::duration<double>(started - *last_received_).count() > stale_timeout_)
      gate_.reset();
    last_received_ = started;
    stale_reported_ = false;
    try {
      if (cloud.header.stamp.nanosec >= 1000000000U)
        throw std::runtime_error("Invalid header timestamp nanoseconds");
      if (cloud.data.size() > 64 * 1024 * 1024)
        throw std::runtime_error("PointCloud2 exceeds 64 MiB input limit");
      const auto signature = cloud.header.frame_id + "|" + schema_signature(cloud);
      if (signature != signature_) { gate_.reset(); signature_ = signature; }
      const auto resolution = metropilot::resolve_configured_native_frame_to_vehicle(cloud.header.frame_id, calibration_);
      if (!resolution.native_to_vehicle_track)
        throw std::runtime_error("No complete configured transform for frame_id=" + cloud.header.frame_id);
      const auto result = metropilot::detect_cloud(make_core_view(cloud).buffer,
          *resolution.native_to_vehicle_track, geometry_, pivot_offset_);
      const int64_t stamp = static_cast<int64_t>(cloud.header.stamp.sec) * 1000000000LL + cloud.header.stamp.nanosec;
      const bool flag = gate_.update(result, stamp);
      if (verbose_) {
        for (const auto& a : result.classification.assessments) {
          const auto& c = result.candidates.at(a.candidate_index);
          if (a.disposition == metropilot::CandidateDisposition::StructuralCompatible ||
              (c.non_structural_nominal_spatial_cells < 12 &&
               c.non_structural_known_independent_firings < 12)) continue;
          RCLCPP_INFO(get_logger(),
              "candidate xyz=%.3f,%.3f,%.3f span=%.3f,%.3f,%.3f firings=%zu cells=%zu above_rail_cells=%zu nominal=%zu structural=%.3f",
              c.centroid.x, c.centroid.y, c.centroid.z,
              c.longitudinal_span_m, c.lateral_span_m, c.vertical_span_m,
              c.non_structural_known_independent_firings,
              c.non_structural_nominal_spatial_cells,
              c.non_structural_nominal_above_rail_spatial_cells,
              c.non_structural_nominal_sweep_endpoints, c.soft_structural_fraction);
        }
      }
      const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
      if (verbose_ || flag != last_flag_) {
        RCLCPP_INFO(get_logger(), "stamp=%ld flag=%d valid=%d raw_candidates=%zu qualified=%zu ms=%.2f",
            static_cast<long>(stamp), flag, result.status == metropilot::DetectorStatus::Valid,
            result.supported_obstacle_candidate_count, gate_.qualified_count(), elapsed);
      }
      if (result.status != metropilot::DetectorStatus::Valid)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Track geometry unavailable/ambiguous: flag=false");
      publish(flag);
    } catch (const std::exception& error) {
      gate_.reset();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected cloud: %s", error.what());
      publish(false);
    }
  }
  metropilot::SensorCalibrationConfig calibration_;
  metropilot::VehicleSweepGeometry geometry_;
  metropilot::PrecisionGate gate_;
  double pivot_offset_{};
  double stale_timeout_{};
  bool verbose_{};
  bool last_flag_{};
  bool stale_reported_{};
  std::string signature_;
  std::optional<Clock::time_point> last_received_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr output_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<ObstacleDetector>()); }
  catch (const std::exception& error) { std::cerr << error.what() << std::endl; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown();
  return 0;
}
