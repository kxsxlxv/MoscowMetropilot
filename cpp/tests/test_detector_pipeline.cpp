#include "metropilot/detector_pipeline.hpp"
#include "metropilot/precision_gate.hpp"
#include "metropilot/vehicle_geometry_config.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace metropilot;
template<class T> void append(std::vector<std::uint8_t>& bytes, T value) {
  const auto offset = bytes.size();
  bytes.resize(offset + sizeof(T));
  std::memcpy(bytes.data() + offset, &value, sizeof(T));
}

// Small deterministic scene in native lidar axes; no files or ROS required.
CurrentScanResult scene(const VehicleSweepGeometry& geometry, int schema,
                        bool obstacle, double bottom = 0.45, int layers = 8) {
  std::vector<Vec3> points;
  for (int i = 10; i < 400; ++i) {
    const double x = i * 0.1;
    for (double y : {-0.775, 0.775})
      for (double dy : {-0.006, -0.002, 0.002, 0.006}) points.push_back({x, y + dy, 0});
    for (double z : {0.3, 0.8, 1.3, 1.8, 2.3, 2.8}) {
      points.push_back({x, -2.4, z}); points.push_back({x, 2.4, z});
    }
  }
  if (obstacle)
    for (int x = 0; x < 8; ++x)
      for (int y = 0; y < 8; ++y)
        for (int z = 0; z < layers; ++z)
          points.push_back({12 + x * 0.04, -0.15 + y * 0.04, bottom + z * 0.04});
  std::vector<std::uint8_t> bytes;
  for (std::size_t i = 0; i < points.size(); ++i) {
    append(bytes, static_cast<float>(points[i].y));
    append(bytes, static_cast<float>(-points[i].x));
    append(bytes, static_cast<float>(points[i].z - 1.075));
    if (schema > 0) append(bytes, 32.0f);
    if (schema == 2) {
      append(bytes, static_cast<std::uint16_t>(i % 128));
      append(bytes, 1000.0 + (i / 32) * 0.0001);
    }
  }
  PointCloudBuffer cloud;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.point_step = schema == 2 ? 26 : schema == 1 ? 16 : 12;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.fields = {{"x", 0, PointFieldType::Float32}, {"y", 4, PointFieldType::Float32}, {"z", 8, PointFieldType::Float32}};
  if (schema > 0) cloud.fields.push_back({"intensity", 12, PointFieldType::Float32});
  if (schema == 2) {
    cloud.fields.push_back({"ring", 16, PointFieldType::UInt16});
    cloud.fields.push_back({"timestamp", 18, PointFieldType::Float64});
  }
  cloud.data = bytes.data(); cloud.data_size = bytes.size();
  const RigidTransform transform{{0,-1,0, 1,0,0, 0,0,1}, {0,0,1.075}};
  return detect_cloud(cloud, transform, geometry);
}

int main(int argc, char** argv) {
  if (argc != 2) throw std::runtime_error("vehicle geometry path required");
  const auto geometry = load_vehicle_sweep_geometry_yaml(argv[1]);
  int checks = 0;
  for (int schema = 0; schema < 3; ++schema) {
    PrecisionGate gate;
    std::int64_t stamp = 1000000000;
    auto check = [&](const CurrentScanResult& result, bool expected) {
      if (result.status != DetectorStatus::Valid || gate.update(result, stamp) != expected)
        throw std::runtime_error("pipeline mismatch schema=" + std::to_string(schema) + " check=" + std::to_string(checks));
      stamp += 100000000; ++checks;
    };
    const auto empty = scene(geometry, schema, false);
    const auto cube = scene(geometry, schema, true);
    const auto below = scene(geometry, schema, true, -0.12, 2);
    const auto low = scene(geometry, schema, true, 0.06, 3);
    check(empty, false);
    for (int i = 0; i < 4; ++i) check(cube, i >= 2);
    check(empty, false);
    for (int i = 0; i < 4; ++i) check(below, false);
    for (int i = 0; i < 4; ++i) check(low, i >= 2);
    check(empty, false);
  }
  std::cout << "PASS: " << checks << " pipeline assertions across XYZ/XYZI/enriched\n";
}
