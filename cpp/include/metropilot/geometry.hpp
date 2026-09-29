#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace metropilot {

struct Vec3 {
  double x{};
  double y{};
  double z{};
};

struct RigidTransform {
  // Row-major active rotation mapping source vectors into destination axes.
  std::array<double, 9> rotation{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
  };
  Vec3 translation{};

  Vec3 apply(const Vec3& p) const;
};

RigidTransform compose(
    const RigidTransform& destination_from_intermediate,
    const RigidTransform& intermediate_from_source
);

struct SensorMountCalibration {
  // Translation of sensor origin in vehicle_track_reference coordinates.
  // vehicle_track_reference uses +x forward, +y left, +z up, z=0 at TOR.
  std::optional<double> longitudinal_offset_m;
  std::optional<double> lateral_offset_m;
  std::optional<double> height_above_tor_m;

  // Active sensor-native -> vehicle_track_reference orientation.
  // Convention: R = Rz(yaw) * Ry(pitch) * Rx(roll), degrees.
  std::optional<double> roll_deg;
  std::optional<double> pitch_deg;
  std::optional<double> yaw_deg;
};

enum class CalibrationStatus {
  Valid,
  Incomplete,
  NonFinite,
};

struct CalibrationResult {
  CalibrationStatus status{CalibrationStatus::Incomplete};
  std::vector<std::string> missing_fields;
  std::optional<RigidTransform> sensor_to_vehicle_track;
};

CalibrationResult resolve_sensor_mount(
    const SensorMountCalibration& calibration
);

// Static adapter from an observed PointCloud frame into the physical sensor
// frame to which SensorMountCalibration applies. frame_id is a label only;
// no frame is trusted without an explicit adapter entry.
struct NativeFrameAdapterCalibration {
  std::string frame_id;

  // Translation of native-frame origin expressed in physical sensor axes.
  std::optional<double> translation_x_m;
  std::optional<double> translation_y_m;
  std::optional<double> translation_z_m;

  // Active native -> physical-sensor orientation.
  // R = Rz(yaw) * Ry(pitch) * Rx(roll), degrees.
  std::optional<double> roll_deg;
  std::optional<double> pitch_deg;
  std::optional<double> yaw_deg;
};

enum class NativeFrameResolutionStatus {
  Valid,
  UnknownFrame,
  AdapterIncomplete,
  AdapterNonFinite,
  MountIncomplete,
  MountNonFinite,
};

struct NativeFrameResolution {
  NativeFrameResolutionStatus status{
      NativeFrameResolutionStatus::UnknownFrame
  };
  std::vector<std::string> missing_fields;
  std::optional<RigidTransform> native_to_vehicle_track;
};

NativeFrameResolution resolve_native_frame_to_vehicle(
    const std::string& frame_id,
    const std::vector<NativeFrameAdapterCalibration>& adapters,
    const SensorMountCalibration& sensor_mount
);

}  // namespace metropilot
