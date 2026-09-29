#include "metropilot/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace metropilot {
namespace {

constexpr double kPi =
    3.141592653589793238462643383279502884;

double radians(double degrees) {
  return degrees * kPi / 180.0;
}

bool finite(const std::optional<double>& value) {
  return !value.has_value() || std::isfinite(*value);
}

RigidTransform from_rpy_xyz(
    double x_m,
    double y_m,
    double z_m,
    double roll_deg,
    double pitch_deg,
    double yaw_deg
) {
  const double cr = std::cos(radians(roll_deg));
  const double sr = std::sin(radians(roll_deg));
  const double cp = std::cos(radians(pitch_deg));
  const double sp = std::sin(radians(pitch_deg));
  const double cy = std::cos(radians(yaw_deg));
  const double sy = std::sin(radians(yaw_deg));

  RigidTransform transform;
  transform.rotation = {
      cy * cp,
      cy * sp * sr - sy * cr,
      cy * sp * cr + sy * sr,
      sy * cp,
      sy * sp * sr + cy * cr,
      sy * sp * cr - cy * sr,
      -sp,
      cp * sr,
      cp * cr,
  };
  transform.translation = Vec3{x_m, y_m, z_m};
  return transform;
}

}  // namespace

Vec3 RigidTransform::apply(const Vec3& p) const {
  return Vec3{
      rotation[0] * p.x +
          rotation[1] * p.y +
          rotation[2] * p.z +
          translation.x,
      rotation[3] * p.x +
          rotation[4] * p.y +
          rotation[5] * p.z +
          translation.y,
      rotation[6] * p.x +
          rotation[7] * p.y +
          rotation[8] * p.z +
          translation.z,
  };
}

RigidTransform compose(
    const RigidTransform& a,
    const RigidTransform& b
) {
  RigidTransform out;

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double value = 0.0;
      for (int k = 0; k < 3; ++k) {
        value +=
            a.rotation[row * 3 + k] *
            b.rotation[k * 3 + col];
      }
      out.rotation[row * 3 + col] = value;
    }
  }

  const Vec3 rotated_translation{
      a.rotation[0] * b.translation.x +
          a.rotation[1] * b.translation.y +
          a.rotation[2] * b.translation.z,
      a.rotation[3] * b.translation.x +
          a.rotation[4] * b.translation.y +
          a.rotation[5] * b.translation.z,
      a.rotation[6] * b.translation.x +
          a.rotation[7] * b.translation.y +
          a.rotation[8] * b.translation.z,
  };
  out.translation = Vec3{
      rotated_translation.x + a.translation.x,
      rotated_translation.y + a.translation.y,
      rotated_translation.z + a.translation.z,
  };
  return out;
}

CalibrationResult resolve_sensor_mount(
    const SensorMountCalibration& c
) {
  CalibrationResult out;

  auto require = [&out](
                     const std::optional<double>& value,
                     const char* name
                 ) {
    if (!value.has_value()) {
      out.missing_fields.emplace_back(name);
    }
  };

  require(
      c.longitudinal_offset_m,
      "longitudinal_offset_m"
  );
  require(c.lateral_offset_m, "lateral_offset_m");
  require(c.height_above_tor_m, "height_above_tor_m");
  require(c.roll_deg, "roll_deg");
  require(c.pitch_deg, "pitch_deg");
  require(c.yaw_deg, "yaw_deg");

  if (!out.missing_fields.empty()) {
    out.status = CalibrationStatus::Incomplete;
    return out;
  }

  if (!finite(c.longitudinal_offset_m) ||
      !finite(c.lateral_offset_m) ||
      !finite(c.height_above_tor_m) ||
      !finite(c.roll_deg) ||
      !finite(c.pitch_deg) ||
      !finite(c.yaw_deg)) {
    out.status = CalibrationStatus::NonFinite;
    return out;
  }

  out.status = CalibrationStatus::Valid;
  out.sensor_to_vehicle_track =
      from_rpy_xyz(
          *c.longitudinal_offset_m,
          *c.lateral_offset_m,
          *c.height_above_tor_m,
          *c.roll_deg,
          *c.pitch_deg,
          *c.yaw_deg
      );
  return out;
}

NativeFrameResolution resolve_native_frame_to_vehicle(
    const std::string& frame_id,
    const std::vector<NativeFrameAdapterCalibration>& adapters,
    const SensorMountCalibration& sensor_mount
) {
  NativeFrameResolution out;

  const auto it = std::find_if(
      adapters.begin(),
      adapters.end(),
      [&](const NativeFrameAdapterCalibration& adapter) {
        return adapter.frame_id == frame_id;
      }
  );
  if (it == adapters.end()) {
    out.status = NativeFrameResolutionStatus::UnknownFrame;
    return out;
  }

  auto require = [&out](
                     const std::optional<double>& value,
                     const char* name
                 ) {
    if (!value.has_value()) {
      out.missing_fields.emplace_back(name);
    }
  };

  require(
      it->translation_x_m,
      "adapter.translation_x_m"
  );
  require(
      it->translation_y_m,
      "adapter.translation_y_m"
  );
  require(
      it->translation_z_m,
      "adapter.translation_z_m"
  );
  require(it->roll_deg, "adapter.roll_deg");
  require(it->pitch_deg, "adapter.pitch_deg");
  require(it->yaw_deg, "adapter.yaw_deg");

  if (!out.missing_fields.empty()) {
    out.status =
        NativeFrameResolutionStatus::AdapterIncomplete;
    return out;
  }

  if (!finite(it->translation_x_m) ||
      !finite(it->translation_y_m) ||
      !finite(it->translation_z_m) ||
      !finite(it->roll_deg) ||
      !finite(it->pitch_deg) ||
      !finite(it->yaw_deg)) {
    out.status =
        NativeFrameResolutionStatus::AdapterNonFinite;
    return out;
  }

  const auto mount =
      resolve_sensor_mount(sensor_mount);
  if (mount.status == CalibrationStatus::Incomplete) {
    out.status =
        NativeFrameResolutionStatus::MountIncomplete;
    out.missing_fields = mount.missing_fields;
    return out;
  }
  if (mount.status == CalibrationStatus::NonFinite) {
    out.status =
        NativeFrameResolutionStatus::MountNonFinite;
    return out;
  }

  const RigidTransform sensor_from_native =
      from_rpy_xyz(
          *it->translation_x_m,
          *it->translation_y_m,
          *it->translation_z_m,
          *it->roll_deg,
          *it->pitch_deg,
          *it->yaw_deg
      );

  out.status = NativeFrameResolutionStatus::Valid;
  out.native_to_vehicle_track =
      compose(
          *mount.sensor_to_vehicle_track,
          sensor_from_native
      );
  return out;
}

}  // namespace metropilot
