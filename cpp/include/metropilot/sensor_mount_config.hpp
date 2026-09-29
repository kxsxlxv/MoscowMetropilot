#pragma once

#include "metropilot/geometry.hpp"

#include <string>
#include <vector>

namespace metropilot {

struct SensorCalibrationConfig {
  SensorMountCalibration physical_mount;
  std::vector<NativeFrameAdapterCalibration> runtime_adapters;
  std::vector<std::string> configured_frame_ids;

  bool trust_frame_id_as_extrinsic_calibration{false};
  bool require_explicit_native_frame_adapter{true};
  bool require_complete_physical_sensor_mount{true};
  bool require_both_transforms_before_physical_swept_volume_decision{true};
  bool permit_research_only_adapter_hypotheses_in_submission{false};

  // Explicit challenge/submission override for assumptions that cannot be
  // independently calibrated from the supplied organizer bags. This does not
  // make an assumed value measured/calibrated; callers can inspect
  // uses_assumed_calibration for provenance.
  bool permit_explicit_assumed_calibration_in_submission{false};
  bool uses_assumed_calibration{false};
};

// Parse only the production runtime subset of config/sensor_mount.yaml.
// Research/development adapter hypotheses are intentionally ignored.
SensorCalibrationConfig parse_sensor_calibration_yaml(
    const std::string& yaml_text
);

SensorCalibrationConfig load_sensor_calibration_yaml(
    const std::string& path
);

// Resolve using the explicitly configured runtime adapter and physical mount.
// Known frames with an incomplete runtime_adapter fail as AdapterIncomplete;
// unknown frame IDs fail as UnknownFrame.
NativeFrameResolution resolve_configured_native_frame_to_vehicle(
    const std::string& frame_id,
    const SensorCalibrationConfig& config
);

}  // namespace metropilot
