#pragma once

#include "metropilot/swept_volume.hpp"

#include <string>

namespace metropilot {

// Parse the runtime-oriented-box subset of vehicle_geometry.yaml.
//
// The geometry contract may contain additional documentation/source fields;
// this parser intentionally consumes only the kinematic dimensions and
// runtime_lod.components needed by the sweep engine. Unsupported primitives
// or incomplete components are rejected rather than silently approximated.
VehicleSweepGeometry parse_vehicle_sweep_geometry_yaml(
    const std::string& yaml_text
);

VehicleSweepGeometry load_vehicle_sweep_geometry_yaml(
    const std::string& path
);

}  // namespace metropilot
