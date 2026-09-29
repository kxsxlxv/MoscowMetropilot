#pragma once
#include "metropilot/current_scan_detector.hpp"
#include "metropilot/pointcloud_decoder.hpp"
#include "metropilot/swept_volume.hpp"
#include "metropilot/track_estimation.hpp"

namespace metropilot {
// Same single-scan geometry as the offline replay, reusable by ROS adapters.
CurrentScanResult detect_cloud(
    const PointCloudBuffer& cloud, const RigidTransform& native_to_vehicle,
    const VehicleSweepGeometry& geometry,
    double front_pivot_offset_m = -3.960);
}
