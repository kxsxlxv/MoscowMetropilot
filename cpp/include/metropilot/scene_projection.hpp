#pragma once

#include "metropilot/evidence.hpp"
#include "metropilot/geometry.hpp"

#include <vector>

namespace metropilot {

struct VehicleEvidenceEndpoint {
  Vec3 position;
  double intensity{};
  double range_from_sensor_m{};
  FiringIdentityQuality firing_identity_quality{
      FiringIdentityQuality::Unknown
  };
  std::optional<std::uint64_t> firing_id;
  // Preserved PointCloud2 point/group timestamp provenance. This is not
  // claimed to be exact channel firetime; it is sufficient to distinguish
  // separately recorded acquisition groups within one scan.
  std::optional<double> point_timestamp_s;
  std::uint32_t raw_return_multiplicity{1};
  std::uint64_t first_slot_index{};
};

std::vector<VehicleEvidenceEndpoint>
project_evidence_to_vehicle(
    const EvidenceFrame& evidence,
    const RigidTransform& sensor_to_vehicle_track,
    std::vector<Vec3>* positions_out = nullptr
);

std::vector<Vec3> positions_only(
    const std::vector<VehicleEvidenceEndpoint>& endpoints
);

}  // namespace metropilot
