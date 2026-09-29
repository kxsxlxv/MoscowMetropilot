#include "metropilot/scene_projection.hpp"

namespace metropilot {

std::vector<VehicleEvidenceEndpoint>
project_evidence_to_vehicle(
    const EvidenceFrame& evidence,
    const RigidTransform& transform,
    std::vector<Vec3>* positions_out
) {
  std::vector<VehicleEvidenceEndpoint> out;
  out.reserve(evidence.endpoints.size());
  if (positions_out != nullptr) {
    positions_out->clear();
    positions_out->reserve(evidence.endpoints.size());
  }

  for (const auto& endpoint : evidence.endpoints) {
    VehicleEvidenceEndpoint projected;
    projected.position = transform.apply(
        Vec3{
            static_cast<double>(endpoint.x),
            static_cast<double>(endpoint.y),
            static_cast<double>(endpoint.z),
        }
    );
    projected.intensity = endpoint.intensity;
    projected.range_from_sensor_m = endpoint.range_m;
    projected.firing_identity_quality =
        endpoint.firing_identity_quality;
    projected.firing_id = endpoint.firing_id;
    projected.point_timestamp_s =
        endpoint.point_timestamp_s;
    projected.raw_return_multiplicity =
        endpoint.raw_return_multiplicity;
    projected.first_slot_index =
        endpoint.first_slot_index;
    if (positions_out != nullptr) {
      positions_out->push_back(projected.position);
    }
    out.push_back(projected);
  }

  return out;
}

std::vector<Vec3> positions_only(
    const std::vector<VehicleEvidenceEndpoint>& endpoints
) {
  std::vector<Vec3> out;
  out.reserve(endpoints.size());
  for (const auto& endpoint : endpoints) {
    out.push_back(endpoint.position);
  }
  return out;
}

}  // namespace metropilot
