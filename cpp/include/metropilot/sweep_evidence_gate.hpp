#pragma once

#include "metropilot/scene_projection.hpp"
#include "metropilot/swept_volume.hpp"

#include <cstddef>
#include <vector>

namespace metropilot {

struct SweepEvidenceSeed {
  std::size_t endpoint_index{};
  std::vector<std::size_t> matching_branch_indices;
};

std::vector<SweepEvidenceSeed>
gate_evidence_to_possible_sweep(
    const std::vector<VehicleEvidenceEndpoint>& endpoints,
    const PossibleSweptVolume& possible_sweep
);

}  // namespace metropilot
