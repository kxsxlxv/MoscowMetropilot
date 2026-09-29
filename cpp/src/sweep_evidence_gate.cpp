#include "metropilot/sweep_evidence_gate.hpp"

namespace metropilot {

std::vector<SweepEvidenceSeed>
gate_evidence_to_possible_sweep(
    const std::vector<VehicleEvidenceEndpoint>& endpoints,
    const PossibleSweptVolume& possible_sweep
) {
  std::vector<SweepEvidenceSeed> out;

  for (std::size_t endpoint_index = 0;
       endpoint_index < endpoints.size();
       ++endpoint_index) {
    SweepEvidenceSeed seed;
    seed.endpoint_index = endpoint_index;

    for (std::size_t branch_index = 0;
         branch_index < possible_sweep.branches.size();
         ++branch_index) {
      if (possible_sweep.branches[branch_index].contains(
              endpoints[endpoint_index].position)) {
        seed.matching_branch_indices.push_back(
            branch_index
        );
      }
    }

    if (!seed.matching_branch_indices.empty()) {
      out.push_back(std::move(seed));
    }
  }

  return out;
}

}  // namespace metropilot
