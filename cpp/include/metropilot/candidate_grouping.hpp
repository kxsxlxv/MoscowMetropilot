#pragma once

#include "metropilot/scene_projection.hpp"
#include "metropilot/structural_residual.hpp"
#include "metropilot/sweep_evidence_gate.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace metropilot {

struct CandidateGroupingConfig {
  // Neighbor radius = clamp(base + range*tan(angular), min, max).
  double base_radius_m{0.03};
  double angular_radius_deg{0.15};
  double min_neighbor_radius_m{0.05};
  double max_neighbor_radius_m{0.35};

  // Two endpoints that are both explained by current-scan structural support
  // are not chained directly. This avoids converting rail/trackbed support into
  // giant obstacle candidates while still allowing support-compatible points
  // to attach to a protruding residual.
  bool prevent_soft_structural_to_structural_links{true};
};

struct CandidateBBox {
  Vec3 min;
  Vec3 max;
};

struct CurrentScanCandidate {
  // Optional pipeline measurement for precision confirmation in all schemas: occupied
  // 3 cm cells of non-structural endpoints inside the nominal sweep.
  std::size_t non_structural_nominal_spatial_cells{};
  // Of those cells, evidence at least 5 cm above the locally measured TOR.
  std::size_t non_structural_nominal_above_rail_spatial_cells{};
  std::vector<std::size_t> seed_indices;
  std::vector<std::size_t> endpoint_indices;
  std::vector<std::size_t> matching_branch_indices;

  std::size_t endpoint_count{};
  std::size_t known_independent_firings{};
  // Distinct recorded PointCloud2 timestamp groups represented by known
  // (timestamp, ring) firings. This is scan-phase/acquisition diversity, not
  // exact per-channel firetime.
  std::size_t known_firing_timestamp_groups{};
  std::size_t unknown_identity_endpoints{};

  // Confirmation diversity must be computed from protruding/non-structural
  // evidence only. Structural endpoints may attach spatially to a candidate,
  // but they must never strengthen obstacle confirmation.
  std::size_t non_structural_known_independent_firings{};
  std::size_t non_structural_known_firing_timestamp_groups{};
  std::size_t non_structural_unknown_identity_endpoints{};
  std::uint64_t raw_return_multiplicity{};

  std::size_t rail_compatible_endpoints{};
  std::size_t longitudinal_support_compatible_endpoints{};
  std::size_t soft_structural_endpoints{};
  std::size_t low_track_zone_endpoints{};
  std::size_t nominal_sweep_endpoints{};
  std::size_t uncertainty_shell_only_endpoints{};
  std::size_t non_structural_nominal_sweep_endpoints{};
  std::size_t non_structural_uncertainty_shell_only_endpoints{};

  double nearest_range_m{};
  double mean_range_m{};
  Vec3 centroid;
  CandidateBBox bbox;

  double longitudinal_span_m{};
  double lateral_span_m{};
  double vertical_span_m{};
  double rail_compatible_fraction{};
  double longitudinal_support_compatible_fraction{};
  double soft_structural_fraction{};
  double low_track_zone_fraction{};
};

class CandidateGrouper {
 public:
  explicit CandidateGrouper(
      CandidateGroupingConfig config = {}
  );

  std::vector<CurrentScanCandidate> group(
      const std::vector<VehicleEvidenceEndpoint>& endpoints,
      const std::vector<SweepEvidenceSeed>& seeds,
      const std::vector<StructuralResidual>& residuals
  ) const;

 private:
  CandidateGroupingConfig config_;
};

}  // namespace metropilot
