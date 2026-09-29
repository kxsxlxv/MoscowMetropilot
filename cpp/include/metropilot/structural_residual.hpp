#pragma once

#include "metropilot/scene_projection.hpp"
#include "metropilot/sweep_evidence_gate.hpp"
#include "metropilot/track_estimation.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace metropilot {

enum class RailIdentity {
  None,
  Right,
  Left,
};

struct StructuralResidualConfig {
  // Elliptical tube around the observed rail-support ridges. This is a soft
  // compatibility feature only; points in the tube are never hard-deleted.
  double rail_lateral_tolerance_m{0.07};
  double rail_vertical_tolerance_m{0.07};

  // Descriptive low-track-zone feature used by candidate morphology.
  double low_zone_min_height_m{-0.30};
  double low_zone_max_height_m{0.12};
  double low_zone_half_width_m{1.80};

  // Do not extrapolate rail-support observations far beyond measured bins.
  double rail_reference_max_extrapolation_m{1.0};

  // Current-scan longitudinal continuity model for low track structures.
  // Cross-section cells that repeat over several forward bins are marked as
  // structurally compatible. This remains a soft feature: no point is deleted.
  double continuity_zone_min_height_m{-0.30};
  double continuity_zone_max_height_m{0.45};
  double continuity_zone_half_width_m{2.20};

  double continuity_forward_bin_m{0.50};
  double continuity_lateral_bin_m{0.08};
  double continuity_vertical_bin_m{0.06};
  int continuity_cross_section_neighbor_bins{1};
  std::size_t continuity_min_forward_bins{4};
  double continuity_min_span_m{2.00};

  // High side infrastructure (cable trays, wall/service structures) is allowed
  // to become soft-structural only when it repeats for a much longer distance.
  // The central vehicle corridor is intentionally excluded so a hanging or
  // tall central obstacle cannot disappear merely because it is vertically high.
  double side_continuity_min_abs_lateral_m{1.15};
  double side_continuity_max_abs_lateral_m{2.50};
  double side_continuity_zone_min_height_m{-0.60};
  double side_continuity_zone_max_height_m{3.80};
  // Side infrastructure can drift slowly in path-relative Y/Z on curves and
  // with perspective. Use a coarser cross-section lattice than the central
  // low-track continuity rule, while retaining the long forward-span gate.
  double side_continuity_lateral_bin_m{0.12};
  double side_continuity_vertical_bin_m{0.12};
  int side_continuity_cross_section_neighbor_bins{2};
  std::size_t side_continuity_min_forward_bins{8};
  double side_continuity_min_span_m{4.00};

  // A second, stricter continuity rule applies only to points that lie inside
  // the uncertainty-expanded possible sweep but outside the nominal sweep.
  // This is intended for continuous tunnel/trackside boundary infrastructure,
  // not for evidence inside the nominal vehicle occupancy.
  double shell_continuity_zone_min_height_m{-0.60};
  double shell_continuity_zone_max_height_m{3.80};
  double shell_continuity_zone_half_width_m{2.50};
  double shell_continuity_lateral_bin_m{0.12};
  double shell_continuity_vertical_bin_m{0.12};
  int shell_continuity_cross_section_neighbor_bins{2};
  std::size_t shell_continuity_min_forward_bins{8};
  double shell_continuity_min_span_m{4.00};
};

struct StructuralResidual {
  std::size_t endpoint_index{};
  bool branch_reference_available{false};
  std::size_t reference_branch_index{};
  std::size_t matching_branch_count{};

  bool within_supported_path{false};
  double path_center_y_m{};
  double path_tor_z_m{};
  double lateral_from_path_m{};
  double height_from_tor_m{};

  bool rail_reference_available{false};
  RailIdentity nearest_rail{RailIdentity::None};
  double rail_lateral_residual_m{};
  double rail_vertical_residual_m{};
  double rail_normalized_distance{};

  bool rail_compatible{false};
  bool low_track_zone{false};

  bool nominal_sweep_available{false};
  bool inside_nominal_sweep{false};
  bool uncertainty_shell_only{false};

  bool longitudinal_support_compatible{false};
  bool uncertainty_shell_longitudinal_support_compatible{false};
  std::size_t longitudinal_support_forward_bins{};
  double longitudinal_support_span_m{};
};

class StructuralResidualEvaluator {
 public:
  explicit StructuralResidualEvaluator(
      StructuralResidualConfig config = {}
  );

  std::vector<StructuralResidual> evaluate(
      const std::vector<VehicleEvidenceEndpoint>& endpoints,
      const std::vector<SweepEvidenceSeed>& seeds,
      const RailSupportResult& rail_support,
      const LocalTrackHypothesis& track,
      const PossibleSweptVolume* nominal_sweep = nullptr
  ) const;

  // Branch-aware variant. LocalTrackHypothesisSet::branches must use the same
  // indices as PossibleSweptVolume::branches. For seeds that match multiple
  // credible branches, the evaluator keeps the most structurally compatible
  // measured explanation instead of forcing all residuals into branch 0.
  std::vector<StructuralResidual> evaluate(
      const std::vector<VehicleEvidenceEndpoint>& endpoints,
      const std::vector<SweepEvidenceSeed>& seeds,
      const LocalTrackHypothesisSet& hypotheses,
      const PossibleSweptVolume* nominal_sweep = nullptr
  ) const;

 private:
  StructuralResidualConfig config_;
};

}  // namespace metropilot
