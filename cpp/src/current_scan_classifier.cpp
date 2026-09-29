#include "metropilot/current_scan_classifier.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace metropilot {

CurrentScanClassifier::CurrentScanClassifier(
    CurrentScanClassifierConfig config
) : config_(std::move(config)) {
  if (config_.strong_min_known_firings == 0 ||
      config_.strong_min_known_acquisition_groups == 0 ||
      config_.strong_min_unknown_endpoints == 0 ||
      config_.pure_rail_fraction < 0.0 ||
      config_.pure_rail_fraction > 1.0 ||
      config_.broad_low_surface_min_endpoints == 0 ||
      config_.broad_low_surface_min_longitudinal_span_m < 0.0 ||
      config_.broad_low_surface_max_vertical_span_m < 0.0 ||
      config_.broad_low_surface_min_low_zone_fraction < 0.0 ||
      config_.broad_low_surface_min_low_zone_fraction > 1.0) {
    throw std::invalid_argument(
        "invalid CurrentScanClassifierConfig"
    );
  }
}

CurrentScanClassification
CurrentScanClassifier::classify(
    const std::vector<CurrentScanCandidate>& candidates
) const {
  CurrentScanClassification out;
  out.assessments.reserve(candidates.size());

  for (std::size_t i = 0;
       i < candidates.size();
       ++i) {
    const auto& candidate = candidates[i];
    CandidateAssessment assessment;
    assessment.candidate_index = i;

    if (candidate.endpoint_count == 0) {
      throw std::invalid_argument(
          "candidate with zero endpoints"
      );
    }

    const bool pure_rail =
        candidate.rail_compatible_fraction >=
        config_.pure_rail_fraction;
    const bool pure_soft_structure =
        candidate.soft_structural_fraction >=
        config_.pure_rail_fraction;

    const bool broad_low_surface =
        candidate.endpoint_count >=
            config_.broad_low_surface_min_endpoints &&
        candidate.longitudinal_span_m >=
            config_.broad_low_surface_min_longitudinal_span_m &&
        candidate.vertical_span_m <=
            config_.broad_low_surface_max_vertical_span_m &&
        candidate.low_track_zone_fraction >=
            config_.broad_low_surface_min_low_zone_fraction;

    if (pure_soft_structure) {
      assessment.disposition =
          CandidateDisposition::StructuralCompatible;
      assessment.reason =
          pure_rail
              ? "all endpoints compatible with observed rail support"
              : "all endpoints compatible with current-scan structural support";
      assessment.effective_support_count =
          candidate.known_independent_firings;
      assessment.support_identity_fully_known =
          candidate.unknown_identity_endpoints == 0;
      out.assessments.push_back(
          std::move(assessment)
      );
      continue;
    }

    if (broad_low_surface) {
      assessment.disposition =
          CandidateDisposition::StructuralCompatible;
      assessment.reason =
          "broad longitudinally extended low surface";
      assessment.effective_support_count =
          candidate.known_independent_firings;
      assessment.support_identity_fully_known =
          candidate.unknown_identity_endpoints == 0;
      out.assessments.push_back(
          std::move(assessment)
      );
      continue;
    }

    const std::size_t non_structural_endpoints =
        candidate.endpoint_count -
        candidate.soft_structural_endpoints;
    const bool all_identity_known =
        candidate.non_structural_unknown_identity_endpoints == 0;

    std::size_t effective_support = 0;
    std::size_t required_support = 0;
    std::size_t effective_acquisition_groups = 0;
    bool acquisition_diversity_required = false;
    bool same_scan_confirmation_allowed = true;

    if (all_identity_known) {
      effective_support =
          candidate.non_structural_known_independent_firings;
      required_support =
          config_.strong_min_known_firings;
      effective_acquisition_groups =
          candidate.non_structural_known_firing_timestamp_groups;
      acquisition_diversity_required = true;
    } else if (
        candidate.non_structural_known_independent_firings == 0) {
      // Plain XYZI fixture: do not pretend these endpoints are proven
      // independent. The count can still exercise geometry logic, but the
      // assessment records that independence is unknown.
      effective_support =
          candidate.non_structural_unknown_identity_endpoints;
      required_support =
          config_.strong_min_unknown_endpoints;
      same_scan_confirmation_allowed =
          config_.allow_unknown_identity_same_scan_confirmation;
    } else {
      // Mixed identity quality is conservatively based only on known physical
      // firings. Unknown endpoints remain available but do not strengthen a
      // same-scan confirmation.
      effective_support =
          candidate.non_structural_known_independent_firings;
      required_support =
          config_.strong_min_known_firings;
      effective_acquisition_groups =
          candidate.non_structural_known_firing_timestamp_groups;
      acquisition_diversity_required = true;
    }

    assessment.effective_support_count =
        effective_support;
    assessment.effective_acquisition_group_count =
        effective_acquisition_groups;
    assessment.support_identity_fully_known =
        all_identity_known;

    const bool acquisition_diversity_sufficient =
        !acquisition_diversity_required ||
        effective_acquisition_groups >=
            config_.strong_min_known_acquisition_groups;

    // A possible-sweep uncertainty shell means the point may intersect the
    // vehicle only under the currently modeled geometric uncertainty. Such
    // evidence must remain visible, but it is not a nominally supported
    // collision. Do not turn uncertainty expansion into a confirmed obstacle.
    const bool sweep_membership_observable =
        candidate.non_structural_nominal_sweep_endpoints > 0 ||
        candidate.non_structural_uncertainty_shell_only_endpoints > 0;
    const bool uncertainty_shell_only =
        sweep_membership_observable &&
        candidate.non_structural_nominal_sweep_endpoints == 0 &&
        candidate.non_structural_uncertainty_shell_only_endpoints > 0;

    if (non_structural_endpoints > 0 &&
        same_scan_confirmation_allowed &&
        effective_support >= required_support &&
        acquisition_diversity_sufficient &&
        !uncertainty_shell_only) {
      assessment.disposition =
          CandidateDisposition::SupportedObstacle;
      assessment.reason =
          all_identity_known
              ? "non-structural support from multiple independent firings across acquisition groups"
              : "non-structural support with unknown firing identity";

      out.obstacle_present = true;
      if (!out.nearest_supported_obstacle_range_m ||
          candidate.nearest_range_m <
              *out.nearest_supported_obstacle_range_m) {
        out.nearest_supported_obstacle_range_m =
            candidate.nearest_range_m;
      }
    } else {
      assessment.disposition =
          CandidateDisposition::TentativeObstacle;
      assessment.reason =
          non_structural_endpoints == 0
              ? "structural-compatible evidence without enough protruding residual"
              : (uncertainty_shell_only
                    ? "non-structural evidence intersects uncertainty shell only"
                    : (!same_scan_confirmation_allowed
                          ? "firing identity unknown; same-scan confirmation disabled"
                          : (!acquisition_diversity_sufficient
                                ? "non-structural evidence lacks acquisition-group diversity"
                                : "non-structural evidence below same-scan confirmation support")));

      out.tentative_present = true;
      if (!out.nearest_tentative_range_m ||
          candidate.nearest_range_m <
              *out.nearest_tentative_range_m) {
        out.nearest_tentative_range_m =
            candidate.nearest_range_m;
      }
    }

    out.assessments.push_back(
        std::move(assessment)
    );
  }

  return out;
}

}  // namespace metropilot
