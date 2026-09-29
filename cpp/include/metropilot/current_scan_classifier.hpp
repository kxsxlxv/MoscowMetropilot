#pragma once

#include "metropilot/candidate_grouping.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace metropilot {

enum class CandidateDisposition {
  StructuralCompatible,
  TentativeObstacle,
  SupportedObstacle,
};

struct CandidateAssessment {
  std::size_t candidate_index{};
  CandidateDisposition disposition{
      CandidateDisposition::TentativeObstacle
  };
  std::string reason;

  std::size_t effective_support_count{};
  std::size_t effective_acquisition_group_count{};
  bool support_identity_fully_known{false};
};

struct CurrentScanClassifierConfig {
  // These are confirmation thresholds, not candidate-preservation thresholds.
  // A one-endpoint candidate remains available as TentativeObstacle.
  std::size_t strong_min_known_firings{2};
  // Known firings must also span at least this many recorded acquisition
  // timestamp groups. This prevents several channels at one scan phase from
  // being treated as distributed same-scan confirmation.
  std::size_t strong_min_known_acquisition_groups{2};
  std::size_t strong_min_unknown_endpoints{2};

  // Runtime-safe default: plain XYZI endpoints do not prove independent
  // firings. Enable only in offline fixture mode to exercise geometry.
  bool allow_unknown_identity_same_scan_confirmation{false};

  // Pure observed rail-support returns are structural-compatible.
  double pure_rail_fraction{0.999};

  // Broad, longitudinally extended, vertically thin low surfaces are treated
  // as trackbed/support-like in the current scan. A transverse low object is
  // deliberately not caught by this rule.
  std::size_t broad_low_surface_min_endpoints{8};
  double broad_low_surface_min_longitudinal_span_m{1.50};
  double broad_low_surface_max_vertical_span_m{0.12};
  double broad_low_surface_min_low_zone_fraction{0.80};
};

struct CurrentScanClassification {
  bool obstacle_present{false};
  bool tentative_present{false};

  std::optional<double> nearest_supported_obstacle_range_m;
  std::optional<double> nearest_tentative_range_m;

  std::vector<CandidateAssessment> assessments;
};

class CurrentScanClassifier {
 public:
  explicit CurrentScanClassifier(
      CurrentScanClassifierConfig config = {}
  );

  CurrentScanClassification classify(
      const std::vector<CurrentScanCandidate>& candidates
  ) const;

 private:
  CurrentScanClassifierConfig config_;
};

}  // namespace metropilot
