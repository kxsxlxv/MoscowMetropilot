#pragma once

#include "metropilot/candidate_grouping.hpp"
#include "metropilot/current_scan_classifier.hpp"
#include "metropilot/structural_residual.hpp"
#include "metropilot/sweep_evidence_gate.hpp"

#include <string>
#include <vector>

namespace metropilot {

enum class DetectorStatus {
  Valid,
  Degraded,
  Unknown,
};

struct CurrentScanDetectorConfig {
  StructuralResidualConfig structural;
  CandidateGroupingConfig grouping;
  CurrentScanClassifierConfig classifier;
};

struct CurrentScanTimings {
  double sweep_gate_ms{};
  double structural_residual_ms{};
  double supported_filter_ms{};
  double grouping_ms{};
  double classifier_ms{};
};

struct CurrentScanResult {
  DetectorStatus status{DetectorStatus::Unknown};
  bool obstacle_present{false};
  bool tentative_present{false};
  bool branch_ambiguous{false};

  double supported_path_distance_m{};
  std::vector<std::string> reason_codes;

  std::size_t sweep_seed_count{};
  std::size_t unsupported_path_seed_count{};
  std::size_t candidate_count{};
  std::size_t structural_candidate_count{};
  std::size_t tentative_candidate_count{};
  std::size_t supported_obstacle_candidate_count{};

  std::vector<SweepEvidenceSeed> seeds;
  std::vector<StructuralResidual> residuals;
  std::vector<CurrentScanCandidate> candidates;
  CurrentScanClassification classification;
  CurrentScanTimings timings;
};

class CurrentScanDetector {
 public:
  explicit CurrentScanDetector(
      CurrentScanDetectorConfig config = {}
  );

  CurrentScanResult run(
      const std::vector<VehicleEvidenceEndpoint>& endpoints,
      const RailSupportResult& rail_support,
      const LocalTrackHypothesis& track,
      const PossibleSweptVolume& possible_sweep,
      const PossibleSweptVolume* nominal_sweep = nullptr
  ) const;

  CurrentScanResult run(
      const std::vector<VehicleEvidenceEndpoint>& endpoints,
      const LocalTrackHypothesisSet& hypotheses,
      const PossibleSweptVolume& possible_sweep,
      const PossibleSweptVolume* nominal_sweep = nullptr
  ) const;

 private:
  StructuralResidualEvaluator structural_;
  CandidateGrouper grouper_;
  CurrentScanClassifier classifier_;
};

}  // namespace metropilot
