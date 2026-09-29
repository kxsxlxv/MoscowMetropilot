#include "metropilot/current_scan_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace metropilot {
namespace {

double elapsed_ms(
    std::chrono::steady_clock::time_point begin,
    std::chrono::steady_clock::time_point end
) {
  return std::chrono::duration<double, std::milli>(
      end - begin
  ).count();
}

}  // namespace

CurrentScanDetector::CurrentScanDetector(
    CurrentScanDetectorConfig config
) : structural_(std::move(config.structural)),
    grouper_(std::move(config.grouping)),
    classifier_(std::move(config.classifier)) {}

CurrentScanResult CurrentScanDetector::run(
    const std::vector<VehicleEvidenceEndpoint>& endpoints,
    const RailSupportResult& rail_support,
    const LocalTrackHypothesis& track,
    const PossibleSweptVolume& possible_sweep,
    const PossibleSweptVolume* nominal_sweep
) const {
  CurrentScanResult out;
  out.branch_ambiguous =
      possible_sweep.branch_ambiguous;
  out.supported_path_distance_m =
      track.supported_lookahead_m;

  if (!track.valid) {
    out.status = DetectorStatus::Unknown;
    out.reason_codes.push_back(
        "TRACK_HYPOTHESIS_INVALID"
    );
    return out;
  }

  if (possible_sweep.branches.empty()) {
    out.status = DetectorStatus::Unknown;
    out.reason_codes.push_back(
        "POSSIBLE_SWEEP_EMPTY"
    );
    return out;
  }

  out.status =
      possible_sweep.branch_ambiguous
          ? DetectorStatus::Degraded
          : DetectorStatus::Valid;

  if (possible_sweep.branch_ambiguous) {
    out.reason_codes.push_back(
        "PATH_BRANCH_AMBIGUOUS"
    );
  }
  if (rail_support.pairs.empty()) {
    out.status = DetectorStatus::Degraded;
    out.reason_codes.push_back(
        "RAIL_REFERENCE_UNAVAILABLE"
    );
  }

  const auto sweep_gate_started =
      std::chrono::steady_clock::now();
  out.seeds =
      gate_evidence_to_possible_sweep(
          endpoints,
          possible_sweep
      );
  const auto sweep_gate_finished =
      std::chrono::steady_clock::now();
  out.timings.sweep_gate_ms =
      elapsed_ms(sweep_gate_started, sweep_gate_finished);
  out.sweep_seed_count = out.seeds.size();

  if (out.seeds.empty()) {
    out.obstacle_present = false;
    out.tentative_present = false;
    return out;
  }

  const auto structural_started =
      std::chrono::steady_clock::now();
  auto all_residuals =
      structural_.evaluate(
          endpoints,
          out.seeds,
          rail_support,
          track,
          nominal_sweep
      );
  const auto structural_finished =
      std::chrono::steady_clock::now();
  out.timings.structural_residual_ms =
      elapsed_ms(structural_started, structural_finished);

  const auto filter_started =
      std::chrono::steady_clock::now();
  std::vector<SweepEvidenceSeed> supported_seeds;
  std::vector<StructuralResidual> supported_residuals;
  supported_seeds.reserve(out.seeds.size());
  supported_residuals.reserve(all_residuals.size());

  for (std::size_t i = 0; i < out.seeds.size(); ++i) {
    if (!all_residuals[i].within_supported_path) {
      ++out.unsupported_path_seed_count;
      continue;
    }
    supported_seeds.push_back(out.seeds[i]);
    supported_residuals.push_back(all_residuals[i]);
  }

  out.seeds = std::move(supported_seeds);
  out.residuals = std::move(supported_residuals);
  const auto filter_finished =
      std::chrono::steady_clock::now();
  out.timings.supported_filter_ms =
      elapsed_ms(filter_started, filter_finished);

  const auto grouping_started =
      std::chrono::steady_clock::now();
  out.candidates =
      grouper_.group(
          endpoints,
          out.seeds,
          out.residuals
      );
  const auto grouping_finished =
      std::chrono::steady_clock::now();
  out.timings.grouping_ms =
      elapsed_ms(grouping_started, grouping_finished);
  out.candidate_count = out.candidates.size();

  const auto classifier_started =
      std::chrono::steady_clock::now();
  out.classification =
      classifier_.classify(out.candidates);
  const auto classifier_finished =
      std::chrono::steady_clock::now();
  out.timings.classifier_ms =
      elapsed_ms(classifier_started, classifier_finished);
  out.obstacle_present =
      out.classification.obstacle_present;
  out.tentative_present =
      out.classification.tentative_present;

  for (const auto& assessment :
       out.classification.assessments) {
    switch (assessment.disposition) {
      case CandidateDisposition::StructuralCompatible:
        ++out.structural_candidate_count;
        break;
      case CandidateDisposition::TentativeObstacle:
        ++out.tentative_candidate_count;
        break;
      case CandidateDisposition::SupportedObstacle:
        ++out.supported_obstacle_candidate_count;
        break;
    }
  }

  return out;
}


CurrentScanResult CurrentScanDetector::run(
    const std::vector<VehicleEvidenceEndpoint>& endpoints,
    const LocalTrackHypothesisSet& hypotheses,
    const PossibleSweptVolume& possible_sweep,
    const PossibleSweptVolume* nominal_sweep
) const {
  CurrentScanResult out;
  out.branch_ambiguous =
      possible_sweep.branch_ambiguous ||
      hypotheses.branch_ambiguous;

  if (hypotheses.branches.empty()) {
    out.status = DetectorStatus::Unknown;
    out.reason_codes.push_back(
        "TRACK_HYPOTHESES_EMPTY"
    );
    return out;
  }

  if (possible_sweep.branches.empty()) {
    out.status = DetectorStatus::Unknown;
    out.reason_codes.push_back(
        "POSSIBLE_SWEEP_EMPTY"
    );
    return out;
  }

  if (possible_sweep.branches.size() !=
      hypotheses.branches.size()) {
    out.status = DetectorStatus::Unknown;
    out.reason_codes.push_back(
        "BRANCH_GEOMETRY_MISMATCH"
    );
    return out;
  }

  bool all_tracks_valid = true;
  bool all_rail_references_available = true;
  double conservative_lookahead =
      std::numeric_limits<double>::infinity();

  for (const auto& branch : hypotheses.branches) {
    if (!branch.track.valid) {
      all_tracks_valid = false;
      break;
    }
    conservative_lookahead =
        std::min(
            conservative_lookahead,
            branch.track.supported_lookahead_m
        );
    if (branch.support.pairs.empty()) {
      all_rail_references_available = false;
    }
  }

  if (!all_tracks_valid ||
      !std::isfinite(conservative_lookahead)) {
    out.status = DetectorStatus::Unknown;
    out.reason_codes.push_back(
        "TRACK_HYPOTHESIS_INVALID"
    );
    return out;
  }

  out.supported_path_distance_m =
      conservative_lookahead;
  out.status =
      out.branch_ambiguous
          ? DetectorStatus::Degraded
          : DetectorStatus::Valid;

  if (out.branch_ambiguous) {
    out.reason_codes.push_back(
        "PATH_BRANCH_AMBIGUOUS"
    );
  }
  if (!all_rail_references_available) {
    out.status = DetectorStatus::Degraded;
    out.reason_codes.push_back(
        "RAIL_REFERENCE_UNAVAILABLE"
    );
  }

  const auto sweep_gate_started =
      std::chrono::steady_clock::now();
  out.seeds =
      gate_evidence_to_possible_sweep(
          endpoints,
          possible_sweep
      );
  const auto sweep_gate_finished =
      std::chrono::steady_clock::now();
  out.timings.sweep_gate_ms =
      elapsed_ms(sweep_gate_started, sweep_gate_finished);
  out.sweep_seed_count = out.seeds.size();

  if (out.seeds.empty()) {
    return out;
  }

  const auto structural_started =
      std::chrono::steady_clock::now();
  auto all_residuals =
      structural_.evaluate(
          endpoints,
          out.seeds,
          hypotheses,
          nominal_sweep
      );
  const auto structural_finished =
      std::chrono::steady_clock::now();
  out.timings.structural_residual_ms =
      elapsed_ms(structural_started, structural_finished);

  const auto filter_started =
      std::chrono::steady_clock::now();
  std::vector<SweepEvidenceSeed> supported_seeds;
  std::vector<StructuralResidual> supported_residuals;
  supported_seeds.reserve(out.seeds.size());
  supported_residuals.reserve(all_residuals.size());

  for (std::size_t i = 0;
       i < out.seeds.size();
       ++i) {
    if (!all_residuals[i].within_supported_path) {
      ++out.unsupported_path_seed_count;
      continue;
    }
    supported_seeds.push_back(out.seeds[i]);
    supported_residuals.push_back(
        all_residuals[i]
    );
  }

  out.seeds = std::move(supported_seeds);
  out.residuals = std::move(supported_residuals);
  const auto filter_finished =
      std::chrono::steady_clock::now();
  out.timings.supported_filter_ms =
      elapsed_ms(filter_started, filter_finished);

  const auto grouping_started =
      std::chrono::steady_clock::now();
  out.candidates =
      grouper_.group(
          endpoints,
          out.seeds,
          out.residuals
      );
  const auto grouping_finished =
      std::chrono::steady_clock::now();
  out.timings.grouping_ms =
      elapsed_ms(grouping_started, grouping_finished);
  out.candidate_count = out.candidates.size();

  const auto classifier_started =
      std::chrono::steady_clock::now();
  out.classification =
      classifier_.classify(out.candidates);
  const auto classifier_finished =
      std::chrono::steady_clock::now();
  out.timings.classifier_ms =
      elapsed_ms(classifier_started, classifier_finished);
  out.obstacle_present =
      out.classification.obstacle_present;
  out.tentative_present =
      out.classification.tentative_present;

  for (const auto& assessment :
       out.classification.assessments) {
    switch (assessment.disposition) {
      case CandidateDisposition::StructuralCompatible:
        ++out.structural_candidate_count;
        break;
      case CandidateDisposition::TentativeObstacle:
        ++out.tentative_candidate_count;
        break;
      case CandidateDisposition::SupportedObstacle:
        ++out.supported_obstacle_candidate_count;
        break;
    }
  }

  return out;
}


}  // namespace metropilot
