#include "metropilot/precision_gate.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace metropilot {
PrecisionGate::PrecisionGate(PrecisionGateConfig config) : config_(config) {
  if (!config.min_known_above_rail_cells || !config.min_xyzi_above_rail_cells || !config.min_xyzi_spatial_cells || !config.min_firings || !config.min_acquisition_groups ||
      !config.min_nominal_endpoints || !config.confirmation_frames ||
      !std::isfinite(config.min_non_structural_fraction) ||
      config.min_non_structural_fraction < 0 || config.min_non_structural_fraction > 1 ||
      !std::isfinite(config.max_source_gap_s) || config.max_source_gap_s <= 0 ||
      !std::isfinite(config.max_longitudinal_speed_mps) || config.max_longitudinal_speed_mps < 0 ||
      !std::isfinite(config.match_longitudinal_slack_m) || config.match_longitudinal_slack_m <= 0 ||
      !std::isfinite(config.match_lateral_m) || config.match_lateral_m <= 0 ||
      !std::isfinite(config.match_vertical_m) || config.match_vertical_m <= 0) {
    throw std::invalid_argument("invalid precision gate configuration");
  }
}
void PrecisionGate::reset() { previous_.clear(); previous_time_.reset(); }

bool PrecisionGate::update(const CurrentScanResult& result, std::int64_t source_time_ns) {
  if (result.status != DetectorStatus::Valid || result.branch_ambiguous) {
    reset();
    return false;
  }
  double dt = 0;
  if (previous_time_) {
    dt = static_cast<double>(source_time_ns - *previous_time_) * 1e-9;
    if (dt <= 0 || dt > config_.max_source_gap_s) reset();
  }
  std::vector<Track> current;
  std::vector<bool> used(previous_.size(), false);
  bool alarm = false;
  for (const auto& assessment : result.classification.assessments) {
    const auto& c = result.candidates.at(assessment.candidate_index);
    const bool known_support =
        assessment.disposition == CandidateDisposition::SupportedObstacle &&
        c.non_structural_known_independent_firings >= config_.min_firings &&
        c.non_structural_nominal_above_rail_spatial_cells >= config_.min_known_above_rail_cells &&
        c.non_structural_known_firing_timestamp_groups >= config_.min_acquisition_groups;
    const bool spatial_support = config_.allow_xyzi &&
        config_.confirmation_frames >= 2 &&
        assessment.disposition == CandidateDisposition::TentativeObstacle &&
        c.non_structural_known_independent_firings == 0 &&
        c.non_structural_unknown_identity_endpoints > 0 &&
        c.non_structural_nominal_spatial_cells >= config_.min_xyzi_spatial_cells &&
        c.non_structural_nominal_above_rail_spatial_cells >= config_.min_xyzi_above_rail_cells;
    if (!known_support && !spatial_support) continue;
    if (!c.endpoint_count || c.soft_structural_endpoints > c.endpoint_count ||
        c.non_structural_nominal_sweep_endpoints < config_.min_nominal_endpoints ||
        static_cast<double>(c.endpoint_count - c.soft_structural_endpoints) /
            c.endpoint_count < config_.min_non_structural_fraction ||
        !std::isfinite(c.centroid.x) || !std::isfinite(c.centroid.y) ||
        !std::isfinite(c.centroid.z) || c.centroid.x < 0) continue;
    std::size_t best = previous_.size();
    double best_distance = 1e100;
    for (std::size_t i = 0; i < previous_.size(); ++i) {
      if (used[i]) continue;
      const auto& p = previous_[i].center;
      const double dx = std::abs(c.centroid.x - p.x);
      const double dy = std::abs(c.centroid.y - p.y);
      const double dz = std::abs(c.centroid.z - p.z);
      if (dx > config_.match_longitudinal_slack_m + config_.max_longitudinal_speed_mps * dt ||
          dy > config_.match_lateral_m || dz > config_.match_vertical_m) continue;
      const double distance = dx * dx + dy * dy + dz * dz;
      if (distance < best_distance) { best = i; best_distance = distance; }
    }
    std::size_t streak = 1;
    if (best < previous_.size()) {
      used[best] = true;
      streak = std::min(previous_[best].streak + 1, config_.confirmation_frames);
    }
    current.push_back({c.centroid, streak});
    alarm = alarm || streak >= config_.confirmation_frames;
  }
  previous_ = std::move(current);
  previous_time_ = source_time_ns;
  return alarm;
}
}
