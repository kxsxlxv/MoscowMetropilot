#pragma once

#include "metropilot/geometry.hpp"
#include "metropilot/sweep_geometry.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace metropilot {

// "Rail support" intentionally means a persistent low-height ridge consistent
// with the two running rails. It is not claimed to be the geometric rail
// centreline or the 1520 mm inner-face gauge reference.
struct RailSupportPair {
  double forward_m{};
  double right_y_m{};
  double left_y_m{};
  double right_z_m{};
  double left_z_m{};
  double separation_m{};
  std::size_t right_support_points{};
  std::size_t left_support_points{};
  double prediction_error_m{};
  bool sparse_extension{false};
};

struct RailSupportConfig {
  double forward_min_m{1.0};
  double forward_max_m{40.0};
  double forward_bin_m{2.0};

  // Input points are in vehicle_track_reference:
  // +x forward, +y left, +z up, z=0 at TOR.
  double lateral_abs_max_m{2.8};
  double tor_z_min_m{-0.15};
  double tor_z_max_m{0.12};

  double lateral_histogram_bin_m{0.02};
  double refine_half_width_m{0.04};
  std::size_t min_peak_points{2};

  // Deliberately broad: the extractor is following rail-like paired ridges,
  // not asserting that their separation equals the 1520 mm inner-face gauge.
  double min_pair_separation_m{1.35};
  double max_pair_separation_m{1.85};

  double initial_right_y_m{-0.8};
  double initial_left_y_m{0.8};
  double max_pair_prediction_error_m{0.70};
  double continuity_penalty_points_per_m{80.0};

  // Once a stable pair has been established, a large separation jump is more
  // likely a switch to unrelated infrastructure than a physical gauge change.
  double max_separation_drift_m{0.15};
  std::size_t separation_baseline_min_pairs{3};

  std::size_t max_consecutive_missing_bins{1};

  // After the dense near-field chain ends, continue only from measured
  // paired rail evidence gated by the fitted lateral and vertical trends.
  bool enable_sparse_extension{true};
  std::size_t sparse_min_seed_pairs{4};
  double sparse_forward_max_m{100.0};
  double sparse_medium_start_m{30.0};
  double sparse_far_start_m{60.0};
  double sparse_near_bin_m{5.0};
  double sparse_medium_bin_m{10.0};
  double sparse_far_bin_m{20.0};

  double sparse_lateral_abs_max_m{4.0};
  double sparse_lateral_base_window_m{0.06};
  double sparse_lateral_window_per_m{0.002};
  double sparse_lateral_max_window_m{0.28};

  double sparse_vertical_base_window_m{0.05};
  double sparse_vertical_window_per_m{0.001};
  double sparse_vertical_max_window_m{0.18};

  std::size_t sparse_min_points_per_rail{2};
  double sparse_min_forward_spread_m{0.20};
  double sparse_max_separation_drift_m{0.18};
  std::size_t sparse_max_consecutive_missing_bins{1};
};

struct RailSupportResult {
  std::vector<RailSupportPair> pairs;
  double supported_lookahead_m{};
  std::size_t visited_bins{};
  std::size_t rejected_or_missing_bins{};
  std::size_t sparse_extension_bins{};
  std::size_t sparse_extension_pairs{};
};

class RailSupportExtractor {
 public:
  explicit RailSupportExtractor(
      RailSupportConfig config = {}
  );

  RailSupportResult extract(
      const std::vector<Vec3>& vehicle_points
  ) const;

 private:
  RailSupportConfig config_;
};

struct TrackFitQuality {
  double lateral_rms_m{};
  double vertical_rms_m{};
  double rail_separation_median_m{};
  double rail_separation_mad_m{};
  std::size_t support_pair_count{};
  std::size_t total_support_points{};
};

struct LocalTrackHypothesis {
  bool valid{false};
  std::string reason;

  // Local polynomial at the vehicle origin:
  // y(x) = lateral_a*x^2 + lateral_b*x + lateral_c
  // z(x) = vertical_m*x + vertical_c
  double lateral_a_inv_m{};
  double lateral_b{};
  double lateral_c_m{};
  double vertical_m{};
  double vertical_c_m{};

  double heading_rad{};
  double curvature_inv_m{};
  double grade{};
  double cant_rad{};
  double supported_lookahead_m{};

  TrackFitQuality quality;
  std::vector<PathSample> centerline_samples;
};

struct LocalTrackEstimatorConfig {
  std::size_t min_support_pairs{4};
  double min_supported_lookahead_m{7.0};
  double max_lateral_rms_m{0.12};
  double max_vertical_rms_m{0.12};
};

class LocalTrackEstimator {
 public:
  explicit LocalTrackEstimator(
      LocalTrackEstimatorConfig config = {}
  );

  LocalTrackHypothesis estimate(
      const RailSupportResult& supports
  ) const;

 private:
  LocalTrackEstimatorConfig config_;
};

// Conservative multi-hypothesis layer for unresolved turnout geometry.
// The primary branch is the ordinary greedy rail-support result above.
// Secondary branches are admitted only when a measured paired-rail chain
// starts close enough to the primary path to be turnout-connected and then
// develops sustained lateral divergence. This intentionally rejects a merely
// parallel pair of ridges as a turnout hypothesis.
struct LocalTrackHypothesisSetConfig {
  RailSupportConfig rail_support;
  LocalTrackEstimatorConfig track_fit;

  double branch_forward_min_m{6.0};
  double branch_forward_max_m{60.0};
  double branch_bin_m{2.0};
  double branch_lateral_abs_max_m{4.0};
  double branch_vertical_window_m{0.18};
  // Merge nearby histogram maxima before forming rail pairs. A rail head or
  // other broad ridge often creates several 2 cm local maxima; pairing those
  // maxima independently can fabricate a path between adjacent parallel tracks.
  double branch_peak_merge_distance_m{0.12};

  double branch_min_lateral_divergence_m{0.10};
  double branch_seed_max_lateral_divergence_m{0.25};
  double branch_min_final_lateral_divergence_m{0.50};
  double branch_min_divergence_growth_m{0.35};

  double branch_max_prediction_error_m{0.25};
  double branch_max_separation_drift_m{0.15};
  std::size_t branch_min_pairs{5};
  double branch_min_span_m{8.0};
  std::size_t branch_max_missing_bins{1};
  std::size_t max_secondary_hypotheses{2};
};

struct TrackBranchHypothesis {
  bool primary{false};
  double branch_onset_m{};
  double branch_final_offset_m{};
  RailSupportResult support;
  LocalTrackHypothesis track;
};

struct LocalTrackHypothesisSet {
  std::vector<TrackBranchHypothesis> branches;
  bool branch_ambiguous{false};
};

class LocalTrackHypothesisExtractor {
 public:
  explicit LocalTrackHypothesisExtractor(
      LocalTrackHypothesisSetConfig config = {}
  );

  LocalTrackHypothesisSet estimate(
      const std::vector<Vec3>& vehicle_points
  ) const;

 private:
  LocalTrackHypothesisSetConfig config_;
  RailSupportExtractor primary_extractor_;
  LocalTrackEstimator estimator_;
};

}  // namespace metropilot
