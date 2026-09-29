#include "metropilot/detector_pipeline.hpp"
#include "metropilot/evidence.hpp"
#include "metropilot/scene_projection.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <array>
#include <set>
#include <stdexcept>
#include <utility>

namespace metropilot {
namespace {
struct ReplayPath {
  PolylinePath path;
  double current_chainage_m{};
  double supported_chainage_m{};

  ReplayPath(
      PolylinePath p,
      double current,
      double supported
  ) : path(std::move(p)),
      current_chainage_m(current),
      supported_chainage_m(supported) {}
};

ReplayPath build_replay_path(
    const metropilot::LocalTrackHypothesis& track
) {
  constexpr double backward_m = 20.0;
  constexpr double step_m = 0.25;

  const double max_x =
      std::max(track.supported_lookahead_m, 0.0);

  std::vector<PathSample> samples;
  double chainage = 0.0;
  std::optional<Vec3> previous;
  double current_chainage = 0.0;
  double supported_chainage = 0.0;

  std::vector<double> xs;
  for (double x = -backward_m;
       x <= max_x;
       x += step_m) {
    xs.push_back(x);
  }
  if (xs.empty() ||
      std::abs(xs.back() - max_x) > 1e-9) {
    xs.push_back(max_x);
  }

  bool current_recorded = false;
  for (const double x : xs) {
    const double y =
        track.lateral_a_inv_m * x * x +
        track.lateral_b * x +
        track.lateral_c_m;
    const double z =
        track.vertical_m * x +
        track.vertical_c_m;
    const Vec3 point{x, y, z};

    if (previous.has_value()) {
      chainage += std::hypot(
          point.x - previous->x,
          point.y - previous->y
      );
    }
    samples.push_back(
        PathSample{chainage, point}
    );

    if (!current_recorded && x >= 0.0) {
      if (x == 0.0) {
        current_chainage = chainage;
      } else {
        const auto& before = samples[samples.size() - 2];
        const double segment =
            chainage - before.chainage_m;
        const double t =
            (0.0 - before.center.x) /
            (point.x - before.center.x);
        current_chainage =
            before.chainage_m + t * segment;
      }
      current_recorded = true;
    }
    if (std::abs(x - max_x) <= 1e-9) {
      supported_chainage = chainage;
    }

    previous = point;
  }

  if (samples.size() < 2 ||
      !(supported_chainage > current_chainage)) {
    throw std::runtime_error(
        "unable to construct replay path"
    );
  }

  return ReplayPath(
      PolylinePath(std::move(samples)),
      current_chainage,
      supported_chainage
  );
}

}  // namespace
CurrentScanResult detect_cloud(
    const PointCloudBuffer& cloud, const RigidTransform& native_to_vehicle,
    const VehicleSweepGeometry& geometry, double front_pivot_offset_m) {
  PointCloudDecoder decoder(PointCloudDecoderConfig{false});
  const auto decoded = decoder.decode(cloud);
  if (!decoded.ok) throw std::runtime_error(decoded.error);
  const auto evidence = EvidenceBuilder(EvidenceBuilderConfig{false}).build(decoded.cloud);
  std::vector<Vec3> positions;
  const auto endpoints = project_evidence_to_vehicle(evidence, native_to_vehicle, &positions);
  const auto hypotheses = LocalTrackHypothesisExtractor().estimate(positions);
  CurrentScanResult invalid;
  if (hypotheses.branches.empty()) return invalid;
  std::vector<ReplayPath> paths;
  paths.reserve(hypotheses.branches.size());
  for (const auto& branch : hypotheses.branches) {
    if (!branch.track.valid) return invalid;
    paths.push_back(build_replay_path(branch.track));
  }
  std::vector<SweepBuildRequest> requests;
  for (std::size_t i = 0; i < paths.size(); ++i) {
    requests.push_back(SweepBuildRequest{&paths[i].path, i,
        paths[i].current_chainage_m + front_pivot_offset_m,
        paths[i].supported_chainage_m + front_pivot_offset_m, 0.5, 0.0});
  }
  const auto nominal = build_possible_swept_volume(requests, geometry, SweepMargins{});
  const auto possible = build_possible_swept_volume(requests, geometry, SweepMargins{0.15, 0.10, 0.20});
  auto result = CurrentScanDetector().run(endpoints, hypotheses, possible, &nominal);
  for (auto& candidate : result.candidates) {
    std::set<std::array<long long, 3>> cells;
    std::set<std::array<long long, 3>> above_rail_cells;
    for (const auto seed_index : candidate.seed_indices) {
      const auto& residual = result.residuals.at(seed_index);
      if (residual.rail_compatible || residual.longitudinal_support_compatible ||
          residual.uncertainty_shell_longitudinal_support_compatible ||
          !residual.inside_nominal_sweep) continue;
      const auto& p = endpoints.at(result.seeds.at(seed_index).endpoint_index).position;
      const std::array<long long, 3> cell{
          static_cast<long long>(std::floor(p.x / 0.03)),
          static_cast<long long>(std::floor(p.y / 0.03)),
          static_cast<long long>(std::floor(p.z / 0.03))};
      cells.insert(cell);
      if (residual.branch_reference_available && residual.height_from_tor_m >= 0.05)
        above_rail_cells.insert(cell);
    }
    candidate.non_structural_nominal_spatial_cells = cells.size();
    candidate.non_structural_nominal_above_rail_spatial_cells = above_rail_cells.size();
  }
  return result;
}
}  // namespace metropilot
