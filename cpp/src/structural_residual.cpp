#include "metropilot/structural_residual.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace metropilot {
namespace {

struct RailReference {
  double right_y{};
  double right_z{};
  double left_y{};
  double left_z{};
};

std::optional<RailReference> interpolate_rail_reference(
    const RailSupportResult& support,
    double forward_m,
    double max_extrapolation_m
) {
  if (support.pairs.empty()) {
    return std::nullopt;
  }

  const auto& first = support.pairs.front();
  const auto& last = support.pairs.back();
  if (forward_m <
          first.forward_m - max_extrapolation_m ||
      forward_m >
          last.forward_m + max_extrapolation_m) {
    return std::nullopt;
  }

  if (support.pairs.size() == 1 ||
      forward_m <= first.forward_m) {
    return RailReference{
        first.right_y_m,
        first.right_z_m,
        first.left_y_m,
        first.left_z_m,
    };
  }
  if (forward_m >= last.forward_m) {
    return RailReference{
        last.right_y_m,
        last.right_z_m,
        last.left_y_m,
        last.left_z_m,
    };
  }

  const auto upper = std::upper_bound(
      support.pairs.begin(),
      support.pairs.end(),
      forward_m,
      [](double x, const RailSupportPair& pair) {
        return x < pair.forward_m;
      }
  );
  const auto& b = *upper;
  const auto& a = *(upper - 1);
  const double t =
      (forward_m - a.forward_m) /
      (b.forward_m - a.forward_m);

  auto lerp = [t](double u, double v) {
    return u + t * (v - u);
  };

  return RailReference{
      lerp(a.right_y_m, b.right_y_m),
      lerp(a.right_z_m, b.right_z_m),
      lerp(a.left_y_m, b.left_y_m),
      lerp(a.left_z_m, b.left_z_m),
  };
}

struct CrossSectionKey {
  int lateral_bin{};
  int vertical_bin{};

  bool operator==(const CrossSectionKey& other) const {
    return lateral_bin == other.lateral_bin &&
           vertical_bin == other.vertical_bin;
  }
};

struct CrossSectionKeyHash {
  std::size_t operator()(
      const CrossSectionKey& key
  ) const {
    std::size_t h =
        std::hash<int>{}(key.lateral_bin);
    h ^= std::hash<int>{}(key.vertical_bin) +
         0x9e3779b9U +
         (h << 6U) +
         (h >> 2U);
    return h;
  }
};

struct ContinuityCell {
  double min_forward_m{
      std::numeric_limits<double>::infinity()
  };
  double max_forward_m{
      -std::numeric_limits<double>::infinity()
  };
  std::unordered_set<int> forward_bins;
};

struct ContinuitySummary {
  std::size_t forward_bin_count{};
  double span_m{};
  bool compatible{false};
};

struct ScenePathProjection {
  double forward_m{};
  bool within_supported_path{false};
  double path_center_y_m{};
  double path_tor_z_m{};
  double lateral_from_path_m{};
  double height_from_tor_m{};
};

std::vector<ScenePathProjection> project_scene_to_path(
    const std::vector<VehicleEvidenceEndpoint>& endpoints,
    const LocalTrackHypothesis& track
) {
  std::vector<ScenePathProjection> out;
  out.reserve(endpoints.size());

  for (const auto& endpoint : endpoints) {
    const auto& point = endpoint.position;
    ScenePathProjection projection;
    projection.forward_m = point.x;
    projection.within_supported_path =
        point.x >= 0.0 &&
        point.x <= track.supported_lookahead_m;
    projection.path_center_y_m =
        track.lateral_a_inv_m * point.x * point.x +
        track.lateral_b * point.x +
        track.lateral_c_m;
    projection.path_tor_z_m =
        track.vertical_m * point.x +
        track.vertical_c_m;
    projection.lateral_from_path_m =
        point.y - projection.path_center_y_m;
    projection.height_from_tor_m =
        point.z - projection.path_tor_z_m;
    out.push_back(projection);
  }
  return out;
}

struct ContinuityVoxelKey {
  int forward_bin{};
  int lateral_bin{};
  int vertical_bin{};

  bool operator==(const ContinuityVoxelKey& other) const {
    return forward_bin == other.forward_bin &&
           lateral_bin == other.lateral_bin &&
           vertical_bin == other.vertical_bin;
  }
};

struct ContinuityVoxelKeyHash {
  std::size_t operator()(
      const ContinuityVoxelKey& key
  ) const {
    std::size_t h = std::hash<int>{}(key.forward_bin);
    h ^= std::hash<int>{}(key.lateral_bin) +
         0x9e3779b9U +
         (h << 6U) +
         (h >> 2U);
    h ^= std::hash<int>{}(key.vertical_bin) +
         0x9e3779b9U +
         (h << 6U) +
         (h >> 2U);
    return h;
  }
};

class ContinuityDisjointSet {
 public:
  explicit ContinuityDisjointSet(std::size_t n)
      : parent_(n), rank_(n, 0) {
    for (std::size_t i = 0; i < n; ++i) {
      parent_[i] = i;
    }
  }

  std::size_t find(std::size_t x) {
    if (parent_[x] != x) {
      parent_[x] = find(parent_[x]);
    }
    return parent_[x];
  }

  void unite(std::size_t a, std::size_t b) {
    a = find(a);
    b = find(b);
    if (a == b) {
      return;
    }
    if (rank_[a] < rank_[b]) {
      std::swap(a, b);
    }
    parent_[b] = a;
    if (rank_[a] == rank_[b]) {
      ++rank_[a];
    }
  }

 private:
  std::vector<std::size_t> parent_;
  std::vector<unsigned> rank_;
};

template <typename Eligible, typename SupportEligible>
void mark_continuity(
    std::vector<StructuralResidual>* residuals,
    const std::vector<ScenePathProjection>& scene,
    double lateral_bin_m,
    double vertical_bin_m,
    double forward_bin_m,
    int cross_section_neighbor_bins,
    std::size_t min_forward_bins,
    double min_span_m,
    Eligible eligible,
    SupportEligible support_eligible,
    bool uncertainty_shell
) {
  std::unordered_map<
      CrossSectionKey,
      ContinuityCell,
      CrossSectionKeyHash
  > cells;
  cells.reserve(residuals->size() / 4 + 1);

  auto key_for = [&](const auto& residual) {
    return CrossSectionKey{
        static_cast<int>(
            std::floor(
                residual.lateral_from_path_m /
                lateral_bin_m
            )
        ),
        static_cast<int>(
            std::floor(
                residual.height_from_tor_m /
                vertical_bin_m
            )
        ),
    };
  };

  // Build structural continuity from the complete current-scan scene, not
  // only from endpoints that already entered the possible vehicle sweep.
  // Otherwise a continuous wall/trackbed surface can look like a localized
  // obstacle exactly where a few boundary returns clip the sweep.
  for (const auto& support : scene) {
    if (!support_eligible(support)) {
      continue;
    }

    auto& cell = cells[key_for(support)];
    cell.min_forward_m =
        std::min(
            cell.min_forward_m,
            support.forward_m
        );
    cell.max_forward_m =
        std::max(
            cell.max_forward_m,
            support.forward_m
        );
    cell.forward_bins.insert(
        static_cast<int>(
            std::floor(
                support.forward_m /
                forward_bin_m
            )
        )
    );
  }

  std::unordered_map<
      CrossSectionKey,
      ContinuitySummary,
      CrossSectionKeyHash
  > summaries;
  summaries.reserve(cells.size());

  for (const auto& [key, _] : cells) {
    std::unordered_set<int> forward_bins;
    double min_forward =
        std::numeric_limits<double>::infinity();
    double max_forward =
        -std::numeric_limits<double>::infinity();

    for (int dy = -cross_section_neighbor_bins;
         dy <= cross_section_neighbor_bins;
         ++dy) {
      for (int dz = -cross_section_neighbor_bins;
           dz <= cross_section_neighbor_bins;
           ++dz) {
        const CrossSectionKey neighbor{
            key.lateral_bin + dy,
            key.vertical_bin + dz,
        };
        const auto it = cells.find(neighbor);
        if (it == cells.end()) {
          continue;
        }

        min_forward =
            std::min(
                min_forward,
                it->second.min_forward_m
            );
        max_forward =
            std::max(
                max_forward,
                it->second.max_forward_m
            );
        forward_bins.insert(
            it->second.forward_bins.begin(),
            it->second.forward_bins.end()
        );
      }
    }

    ContinuitySummary summary;
    summary.forward_bin_count =
        forward_bins.size();
    summary.span_m =
        std::isfinite(min_forward) &&
                std::isfinite(max_forward)
            ? max_forward - min_forward
            : 0.0;
    summary.compatible =
        summary.forward_bin_count >= min_forward_bins &&
        summary.span_m >= min_span_m;
    summaries.emplace(key, summary);
  }

  for (auto& residual : *residuals) {
    if (!eligible(residual)) {
      continue;
    }
    const auto it = summaries.find(key_for(residual));
    if (it == summaries.end()) {
      continue;
    }

    residual.longitudinal_support_forward_bins =
        std::max(
            residual.longitudinal_support_forward_bins,
            it->second.forward_bin_count
        );
    residual.longitudinal_support_span_m =
        std::max(
            residual.longitudinal_support_span_m,
            it->second.span_m
        );

    if (uncertainty_shell) {
      residual
          .uncertainty_shell_longitudinal_support_compatible =
          it->second.compatible;
    } else {
      residual.longitudinal_support_compatible =
          it->second.compatible;
    }
  }
}

template <typename Eligible, typename SupportEligible>
void mark_drifting_continuity(
    std::vector<StructuralResidual>* residuals,
    const std::vector<ScenePathProjection>& scene,
    const std::vector<SweepEvidenceSeed>& seeds,
    double lateral_bin_m,
    double vertical_bin_m,
    double forward_bin_m,
    int cross_section_neighbor_bins,
    std::size_t min_forward_bins,
    double min_span_m,
    Eligible eligible,
    SupportEligible support_eligible
) {
  struct Voxel {
    ContinuityVoxelKey key;
    double min_forward_m{
        std::numeric_limits<double>::infinity()
    };
    double max_forward_m{
        -std::numeric_limits<double>::infinity()
    };
  };

  auto key_for = [&](double forward_m,
                     const auto& support) {
    return ContinuityVoxelKey{
        static_cast<int>(
            std::floor(forward_m / forward_bin_m)
        ),
        static_cast<int>(
            std::floor(
                support.lateral_from_path_m /
                lateral_bin_m
            )
        ),
        static_cast<int>(
            std::floor(
                support.height_from_tor_m /
                vertical_bin_m
            )
        ),
    };
  };

  std::unordered_map<
      ContinuityVoxelKey,
      std::size_t,
      ContinuityVoxelKeyHash
  > voxel_index;
  std::vector<Voxel> voxels;
  voxel_index.reserve(scene.size() / 8 + 1);
  voxels.reserve(scene.size() / 8 + 1);

  for (const auto& support : scene) {
    if (!support_eligible(support)) {
      continue;
    }

    const auto key =
        key_for(support.forward_m, support);
    const auto [it, inserted] =
        voxel_index.emplace(key, voxels.size());
    if (inserted) {
      Voxel voxel;
      voxel.key = key;
      voxel.min_forward_m = support.forward_m;
      voxel.max_forward_m = support.forward_m;
      voxels.push_back(voxel);
    } else {
      auto& voxel = voxels[it->second];
      voxel.min_forward_m =
          std::min(
              voxel.min_forward_m,
              support.forward_m
          );
      voxel.max_forward_m =
          std::max(
              voxel.max_forward_m,
              support.forward_m
          );
    }
  }

  if (voxels.empty()) {
    return;
  }

  ContinuityDisjointSet dsu(voxels.size());
  for (std::size_t i = 0; i < voxels.size(); ++i) {
    const auto& key = voxels[i].key;
    for (int dy = -cross_section_neighbor_bins;
         dy <= cross_section_neighbor_bins;
         ++dy) {
      for (int dz = -cross_section_neighbor_bins;
           dz <= cross_section_neighbor_bins;
           ++dz) {
        const ContinuityVoxelKey next{
            key.forward_bin + 1,
            key.lateral_bin + dy,
            key.vertical_bin + dz,
        };
        const auto it = voxel_index.find(next);
        if (it != voxel_index.end()) {
          dsu.unite(i, it->second);
        }
      }
    }
  }

  struct Component {
    double min_forward_m{
        std::numeric_limits<double>::infinity()
    };
    double max_forward_m{
        -std::numeric_limits<double>::infinity()
    };
    std::unordered_set<int> forward_bins;
  };
  std::unordered_map<std::size_t, Component> components;
  components.reserve(voxels.size());

  for (std::size_t i = 0; i < voxels.size(); ++i) {
    auto& component = components[dsu.find(i)];
    component.min_forward_m =
        std::min(
            component.min_forward_m,
            voxels[i].min_forward_m
        );
    component.max_forward_m =
        std::max(
            component.max_forward_m,
            voxels[i].max_forward_m
        );
    component.forward_bins.insert(
        voxels[i].key.forward_bin
    );
  }

  for (std::size_t i = 0;
       i < residuals->size();
       ++i) {
    auto& residual = (*residuals)[i];
    if (!eligible(residual)) {
      continue;
    }

    const auto endpoint_index =
        seeds[i].endpoint_index;
    const auto key = key_for(
        scene[endpoint_index].forward_m,
        residual
    );
    const auto voxel_it = voxel_index.find(key);
    if (voxel_it == voxel_index.end()) {
      continue;
    }

    const auto component_it =
        components.find(
            dsu.find(voxel_it->second)
        );
    if (component_it == components.end()) {
      continue;
    }

    const auto& component = component_it->second;
    const std::size_t forward_bin_count =
        component.forward_bins.size();
    const double span_m =
        component.max_forward_m -
        component.min_forward_m;
    const bool compatible =
        forward_bin_count >= min_forward_bins &&
        span_m >= min_span_m;

    residual.longitudinal_support_forward_bins =
        std::max(
            residual.longitudinal_support_forward_bins,
            forward_bin_count
        );
    residual.longitudinal_support_span_m =
        std::max(
            residual.longitudinal_support_span_m,
            span_m
        );
    residual.longitudinal_support_compatible =
        residual.longitudinal_support_compatible ||
        compatible;
  }
}

}  // namespace

StructuralResidualEvaluator::StructuralResidualEvaluator(
    StructuralResidualConfig config
) : config_(std::move(config)) {
  if (!(config_.rail_lateral_tolerance_m > 0.0) ||
      !(config_.rail_vertical_tolerance_m > 0.0) ||
      !(config_.low_zone_max_height_m >
        config_.low_zone_min_height_m) ||
      !(config_.low_zone_half_width_m > 0.0) ||
      config_.rail_reference_max_extrapolation_m < 0.0 ||
      !(config_.continuity_zone_max_height_m >
        config_.continuity_zone_min_height_m) ||
      !(config_.continuity_zone_half_width_m > 0.0) ||
      !(config_.continuity_forward_bin_m > 0.0) ||
      !(config_.continuity_lateral_bin_m > 0.0) ||
      !(config_.continuity_vertical_bin_m > 0.0) ||
      config_.continuity_cross_section_neighbor_bins < 0 ||
      config_.continuity_min_forward_bins == 0 ||
      config_.continuity_min_span_m < 0.0 ||
      !(config_.side_continuity_max_abs_lateral_m >
        config_.side_continuity_min_abs_lateral_m) ||
      config_.side_continuity_min_abs_lateral_m < 0.0 ||
      !(config_.side_continuity_zone_max_height_m >
        config_.side_continuity_zone_min_height_m) ||
      !(config_.side_continuity_lateral_bin_m > 0.0) ||
      !(config_.side_continuity_vertical_bin_m > 0.0) ||
      config_.side_continuity_cross_section_neighbor_bins < 0 ||
      config_.side_continuity_min_forward_bins == 0 ||
      config_.side_continuity_min_span_m < 0.0 ||
      !(config_.shell_continuity_zone_max_height_m >
        config_.shell_continuity_zone_min_height_m) ||
      !(config_.shell_continuity_zone_half_width_m > 0.0) ||
      !(config_.shell_continuity_lateral_bin_m > 0.0) ||
      !(config_.shell_continuity_vertical_bin_m > 0.0) ||
      config_.shell_continuity_cross_section_neighbor_bins < 0 ||
      config_.shell_continuity_min_forward_bins == 0 ||
      config_.shell_continuity_min_span_m < 0.0) {
    throw std::invalid_argument(
        "invalid StructuralResidualConfig"
    );
  }
}

std::vector<StructuralResidual>
StructuralResidualEvaluator::evaluate(
    const std::vector<VehicleEvidenceEndpoint>& endpoints,
    const std::vector<SweepEvidenceSeed>& seeds,
    const RailSupportResult& rail_support,
    const LocalTrackHypothesis& track,
    const PossibleSweptVolume* nominal_sweep
) const {
  if (!track.valid) {
    throw std::invalid_argument(
        "structural residual requires a valid track hypothesis"
    );
  }

  const bool nominal_available =
      nominal_sweep != nullptr &&
      !nominal_sweep->branches.empty();

  const auto scene =
      project_scene_to_path(endpoints, track);

  std::vector<StructuralResidual> out;
  out.reserve(seeds.size());

  for (const auto& seed : seeds) {
    if (seed.endpoint_index >= endpoints.size()) {
      throw std::out_of_range(
          "sweep seed endpoint index out of range"
      );
    }

    const auto& point =
        endpoints[seed.endpoint_index].position;
    const auto& projected =
        scene[seed.endpoint_index];
    StructuralResidual residual;
    residual.endpoint_index = seed.endpoint_index;
    residual.branch_reference_available = true;
    residual.reference_branch_index = 0;
    residual.matching_branch_count =
        seed.matching_branch_indices.size();
    residual.within_supported_path =
        projected.within_supported_path;
    residual.path_center_y_m =
        projected.path_center_y_m;
    residual.path_tor_z_m =
        projected.path_tor_z_m;
    residual.lateral_from_path_m =
        projected.lateral_from_path_m;
    residual.height_from_tor_m =
        projected.height_from_tor_m;

    residual.nominal_sweep_available =
        nominal_available;
    if (nominal_available) {
      residual.inside_nominal_sweep =
          nominal_sweep->contains(point);
      residual.uncertainty_shell_only =
          !residual.inside_nominal_sweep;
    }

    const auto rail =
        interpolate_rail_reference(
            rail_support,
            point.x,
            config_.rail_reference_max_extrapolation_m
        );

    if (rail.has_value()) {
      residual.rail_reference_available = true;

      const double right_dy =
          point.y - rail->right_y;
      const double right_dz =
          point.z - rail->right_z;
      const double left_dy =
          point.y - rail->left_y;
      const double left_dz =
          point.z - rail->left_z;

      const double right_norm =
          std::hypot(
              right_dy /
                  config_.rail_lateral_tolerance_m,
              right_dz /
                  config_.rail_vertical_tolerance_m
          );
      const double left_norm =
          std::hypot(
              left_dy /
                  config_.rail_lateral_tolerance_m,
              left_dz /
                  config_.rail_vertical_tolerance_m
          );

      if (right_norm <= left_norm) {
        residual.nearest_rail =
            RailIdentity::Right;
        residual.rail_lateral_residual_m =
            right_dy;
        residual.rail_vertical_residual_m =
            right_dz;
        residual.rail_normalized_distance =
            right_norm;
      } else {
        residual.nearest_rail =
            RailIdentity::Left;
        residual.rail_lateral_residual_m =
            left_dy;
        residual.rail_vertical_residual_m =
            left_dz;
        residual.rail_normalized_distance =
            left_norm;
      }

      residual.rail_compatible =
          residual.rail_normalized_distance <= 1.0;
    } else {
      residual.rail_normalized_distance =
          std::numeric_limits<double>::infinity();
    }

    residual.low_track_zone =
        std::abs(residual.lateral_from_path_m) <=
            config_.low_zone_half_width_m &&
        residual.height_from_tor_m >=
            config_.low_zone_min_height_m &&
        residual.height_from_tor_m <=
            config_.low_zone_max_height_m;

    out.push_back(residual);
  }

  auto standard_eligible =
      [&](const auto& residual) {
        if (!residual.within_supported_path) {
          return false;
        }
        if (nominal_available &&
            residual.uncertainty_shell_only) {
          return false;
        }
        return
            std::abs(residual.lateral_from_path_m) <=
                config_.continuity_zone_half_width_m &&
            residual.height_from_tor_m >=
                config_.continuity_zone_min_height_m &&
            residual.height_from_tor_m <=
                config_.continuity_zone_max_height_m;
      };

  auto standard_support_eligible =
      [&](const auto& residual) {
        return
            residual.within_supported_path &&
            std::abs(residual.lateral_from_path_m) <=
                config_.continuity_zone_half_width_m &&
            residual.height_from_tor_m >=
                config_.continuity_zone_min_height_m &&
            residual.height_from_tor_m <=
                config_.continuity_zone_max_height_m;
      };

  mark_continuity(
      &out,
      scene,
      config_.continuity_lateral_bin_m,
      config_.continuity_vertical_bin_m,
      config_.continuity_forward_bin_m,
      config_.continuity_cross_section_neighbor_bins,
      config_.continuity_min_forward_bins,
      config_.continuity_min_span_m,
      standard_eligible,
      standard_support_eligible,
      false
  );

  auto side_eligible =
      [&](const StructuralResidual& residual) {
        if (!residual.within_supported_path) {
          return false;
        }
        if (nominal_available &&
            residual.uncertainty_shell_only) {
          return false;
        }

        const double lateral =
            std::abs(
                residual.lateral_from_path_m
            );
        return
            lateral >=
                config_
                    .side_continuity_min_abs_lateral_m &&
            lateral <=
                config_
                    .side_continuity_max_abs_lateral_m &&
            residual.height_from_tor_m >=
                config_
                    .side_continuity_zone_min_height_m &&
            residual.height_from_tor_m <=
                config_
                    .side_continuity_zone_max_height_m;
      };

  auto side_support_eligible =
      [&](const auto& residual) {
        if (!residual.within_supported_path) {
          return false;
        }
        const double lateral =
            std::abs(residual.lateral_from_path_m);
        return
            lateral >=
                config_.side_continuity_min_abs_lateral_m &&
            lateral <=
                config_.side_continuity_max_abs_lateral_m &&
            residual.height_from_tor_m >=
                config_.side_continuity_zone_min_height_m &&
            residual.height_from_tor_m <=
                config_.side_continuity_zone_max_height_m;
      };

  mark_continuity(
      &out,
      scene,
      config_.side_continuity_lateral_bin_m,
      config_.side_continuity_vertical_bin_m,
      config_.continuity_forward_bin_m,
      config_.side_continuity_cross_section_neighbor_bins,
      config_.side_continuity_min_forward_bins,
      config_.side_continuity_min_span_m,
      side_eligible,
      side_support_eligible,
      false
  );

  // Curved tunnel/service surfaces can drift slowly in both path-relative
  // lateral position and height. Follow only connected support between
  // consecutive forward bins, preserving the same spatial bin size,
  // neighborhood radius, minimum bin count and minimum span as the existing
  // side-continuity contract. Unlike a lateral-column fallback, this cannot
  // borrow support from an unrelated height slice.
  mark_drifting_continuity(
      &out,
      scene,
      seeds,
      config_.side_continuity_lateral_bin_m,
      config_.side_continuity_vertical_bin_m,
      config_.continuity_forward_bin_m,
      config_.side_continuity_cross_section_neighbor_bins,
      config_.side_continuity_min_forward_bins,
      config_.side_continuity_min_span_m,
      side_eligible,
      side_support_eligible
  );

  if (nominal_available) {
    auto shell_eligible =
        [&](const StructuralResidual& residual) {
          return
              residual.within_supported_path &&
              residual.uncertainty_shell_only &&
              std::abs(residual.lateral_from_path_m) <=
                  config_.shell_continuity_zone_half_width_m &&
              residual.height_from_tor_m >=
                  config_.shell_continuity_zone_min_height_m &&
              residual.height_from_tor_m <=
                  config_.shell_continuity_zone_max_height_m;
        };

    auto shell_support_eligible =
        [&](const auto& residual) {
          return
              residual.within_supported_path &&
              std::abs(residual.lateral_from_path_m) <=
                  config_.shell_continuity_zone_half_width_m &&
              residual.height_from_tor_m >=
                  config_.shell_continuity_zone_min_height_m &&
              residual.height_from_tor_m <=
                  config_.shell_continuity_zone_max_height_m;
        };

    mark_continuity(
        &out,
        scene,
        config_.shell_continuity_lateral_bin_m,
        config_.shell_continuity_vertical_bin_m,
        config_.continuity_forward_bin_m,
        config_.shell_continuity_cross_section_neighbor_bins,
        config_.shell_continuity_min_forward_bins,
        config_.shell_continuity_min_span_m,
        shell_eligible,
        shell_support_eligible,
        true
    );
  }

  return out;
}


std::vector<StructuralResidual>
StructuralResidualEvaluator::evaluate(
    const std::vector<VehicleEvidenceEndpoint>& endpoints,
    const std::vector<SweepEvidenceSeed>& seeds,
    const LocalTrackHypothesisSet& hypotheses,
    const PossibleSweptVolume* nominal_sweep
) const {
  if (seeds.empty()) {
    return {};
  }
  if (hypotheses.branches.empty()) {
    throw std::invalid_argument(
        "branch-aware residual requires track hypotheses"
    );
  }

  std::vector<std::vector<StructuralResidual>>
      candidates_by_seed(seeds.size());

  for (std::size_t branch_index = 0;
       branch_index < hypotheses.branches.size();
       ++branch_index) {
    const auto& branch =
        hypotheses.branches[branch_index];
    if (!branch.track.valid) {
      continue;
    }

    std::vector<SweepEvidenceSeed> branch_seeds;
    std::vector<std::size_t> source_seed_indices;
    branch_seeds.reserve(seeds.size());
    source_seed_indices.reserve(seeds.size());

    for (std::size_t seed_index = 0;
         seed_index < seeds.size();
         ++seed_index) {
      const auto& matching =
          seeds[seed_index].matching_branch_indices;
      if (std::find(
              matching.begin(),
              matching.end(),
              branch_index
          ) == matching.end()) {
        continue;
      }
      branch_seeds.push_back(seeds[seed_index]);
      source_seed_indices.push_back(seed_index);
    }

    if (branch_seeds.empty()) {
      continue;
    }

    PossibleSweptVolume nominal_branch;
    const PossibleSweptVolume* nominal_branch_ptr =
        nullptr;
    if (nominal_sweep != nullptr &&
        branch_index <
            nominal_sweep->branches.size()) {
      nominal_branch.branches.push_back(
          nominal_sweep->branches[branch_index]
      );
      nominal_branch_ptr = &nominal_branch;
    }

    auto branch_residuals =
        evaluate(
            endpoints,
            branch_seeds,
            branch.support,
            branch.track,
            nominal_branch_ptr
        );

    for (std::size_t i = 0;
         i < branch_residuals.size();
         ++i) {
      branch_residuals[i].branch_reference_available =
          true;
      branch_residuals[i].reference_branch_index =
          branch_index;
      branch_residuals[i].matching_branch_count =
          seeds[source_seed_indices[i]]
              .matching_branch_indices.size();
      candidates_by_seed[
          source_seed_indices[i]
      ].push_back(
          std::move(branch_residuals[i])
      );
    }
  }

  auto structural_rank =
      [](const StructuralResidual& residual) {
        int rank = 0;
        if (residual.within_supported_path) {
          rank += 8;
        }
        if (residual.rail_compatible) {
          rank += 4;
        }
        if (residual.longitudinal_support_compatible) {
          rank += 2;
        }
        if (residual
                .uncertainty_shell_longitudinal_support_compatible) {
          rank += 1;
        }
        return rank;
      };

  auto path_score =
      [&](const StructuralResidual& residual) {
        const double lateral_scale =
            std::max(
                0.10,
                config_.continuity_zone_half_width_m
            );
        const double vertical_scale =
            std::max(
                0.10,
                config_.continuity_zone_max_height_m -
                    config_.continuity_zone_min_height_m
            );
        return
            std::abs(residual.lateral_from_path_m) /
                lateral_scale +
            std::abs(residual.height_from_tor_m) /
                vertical_scale;
      };

  std::vector<StructuralResidual> out;
  out.reserve(seeds.size());

  for (std::size_t seed_index = 0;
       seed_index < seeds.size();
       ++seed_index) {
    auto& candidates =
        candidates_by_seed[seed_index];
    if (candidates.empty()) {
      StructuralResidual residual;
      residual.endpoint_index =
          seeds[seed_index].endpoint_index;
      out.push_back(std::move(residual));
      continue;
    }

    auto best = candidates.begin();
    for (auto it = candidates.begin() + 1;
         it != candidates.end();
         ++it) {
      const int best_rank =
          structural_rank(*best);
      const int candidate_rank =
          structural_rank(*it);

      bool choose = candidate_rank > best_rank;
      if (candidate_rank == best_rank) {
        if (it->rail_reference_available &&
            best->rail_reference_available &&
            std::isfinite(it->rail_normalized_distance) &&
            std::isfinite(best->rail_normalized_distance) &&
            it->rail_normalized_distance !=
                best->rail_normalized_distance) {
          choose =
              it->rail_normalized_distance <
              best->rail_normalized_distance;
        } else {
          choose =
              path_score(*it) <
              path_score(*best);
        }
      }

      if (choose) {
        best = it;
      }
    }

    out.push_back(std::move(*best));
  }

  return out;
}


}  // namespace metropilot
