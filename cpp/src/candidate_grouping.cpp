#include "metropilot/candidate_grouping.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace metropilot {
namespace {

constexpr double kPi =
    3.141592653589793238462643383279502884;

struct CellKey {
  int x{};
  int y{};
  int z{};

  bool operator==(const CellKey& other) const {
    return x == other.x &&
           y == other.y &&
           z == other.z;
  }
};

struct CellKeyHash {
  std::size_t operator()(const CellKey& key) const {
    std::size_t h =
        std::hash<int>{}(key.x);
    h ^= std::hash<int>{}(key.y) +
         0x9e3779b9U +
         (h << 6U) +
         (h >> 2U);
    h ^= std::hash<int>{}(key.z) +
         0x9e3779b9U +
         (h << 6U) +
         (h >> 2U);
    return h;
  }
};

class DisjointSet {
 public:
  explicit DisjointSet(std::size_t n)
      : parent_(n),
        rank_(n, 0) {
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

std::uint64_t double_bits(double value) {
  std::uint64_t bits{};
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(value));
  return bits;
}

double distance(
    const Vec3& a,
    const Vec3& b
) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool share_branch(
    const SweepEvidenceSeed& a,
    const SweepEvidenceSeed& b
) {
  for (const auto branch_a :
       a.matching_branch_indices) {
    if (std::find(
            b.matching_branch_indices.begin(),
            b.matching_branch_indices.end(),
            branch_a
        ) != b.matching_branch_indices.end()) {
      return true;
    }
  }
  return false;
}

std::vector<std::size_t> branch_union(
    const std::vector<std::size_t>& indices,
    const std::vector<SweepEvidenceSeed>& seeds
) {
  std::vector<std::size_t> out;
  for (const auto seed_index : indices) {
    for (const auto branch :
         seeds[seed_index].matching_branch_indices) {
      if (std::find(out.begin(), out.end(), branch) ==
          out.end()) {
        out.push_back(branch);
      }
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

}  // namespace

CandidateGrouper::CandidateGrouper(
    CandidateGroupingConfig config
) : config_(std::move(config)) {
  if (!(config_.base_radius_m >= 0.0) ||
      !(config_.angular_radius_deg >= 0.0) ||
      !(config_.min_neighbor_radius_m > 0.0) ||
      !(config_.max_neighbor_radius_m >=
        config_.min_neighbor_radius_m)) {
    throw std::invalid_argument(
        "invalid CandidateGroupingConfig"
    );
  }
}

std::vector<CurrentScanCandidate>
CandidateGrouper::group(
    const std::vector<VehicleEvidenceEndpoint>& endpoints,
    const std::vector<SweepEvidenceSeed>& seeds,
    const std::vector<StructuralResidual>& residuals
) const {
  if (residuals.size() != seeds.size()) {
    throw std::invalid_argument(
        "structural residual count must equal seed count"
    );
  }
  if (seeds.empty()) {
    return {};
  }

  for (std::size_t i = 0; i < seeds.size(); ++i) {
    if (seeds[i].endpoint_index >= endpoints.size()) {
      throw std::out_of_range(
          "candidate seed endpoint index out of range"
      );
    }
    if (residuals[i].endpoint_index !=
        seeds[i].endpoint_index) {
      throw std::invalid_argument(
          "residual/seed endpoint mismatch"
      );
    }
  }

  const double cell_size =
      config_.max_neighbor_radius_m;
  const double angular_rad =
      config_.angular_radius_deg *
      kPi / 180.0;

  auto radius_for = [&](std::size_t seed_index) {
    const auto endpoint_index =
        seeds[seed_index].endpoint_index;
    const double range =
        endpoints[endpoint_index].range_from_sensor_m;
    const double radius =
        config_.base_radius_m +
        range * std::tan(angular_rad);
    return std::clamp(
        radius,
        config_.min_neighbor_radius_m,
        config_.max_neighbor_radius_m
    );
  };

  auto cell_for = [&](const Vec3& p) {
    return CellKey{
        static_cast<int>(
            std::floor(p.x / cell_size)
        ),
        static_cast<int>(
            std::floor(p.y / cell_size)
        ),
        static_cast<int>(
            std::floor(p.z / cell_size)
        ),
    };
  };

  std::unordered_map<
      CellKey,
      std::vector<std::size_t>,
      CellKeyHash
  > cells;
  cells.reserve(seeds.size() * 2);

  DisjointSet dsu(seeds.size());

  for (std::size_t i = 0; i < seeds.size(); ++i) {
    const auto endpoint_index =
        seeds[i].endpoint_index;
    const auto& point =
        endpoints[endpoint_index].position;
    const auto cell = cell_for(point);

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const CellKey neighbor{
              cell.x + dx,
              cell.y + dy,
              cell.z + dz,
          };
          const auto it = cells.find(neighbor);
          if (it == cells.end()) {
            continue;
          }

          for (const auto j : it->second) {
            if (!share_branch(seeds[i], seeds[j])) {
              continue;
            }

            const bool structural_i =
                residuals[i].rail_compatible ||
                residuals[i].longitudinal_support_compatible ||
                residuals[i]
                    .uncertainty_shell_longitudinal_support_compatible;
            const bool structural_j =
                residuals[j].rail_compatible ||
                residuals[j].longitudinal_support_compatible ||
                residuals[j]
                    .uncertainty_shell_longitudinal_support_compatible;
            if (config_.prevent_soft_structural_to_structural_links &&
                structural_i &&
                structural_j) {
              continue;
            }

            const auto other_endpoint =
                seeds[j].endpoint_index;
            const double threshold =
                std::max(
                    radius_for(i),
                    radius_for(j)
                );
            if (distance(
                    point,
                    endpoints[other_endpoint].position
                ) <= threshold) {
              dsu.unite(i, j);
            }
          }
        }
      }
    }

    cells[cell].push_back(i);
  }

  std::unordered_map<
      std::size_t,
      std::vector<std::size_t>
  > groups;
  groups.reserve(seeds.size());
  for (std::size_t i = 0; i < seeds.size(); ++i) {
    groups[dsu.find(i)].push_back(i);
  }

  std::vector<CurrentScanCandidate> out;
  out.reserve(groups.size());

  for (auto& [_, group] : groups) {
    std::sort(group.begin(), group.end());

    CurrentScanCandidate candidate;
    candidate.seed_indices = group;
    candidate.matching_branch_indices =
        branch_union(group, seeds);

    candidate.endpoint_indices.reserve(
        group.size()
    );

    candidate.bbox.min = Vec3{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
    };
    candidate.bbox.max = Vec3{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
    };
    candidate.nearest_range_m =
        std::numeric_limits<double>::infinity();

    std::unordered_set<std::uint64_t> firing_ids;
    std::unordered_set<std::uint64_t> firing_timestamp_groups;
    std::unordered_set<std::uint64_t> non_structural_firing_ids;
    std::unordered_set<std::uint64_t>
        non_structural_firing_timestamp_groups;
    double range_sum = 0.0;
    Vec3 sum{};

    for (const auto seed_index : group) {
      const auto endpoint_index =
          seeds[seed_index].endpoint_index;
      const auto& endpoint =
          endpoints[endpoint_index];
      const auto& residual =
          residuals[seed_index];

      candidate.endpoint_indices.push_back(
          endpoint_index
      );
      ++candidate.endpoint_count;
      candidate.raw_return_multiplicity +=
          endpoint.raw_return_multiplicity;

      const bool soft_structural =
          residual.rail_compatible ||
          residual.longitudinal_support_compatible ||
          residual
              .uncertainty_shell_longitudinal_support_compatible;

      if (endpoint.firing_id.has_value()) {
        firing_ids.insert(*endpoint.firing_id);
        if (endpoint.point_timestamp_s.has_value() &&
            std::isfinite(*endpoint.point_timestamp_s)) {
          const auto timestamp_bits =
              double_bits(*endpoint.point_timestamp_s);
          firing_timestamp_groups.insert(timestamp_bits);
          if (!soft_structural) {
            non_structural_firing_timestamp_groups.insert(
                timestamp_bits
            );
          }
        }
        if (!soft_structural) {
          non_structural_firing_ids.insert(
              *endpoint.firing_id
          );
        }
      } else {
        ++candidate.unknown_identity_endpoints;
        if (!soft_structural) {
          ++candidate.non_structural_unknown_identity_endpoints;
        }
      }

      if (residual.rail_compatible) {
        ++candidate.rail_compatible_endpoints;
      }
      if (residual.longitudinal_support_compatible ||
          residual
              .uncertainty_shell_longitudinal_support_compatible) {
        ++candidate.longitudinal_support_compatible_endpoints;
      }
      if (soft_structural) {
        ++candidate.soft_structural_endpoints;
      }
      if (residual.low_track_zone) {
        ++candidate.low_track_zone_endpoints;
      }
      if (residual.nominal_sweep_available &&
          residual.inside_nominal_sweep) {
        ++candidate.nominal_sweep_endpoints;
        if (!soft_structural) {
          ++candidate.non_structural_nominal_sweep_endpoints;
        }
      }
      if (residual.nominal_sweep_available &&
          residual.uncertainty_shell_only) {
        ++candidate.uncertainty_shell_only_endpoints;
        if (!soft_structural) {
          ++candidate
                .non_structural_uncertainty_shell_only_endpoints;
        }
      }

      candidate.nearest_range_m =
          std::min(
              candidate.nearest_range_m,
              endpoint.range_from_sensor_m
          );
      range_sum += endpoint.range_from_sensor_m;

      sum.x += endpoint.position.x;
      sum.y += endpoint.position.y;
      sum.z += endpoint.position.z;

      candidate.bbox.min.x =
          std::min(
              candidate.bbox.min.x,
              endpoint.position.x
          );
      candidate.bbox.min.y =
          std::min(
              candidate.bbox.min.y,
              endpoint.position.y
          );
      candidate.bbox.min.z =
          std::min(
              candidate.bbox.min.z,
              endpoint.position.z
          );
      candidate.bbox.max.x =
          std::max(
              candidate.bbox.max.x,
              endpoint.position.x
          );
      candidate.bbox.max.y =
          std::max(
              candidate.bbox.max.y,
              endpoint.position.y
          );
      candidate.bbox.max.z =
          std::max(
              candidate.bbox.max.z,
              endpoint.position.z
          );
    }

    candidate.known_independent_firings =
        firing_ids.size();
    candidate.known_firing_timestamp_groups =
        firing_timestamp_groups.size();
    candidate.non_structural_known_independent_firings =
        non_structural_firing_ids.size();
    candidate.non_structural_known_firing_timestamp_groups =
        non_structural_firing_timestamp_groups.size();
    candidate.mean_range_m =
        range_sum /
        static_cast<double>(candidate.endpoint_count);
    candidate.centroid = Vec3{
        sum.x /
            static_cast<double>(candidate.endpoint_count),
        sum.y /
            static_cast<double>(candidate.endpoint_count),
        sum.z /
            static_cast<double>(candidate.endpoint_count),
    };

    candidate.longitudinal_span_m =
        candidate.bbox.max.x -
        candidate.bbox.min.x;
    candidate.lateral_span_m =
        candidate.bbox.max.y -
        candidate.bbox.min.y;
    candidate.vertical_span_m =
        candidate.bbox.max.z -
        candidate.bbox.min.z;
    candidate.rail_compatible_fraction =
        static_cast<double>(
            candidate.rail_compatible_endpoints
        ) /
        static_cast<double>(candidate.endpoint_count);
    candidate.longitudinal_support_compatible_fraction =
        static_cast<double>(
            candidate.longitudinal_support_compatible_endpoints
        ) /
        static_cast<double>(candidate.endpoint_count);
    candidate.soft_structural_fraction =
        static_cast<double>(
            candidate.soft_structural_endpoints
        ) /
        static_cast<double>(candidate.endpoint_count);
    candidate.low_track_zone_fraction =
        static_cast<double>(
            candidate.low_track_zone_endpoints
        ) /
        static_cast<double>(candidate.endpoint_count);

    out.push_back(std::move(candidate));
  }

  std::sort(
      out.begin(),
      out.end(),
      [](const CurrentScanCandidate& a,
         const CurrentScanCandidate& b) {
        if (a.nearest_range_m != b.nearest_range_m) {
          return a.nearest_range_m <
                 b.nearest_range_m;
        }
        return a.endpoint_count >
               b.endpoint_count;
      }
  );
  return out;
}

}  // namespace metropilot
