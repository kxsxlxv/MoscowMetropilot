#include "metropilot/evidence.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <utility>

namespace metropilot {
namespace {

EvidenceEndpoint make_endpoint(
    float x,
    float y,
    float z,
    float intensity,
    const std::optional<double>& point_timestamp_s,
    std::uint32_t multiplicity,
    std::uint64_t first_slot_index,
    bool compute_direction_angles
) {
  EvidenceEndpoint out;
  out.x = x;
  out.y = y;
  out.z = z;
  out.intensity = intensity;
  if (point_timestamp_s.has_value() &&
      std::isfinite(*point_timestamp_s)) {
    out.point_timestamp_s = point_timestamp_s;
  }
  out.raw_return_multiplicity = multiplicity;
  out.first_slot_index = first_slot_index;

  const double xd = static_cast<double>(x);
  const double yd = static_cast<double>(y);
  const double zd = static_cast<double>(z);
  const double xy = std::hypot(xd, yd);
  out.range_m = std::hypot(xy, zd);
  if (compute_direction_angles) {
    out.azimuth_rad = std::atan2(yd, xd);
    out.elevation_rad = std::atan2(zd, xy);
  }
  return out;
}

std::uint32_t float_bits(float value) {
  std::uint32_t bits{};
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(value));
  return bits;
}

std::uint64_t double_bits(double value) {
  std::uint64_t bits{};
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(value));
  return bits;
}

struct EndpointBits {
  std::uint32_t x{};
  std::uint32_t y{};
  std::uint32_t z{};
  bool point_timestamp_known{false};
  std::uint64_t point_timestamp_bits{};

  bool operator==(const EndpointBits& other) const {
    return
        x == other.x &&
        y == other.y &&
        z == other.z &&
        point_timestamp_known ==
            other.point_timestamp_known &&
        point_timestamp_bits ==
            other.point_timestamp_bits;
  }
};

struct EndpointBitsHash {
  std::size_t operator()(const EndpointBits& key) const {
    std::size_t h =
        std::hash<std::uint32_t>{}(key.x);
    const auto combine = [&h](std::size_t value) {
      h ^= value +
           0x9e3779b97f4a7c15ULL +
           (h << 6U) +
           (h >> 2U);
    };
    combine(std::hash<std::uint32_t>{}(key.y));
    combine(std::hash<std::uint32_t>{}(key.z));
    combine(std::hash<bool>{}(key.point_timestamp_known));
    combine(
        std::hash<std::uint64_t>{}(
            key.point_timestamp_bits
        )
    );
    return h;
  }
};

}  // namespace

EvidenceFrame EvidenceBuilder::build(
    const DecodedCloud& cloud
) const {
  EvidenceFrame out;
  out.diagnostics.raw_valid_return_count =
      cloud.diagnostics.valid_points;
  out.diagnostics.raw_slot_count =
      cloud.diagnostics.total_slots;
  out.diagnostics.raw_slots_with_finite_timestamp =
      cloud.diagnostics.raw_slots_with_finite_timestamp;
  out.diagnostics.raw_slot_timestamp_complete =
      cloud.diagnostics.raw_slot_timestamp_complete;
  out.diagnostics.min_raw_slot_timestamp_s =
      cloud.diagnostics.min_raw_slot_timestamp_s;
  out.diagnostics.max_raw_slot_timestamp_s =
      cloud.diagnostics.max_raw_slot_timestamp_s;
  out.diagnostics.raw_slot_timestamp_span_s =
      cloud.diagnostics.raw_slot_timestamp_span_s;

  std::unordered_map<
      EndpointBits,
      std::size_t,
      EndpointBitsHash
  > unknown_index;

  const auto append_unknown_point =
      [&](const PointSample& point) {
        const bool time_known =
            point.timestamp_s.has_value() &&
            std::isfinite(*point.timestamp_s);
        const EndpointBits key{
            float_bits(point.x),
            float_bits(point.y),
            float_bits(point.z),
            time_known,
            time_known
                ? double_bits(*point.timestamp_s)
                : 0U,
        };

        const auto it = unknown_index.find(key);
        if (it == unknown_index.end()) {
          auto endpoint = make_endpoint(
              point.x,
              point.y,
              point.z,
              point.intensity,
              point.timestamp_s,
              1,
              point.slot_index,
              config_.compute_direction_angles
          );
          endpoint.firing_identity_quality =
              FiringIdentityQuality::Unknown;
          unknown_index.emplace(
              key,
              out.endpoints.size()
          );
          out.endpoints.push_back(std::move(endpoint));
        } else {
          auto& endpoint =
              out.endpoints[it->second];
          ++endpoint.raw_return_multiplicity;
          ++out.diagnostics
                .collapsed_exact_duplicate_returns;
        }
      };

  if (!cloud.firing_groups.empty()) {
    out.diagnostics.independent_firing_count =
        cloud.firing_groups.size();

    std::size_t known_endpoint_count = 0;
    std::uint64_t known_raw_return_count = 0;
    for (const auto& group : cloud.firing_groups) {
      known_endpoint_count +=
          group.distinct_returns.size();
      known_raw_return_count +=
          group.raw_valid_return_count;
    }
    std::size_t unknown_raw_upper_bound = 0;
    if (cloud.diagnostics.valid_points >
        known_raw_return_count) {
      const auto unknown_raw =
          cloud.diagnostics.valid_points -
          known_raw_return_count;
      unknown_raw_upper_bound =
          static_cast<std::size_t>(
              std::min<std::uint64_t>(
                  unknown_raw,
                  static_cast<std::uint64_t>(
                      std::numeric_limits<std::size_t>::max()
                  )
              )
          );
    }
    out.endpoints.reserve(
        known_endpoint_count +
        unknown_raw_upper_bound
    );
    if (unknown_raw_upper_bound > 0) {
      unknown_index.reserve(
          unknown_raw_upper_bound
      );
    }

    std::uint64_t firing_id = 0;
    for (const auto& group : cloud.firing_groups) {
      for (const auto& ret : group.distinct_returns) {
        auto endpoint = make_endpoint(
            ret.x,
            ret.y,
            ret.z,
            ret.intensity,
            group.timestamp_s,
            ret.raw_multiplicity,
            ret.first_slot_index,
            config_.compute_direction_angles
        );
        endpoint.firing_identity_quality =
            FiringIdentityQuality::KnownPacketRing;
        endpoint.firing_id = firing_id;
        out.diagnostics.collapsed_exact_duplicate_returns +=
            static_cast<std::uint64_t>(
                ret.raw_multiplicity - 1
            );
        ++out.diagnostics
              .endpoints_with_known_firing_identity;
        out.endpoints.push_back(std::move(endpoint));
      }
      ++firing_id;
    }

    // Mixed enriched clouds may contain valid XYZ returns whose timestamp is
    // non-finite. They cannot be assigned a physical firing identity, but they
    // must not disappear merely because other points in the same cloud were
    // groupable.
    for (const auto& point : cloud.valid_points) {
      if (point.ring.has_value() &&
          point.timestamp_s.has_value()) {
        continue;
      }
      append_unknown_point(point);
    }
  } else {
    unknown_index.reserve(cloud.valid_points.size());
    // Without a complete (timestamp, ring) identity, exact duplicates cannot
    // be treated as independent evidence. Preserve a finite timestamp in the
    // duplicate key when one exists so identical XYZ returns acquired at
    // different times are not collapsed together.
    out.endpoints.reserve(cloud.valid_points.size());
    for (const auto& point : cloud.valid_points) {
      append_unknown_point(point);
    }
  }

  out.diagnostics.endpoint_count =
      out.endpoints.size();

  for (const auto& endpoint : out.endpoints) {
    if (!endpoint.point_timestamp_s.has_value() ||
        !std::isfinite(*endpoint.point_timestamp_s)) {
      continue;
    }

    ++out.diagnostics
          .endpoints_with_known_point_timestamp;
    if (!out.diagnostics
             .min_point_timestamp_s.has_value()) {
      out.diagnostics.min_point_timestamp_s =
          *endpoint.point_timestamp_s;
      out.diagnostics.max_point_timestamp_s =
          *endpoint.point_timestamp_s;
    } else {
      out.diagnostics.min_point_timestamp_s =
          std::min(
              *out.diagnostics.min_point_timestamp_s,
              *endpoint.point_timestamp_s
          );
      out.diagnostics.max_point_timestamp_s =
          std::max(
              *out.diagnostics.max_point_timestamp_s,
              *endpoint.point_timestamp_s
          );
    }
  }

  out.diagnostics.point_timestamp_complete =
      out.diagnostics.endpoint_count > 0 &&
      out.diagnostics
              .endpoints_with_known_point_timestamp ==
          out.diagnostics.endpoint_count;
  if (out.diagnostics.min_point_timestamp_s.has_value() &&
      out.diagnostics.max_point_timestamp_s.has_value()) {
    out.diagnostics.point_timestamp_span_s =
        *out.diagnostics.max_point_timestamp_s -
        *out.diagnostics.min_point_timestamp_s;
  }

  return out;
}

}  // namespace metropilot
