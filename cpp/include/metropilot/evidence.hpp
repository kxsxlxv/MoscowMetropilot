#pragma once

#include "metropilot/pointcloud_decoder.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace metropilot {

enum class FiringIdentityQuality {
  KnownPacketRing,
  Unknown,
};

struct EvidenceEndpoint {
  float x{};
  float y{};
  float z{};
  float intensity{};
  double range_m{};
  double azimuth_rad{};
  double elevation_rad{};

  FiringIdentityQuality firing_identity_quality{FiringIdentityQuality::Unknown};
  std::optional<std::uint64_t> firing_id;

  // Raw PointCloud2 point `timestamp` field when available and finite. This
  // is retained as scan-phase provenance only: the evidence layer does not
  // claim it is the exact laser firing time, nor assume a clock relationship
  // to PointCloud2.header.stamp.
  std::optional<double> point_timestamp_s;

  std::uint32_t raw_return_multiplicity{1};
  std::uint64_t first_slot_index{};
};

struct EvidenceDiagnostics {
  std::uint64_t endpoint_count{};
  std::uint64_t raw_valid_return_count{};
  std::optional<std::uint64_t> independent_firing_count;
  std::uint64_t endpoints_with_known_firing_identity{};
  std::uint64_t endpoints_with_known_point_timestamp{};
  bool point_timestamp_complete{false};
  std::optional<double> min_point_timestamp_s;
  std::optional<double> max_point_timestamp_s;
  std::optional<double> point_timestamp_span_s;

  // Raw PointCloud2 slot timing is acquisition provenance, not geometric
  // evidence. It includes timestamped no-return/origin slots and is kept
  // separate from endpoint timing above.
  std::uint64_t raw_slot_count{};
  std::uint64_t raw_slots_with_finite_timestamp{};
  bool raw_slot_timestamp_complete{false};
  std::optional<double> min_raw_slot_timestamp_s;
  std::optional<double> max_raw_slot_timestamp_s;
  std::optional<double> raw_slot_timestamp_span_s;

  std::uint64_t collapsed_exact_duplicate_returns{};
};

struct EvidenceFrame {
  std::vector<EvidenceEndpoint> endpoints;
  EvidenceDiagnostics diagnostics;
};

struct EvidenceBuilderConfig {
  // Direction angles are descriptive evidence fields. CurrentScanDetector A
  // consumes XYZ/range instead, so latency-sensitive callers may explicitly
  // skip the atan2 work while retaining all detector inputs unchanged.
  bool compute_direction_angles{true};
};

class EvidenceBuilder {
 public:
  explicit EvidenceBuilder(
      EvidenceBuilderConfig config = {}
  ) : config_(config) {}

  EvidenceFrame build(const DecodedCloud& cloud) const;

 private:
  EvidenceBuilderConfig config_;
};

}  // namespace metropilot
