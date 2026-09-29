#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace metropilot {

enum class PointFieldType : std::uint8_t {
  Int8 = 1,
  UInt8 = 2,
  Int16 = 3,
  UInt16 = 4,
  Int32 = 5,
  UInt32 = 6,
  Float32 = 7,
  Float64 = 8,
};

struct PointField {
  std::string name;
  std::uint32_t offset{};
  PointFieldType datatype{};
  std::uint32_t count{1};
};

struct PointCloudBuffer {
  std::uint32_t height{1};
  std::uint32_t width{};
  std::vector<PointField> fields;
  bool is_bigendian{false};
  std::uint32_t point_step{};
  std::uint32_t row_step{};
  const std::uint8_t* data{nullptr};
  std::size_t data_size{};
  bool is_dense{false};
};

enum class CloudSchemaKind {
  XYZI,
  PandarEnriched,
};

struct PointSample {
  float x{};
  float y{};
  float z{};
  float intensity{};
  std::optional<std::uint16_t> ring;
  std::optional<double> timestamp_s;
  std::uint32_t row{};
  std::uint32_t column{};
  std::uint64_t slot_index{};
};

struct ReturnSample {
  float x{};
  float y{};
  float z{};
  float intensity{};
  std::uint64_t first_slot_index{};
  std::uint32_t raw_multiplicity{1};
};

struct FiringGroup {
  std::uint16_t ring{};
  double timestamp_s{};
  std::vector<ReturnSample> distinct_returns;
  std::uint32_t raw_valid_return_count{};
};

struct DecodeDiagnostics {
  CloudSchemaKind schema_kind{CloudSchemaKind::XYZI};
  std::uint64_t total_slots{};
  std::uint64_t finite_xyz_slots{};
  std::uint64_t origin_slots{};
  std::uint64_t nonfinite_slots{};
  std::uint64_t valid_points{};
  bool has_intensity{false};
  bool has_ring{false};
  bool has_timestamp{false};

  // Slot-level timing provenance is intentionally independent of XYZ
  // validity. Organizer clouds can preserve a finite firing/block timestamp
  // on (0,0,0) no-return slots. Those slots are not geometry evidence, but
  // their timestamps still delimit the published acquisition timeline.
  std::uint64_t raw_slots_with_finite_timestamp{};
  bool raw_slot_timestamp_complete{false};
  std::optional<double> min_raw_slot_timestamp_s;
  std::optional<double> max_raw_slot_timestamp_s;
  std::optional<double> raw_slot_timestamp_span_s;

  // Present only when both ring and timestamp are available. This is a count
  // of (packet timestamp, ring) identities, not raw PointCloud2 slots.
  std::optional<std::uint64_t> total_firing_groups;
  std::optional<std::uint64_t> firing_groups_with_valid_return;
  std::optional<std::uint64_t> groups_with_multiple_valid_returns;
  std::optional<std::uint64_t> groups_with_exact_duplicate_returns;
  std::optional<double> exact_duplicate_group_fraction;
};

struct DecodedCloud {
  DecodeDiagnostics diagnostics;
  std::vector<PointSample> valid_points;
  // Empty for schemas where physical firing identity cannot be established.
  std::vector<FiringGroup> firing_groups;
};

struct DecodeResult {
  bool ok{false};
  std::string error;
  DecodedCloud cloud;
};

struct PointCloudDecoderConfig {
  // Generic callers retain the complete valid-point list. Runtime paths that
  // consume enriched known firings through firing_groups may opt out of
  // retaining the same known returns a second time in valid_points.
  bool retain_known_enriched_valid_points{true};
};

class PointCloudDecoder {
 public:
  explicit PointCloudDecoder(
      PointCloudDecoderConfig config = {}
  ) : config_(config) {}

  DecodeResult decode(const PointCloudBuffer& input) const;

 private:
  PointCloudDecoderConfig config_;
};

const char* to_string(CloudSchemaKind kind);

}  // namespace metropilot
