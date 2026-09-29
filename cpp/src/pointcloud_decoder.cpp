#include "metropilot/pointcloud_decoder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace metropilot {
namespace {

std::size_t field_type_size(PointFieldType t) {
  switch (t) {
    case PointFieldType::Int8:
    case PointFieldType::UInt8:
      return 1;
    case PointFieldType::Int16:
    case PointFieldType::UInt16:
      return 2;
    case PointFieldType::Int32:
    case PointFieldType::UInt32:
    case PointFieldType::Float32:
      return 4;
    case PointFieldType::Float64:
      return 8;
  }
  return 0;
}

const PointField* find_field(const PointCloudBuffer& input, const char* name) {
  const auto it = std::find_if(input.fields.begin(), input.fields.end(),
                               [name](const PointField& f) { return f.name == name; });
  return it == input.fields.end() ? nullptr : &*it;
}

bool host_is_little_endian() {
  const std::uint16_t one = 1;
  return *reinterpret_cast<const std::uint8_t*>(&one) == 1;
}

template <typename T>
T byte_swap(T value) {
  std::array<std::uint8_t, sizeof(T)> src{};
  std::array<std::uint8_t, sizeof(T)> dst{};
  std::memcpy(src.data(), &value, sizeof(T));
  std::reverse_copy(src.begin(), src.end(), dst.begin());
  std::memcpy(&value, dst.data(), sizeof(T));
  return value;
}

template <typename T>
T read_scalar(const std::uint8_t* p, bool data_big_endian) {
  T value{};
  std::memcpy(&value, p, sizeof(T));
  const bool host_big_endian = !host_is_little_endian();
  if (host_big_endian != data_big_endian) {
    value = byte_swap(value);
  }
  return value;
}

bool validate_field(const PointField* f, PointFieldType expected,
                    std::uint32_t point_step, std::string* error,
                    const char* required_name) {
  if (!f) {
    *error = std::string("missing required field: ") + required_name;
    return false;
  }
  if (f->datatype != expected || f->count != 1) {
    *error = std::string("unsupported field type/count for ") + required_name;
    return false;
  }
  const auto size = field_type_size(f->datatype);
  if (size == 0 || static_cast<std::uint64_t>(f->offset) + size > point_step) {
    *error = std::string("field exceeds point_step: ") + required_name;
    return false;
  }
  return true;
}

struct FiringKey {
  std::uint64_t timestamp_bits{};
  std::uint16_t ring{};

  bool operator==(const FiringKey& other) const {
    return timestamp_bits == other.timestamp_bits && ring == other.ring;
  }
};

struct FiringKeyHash {
  std::size_t operator()(const FiringKey& key) const {
    const auto a = std::hash<std::uint64_t>{}(key.timestamp_bits);
    const auto b = std::hash<std::uint16_t>{}(key.ring);
    return a ^ (b + 0x9e3779b97f4a7c15ULL + (a << 6U) + (a >> 2U));
  }
};

std::uint64_t bits_of_double(double v) {
  std::uint64_t bits{};
  static_assert(sizeof(bits) == sizeof(v));
  std::memcpy(&bits, &v, sizeof(v));
  return bits;
}

bool finite_timestamps_form_contiguous_runs(
    const PointCloudBuffer& input,
    const PointField& timestamp_field
) {
  std::optional<std::uint64_t> current;
  std::unordered_set<std::uint64_t> closed;
  closed.reserve(
      static_cast<std::size_t>(
          std::max<std::uint64_t>(
              16,
              static_cast<std::uint64_t>(input.width) / 128
          )
      )
  );

  for (std::uint32_t row = 0; row < input.height; ++row) {
    const auto* row_base =
        input.data +
        static_cast<std::size_t>(row) * input.row_step;
    for (std::uint32_t col = 0; col < input.width; ++col) {
      const auto* p =
          row_base +
          static_cast<std::size_t>(col) * input.point_step;
      const double timestamp =
          read_scalar<double>(
              p + timestamp_field.offset,
              input.is_bigendian
          );
      if (!std::isfinite(timestamp)) {
        continue;
      }

      const auto bits = bits_of_double(timestamp);
      if (!current.has_value()) {
        current = bits;
        continue;
      }
      if (bits == *current) {
        continue;
      }

      closed.insert(*current);
      if (closed.find(bits) != closed.end()) {
        return false;
      }
      current = bits;
    }
  }
  return true;
}

bool same_xyz(const ReturnSample& a, const PointSample& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

}  // namespace

const char* to_string(CloudSchemaKind kind) {
  switch (kind) {
    case CloudSchemaKind::XYZI:
      return "XYZI";
    case CloudSchemaKind::PandarEnriched:
      return "PandarEnriched";
  }
  return "Unknown";
}

DecodeResult PointCloudDecoder::decode(const PointCloudBuffer& input) const {
  DecodeResult out;

  if (input.height == 0 || input.width == 0) {
    out.error = "height and width must be non-zero";
    return out;
  }
  if (input.point_step == 0) {
    out.error = "point_step must be non-zero";
    return out;
  }
  const std::uint64_t minimum_row_bytes =
      static_cast<std::uint64_t>(input.width) * input.point_step;
  if (input.row_step < minimum_row_bytes) {
    out.error = "row_step is smaller than width * point_step";
    return out;
  }
  const std::uint64_t required_data_bytes =
      static_cast<std::uint64_t>(input.height) * input.row_step;
  if (required_data_bytes > input.data_size || input.data == nullptr) {
    out.error = "data buffer is smaller than height * row_step";
    return out;
  }

  const auto* fx = find_field(input, "x");
  const auto* fy = find_field(input, "y");
  const auto* fz = find_field(input, "z");
  const auto* fi = find_field(input, "intensity");
  const auto* fring = find_field(input, "ring");
  const auto* ftime = find_field(input, "timestamp");

  if (!validate_field(fx, PointFieldType::Float32, input.point_step, &out.error, "x") ||
      !validate_field(fy, PointFieldType::Float32, input.point_step, &out.error, "y") ||
      !validate_field(fz, PointFieldType::Float32, input.point_step, &out.error, "z")) {
    return out;
  }
  if (fi && !validate_field(fi, PointFieldType::Float32, input.point_step, &out.error,
                            "intensity")) {
    return out;
  }
  if (fring && !validate_field(fring, PointFieldType::UInt16, input.point_step, &out.error,
                               "ring")) {
    return out;
  }
  if (ftime && !validate_field(ftime, PointFieldType::Float64, input.point_step, &out.error,
                               "timestamp")) {
    return out;
  }

  const bool enriched = fring && ftime;
  out.cloud.diagnostics.schema_kind =
      enriched ? CloudSchemaKind::PandarEnriched : CloudSchemaKind::XYZI;
  out.cloud.diagnostics.has_intensity = fi != nullptr;
  out.cloud.diagnostics.has_ring = fring != nullptr;
  out.cloud.diagnostics.has_timestamp = ftime != nullptr;
  out.cloud.diagnostics.total_slots =
      static_cast<std::uint64_t>(input.height) * input.width;
  if (!enriched ||
      config_.retain_known_enriched_valid_points) {
    out.cloud.valid_points.reserve(
        static_cast<std::size_t>(
            out.cloud.diagnostics.total_slots / 2
        )
    );
  }

  const bool contiguous_firing_groups =
      enriched &&
      finite_timestamps_form_contiguous_runs(
          input,
          *ftime
      );

  std::unordered_set<FiringKey, FiringKeyHash> all_firing_keys;
  std::unordered_map<FiringKey, std::size_t, FiringKeyHash> valid_group_index;
  if (enriched && !contiguous_firing_groups) {
    all_firing_keys.reserve(
        static_cast<std::size_t>(
            out.cloud.diagnostics.total_slots / 2
        )
    );
    valid_group_index.reserve(
        static_cast<std::size_t>(
            out.cloud.diagnostics.total_slots / 2
        )
    );
  }

  constexpr std::size_t kFastRingUnseen =
      std::numeric_limits<std::size_t>::max();
  constexpr std::size_t kFastRingSeenNoValid =
      kFastRingUnseen - 1;
  std::vector<std::size_t> fast_ring_group_index;
  std::vector<std::uint16_t> fast_touched_rings;
  std::optional<std::uint64_t> fast_timestamp_bits;
  std::uint64_t fast_total_firing_groups = 0;
  if (contiguous_firing_groups) {
    fast_ring_group_index.assign(
        1U << 16U,
        kFastRingUnseen
    );
    fast_touched_rings.reserve(256);
  }

  std::uint64_t linear_slot = 0;
  for (std::uint32_t row = 0; row < input.height; ++row) {
    const auto* row_base = input.data + static_cast<std::size_t>(row) * input.row_step;
    for (std::uint32_t col = 0; col < input.width; ++col, ++linear_slot) {
      const auto* p = row_base + static_cast<std::size_t>(col) * input.point_step;
      const float x = read_scalar<float>(p + fx->offset, input.is_bigendian);
      const float y = read_scalar<float>(p + fy->offset, input.is_bigendian);
      const float z = read_scalar<float>(p + fz->offset, input.is_bigendian);

      std::optional<std::uint16_t> ring;
      std::optional<double> timestamp;
      std::optional<FiringKey> firing_key;
      if (fring != nullptr) {
        ring = read_scalar<std::uint16_t>(
            p + fring->offset,
            input.is_bigendian
        );
      }
      if (ftime != nullptr) {
        const double raw_timestamp =
            read_scalar<double>(
                p + ftime->offset,
                input.is_bigendian
            );
        if (std::isfinite(raw_timestamp)) {
          timestamp = raw_timestamp;
          ++out.cloud.diagnostics.raw_slots_with_finite_timestamp;
          if (!out.cloud.diagnostics
                   .min_raw_slot_timestamp_s.has_value()) {
            out.cloud.diagnostics.min_raw_slot_timestamp_s =
                raw_timestamp;
            out.cloud.diagnostics.max_raw_slot_timestamp_s =
                raw_timestamp;
          } else {
            out.cloud.diagnostics.min_raw_slot_timestamp_s =
                std::min(
                    *out.cloud.diagnostics
                         .min_raw_slot_timestamp_s,
                    raw_timestamp
                );
            out.cloud.diagnostics.max_raw_slot_timestamp_s =
                std::max(
                    *out.cloud.diagnostics
                         .max_raw_slot_timestamp_s,
                    raw_timestamp
                );
          }
        }
      }
      if (enriched &&
          ring.has_value() &&
          timestamp.has_value()) {
        firing_key =
            FiringKey{
                bits_of_double(*timestamp),
                *ring,
            };

        if (contiguous_firing_groups) {
          if (!fast_timestamp_bits.has_value() ||
              *fast_timestamp_bits !=
                  firing_key->timestamp_bits) {
            for (const auto touched :
                 fast_touched_rings) {
              fast_ring_group_index[touched] =
                  kFastRingUnseen;
            }
            fast_touched_rings.clear();
            fast_timestamp_bits =
                firing_key->timestamp_bits;
          }

          auto& state =
              fast_ring_group_index[*ring];
          if (state == kFastRingUnseen) {
            state = kFastRingSeenNoValid;
            fast_touched_rings.push_back(*ring);
            ++fast_total_firing_groups;
          }
        } else {
          all_firing_keys.insert(*firing_key);
        }
      }

      const bool finite = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
      if (!finite) {
        ++out.cloud.diagnostics.nonfinite_slots;
        continue;
      }
      ++out.cloud.diagnostics.finite_xyz_slots;
      if (x == 0.0F && y == 0.0F && z == 0.0F) {
        ++out.cloud.diagnostics.origin_slots;
        continue;
      }

      PointSample sample;
      sample.x = x;
      sample.y = y;
      sample.z = z;
      sample.intensity = fi ? read_scalar<float>(p + fi->offset, input.is_bigendian) : 0.0F;
      sample.ring = ring;
      sample.timestamp_s = timestamp;
      sample.row = row;
      sample.column = col;
      sample.slot_index = linear_slot;
      if (!enriched ||
          config_.retain_known_enriched_valid_points ||
          !firing_key.has_value()) {
        out.cloud.valid_points.push_back(sample);
      }
      ++out.cloud.diagnostics.valid_points;

      if (enriched && firing_key.has_value()) {
        std::size_t group_index = 0;
        if (contiguous_firing_groups) {
          auto& state =
              fast_ring_group_index[*ring];
          if (state == kFastRingSeenNoValid) {
            state = out.cloud.firing_groups.size();
            FiringGroup group;
            group.ring = *ring;
            group.timestamp_s = *timestamp;
            out.cloud.firing_groups.push_back(
                std::move(group)
            );
          }
          group_index = state;
        } else {
          auto [it, inserted] =
              valid_group_index.emplace(
                  *firing_key,
                  out.cloud.firing_groups.size()
              );
          if (inserted) {
            FiringGroup group;
            group.ring = *ring;
            group.timestamp_s = *timestamp;
            out.cloud.firing_groups.push_back(
                std::move(group)
            );
          }
          group_index = it->second;
        }

        auto& group =
            out.cloud.firing_groups[group_index];
        ++group.raw_valid_return_count;

        const auto duplicate = std::find_if(
            group.distinct_returns.begin(),
            group.distinct_returns.end(),
            [&sample](const ReturnSample& r) {
              return same_xyz(r, sample);
            }
        );
        if (duplicate !=
            group.distinct_returns.end()) {
          ++duplicate->raw_multiplicity;
        } else {
          group.distinct_returns.push_back(
              ReturnSample{
                  sample.x,
                  sample.y,
                  sample.z,
                  sample.intensity,
                  sample.slot_index,
                  1,
              }
          );
        }
      }
    }
  }

  if (ftime != nullptr) {
    out.cloud.diagnostics.raw_slot_timestamp_complete =
        out.cloud.diagnostics.total_slots > 0 &&
        out.cloud.diagnostics.raw_slots_with_finite_timestamp ==
            out.cloud.diagnostics.total_slots;
    if (out.cloud.diagnostics.min_raw_slot_timestamp_s.has_value() &&
        out.cloud.diagnostics.max_raw_slot_timestamp_s.has_value()) {
      out.cloud.diagnostics.raw_slot_timestamp_span_s =
          *out.cloud.diagnostics.max_raw_slot_timestamp_s -
          *out.cloud.diagnostics.min_raw_slot_timestamp_s;
    }
  }

  if (enriched) {
    std::uint64_t multi = 0;
    std::uint64_t duplicate_groups = 0;
    for (const auto& group : out.cloud.firing_groups) {
      if (group.raw_valid_return_count >= 2) {
        ++multi;
        if (group.distinct_returns.size() < group.raw_valid_return_count) {
          ++duplicate_groups;
        }
      }
    }
    out.cloud.diagnostics.total_firing_groups =
        contiguous_firing_groups
            ? fast_total_firing_groups
            : all_firing_keys.size();
    out.cloud.diagnostics.firing_groups_with_valid_return = out.cloud.firing_groups.size();
    out.cloud.diagnostics.groups_with_multiple_valid_returns = multi;
    out.cloud.diagnostics.groups_with_exact_duplicate_returns = duplicate_groups;
    out.cloud.diagnostics.exact_duplicate_group_fraction =
        multi == 0 ? 0.0 : static_cast<double>(duplicate_groups) / static_cast<double>(multi);
  }

  out.ok = true;
  return out;
}

}  // namespace metropilot
