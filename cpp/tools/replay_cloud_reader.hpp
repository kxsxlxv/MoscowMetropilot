#pragma once
#include "metropilot/pointcloud_decoder.hpp"
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>
namespace metropilot::replay {
struct ParsedPointCloud {
  std::int32_t header_sec{};
  std::uint32_t header_nsec{};
  std::string frame_id;
  std::uint32_t height{};
  std::uint32_t width{};
  std::vector<PointField> fields;
  bool is_bigendian{};
  std::uint32_t point_step{};
  std::uint32_t row_step{};
  // Non-owning view into the current sqlite3_column_blob payload. SQLite
  // keeps the pointer valid until the statement advances/resets/finalizes;
  // this replay finishes all frame processing before the next sqlite3_step.
  const std::uint8_t* data{};
  std::size_t data_size{};
  bool is_dense{};

  PointCloudBuffer view() const {
    PointCloudBuffer out;
    out.height = height;
    out.width = width;
    out.fields = fields;
    out.is_bigendian = is_bigendian;
    out.point_step = point_step;
    out.row_step = row_step;
    out.data = data;
    out.data_size = data_size;
    out.is_dense = is_dense;
    return out;
  }

  double source_time_s() const {
    return static_cast<double>(header_sec) +
           static_cast<double>(header_nsec) * 1e-9;
  }
};

struct ByteView {
  const std::uint8_t* data{};
  std::size_t size{};
};

class CdrReader {
 public:
  CdrReader(const void* data, std::size_t size)
      : data_(static_cast<const std::uint8_t*>(data)),
        size_(size) {
    if (size_ < 4 ||
        data_[0] != 0x00 ||
        data_[1] != 0x01 ||
        data_[2] != 0x00 ||
        data_[3] != 0x00) {
      throw std::runtime_error(
          "expected little-endian CDR1 encapsulation"
      );
    }
    pos_ = 4;
  }

  template <typename T>
  T scalar(std::size_t alignment) {
    align(alignment);
    require(sizeof(T));
    T value{};
    std::memcpy(&value, data_ + pos_, sizeof(T));
    pos_ += sizeof(T);
    return value;
  }

  std::string string() {
    const auto n = scalar<std::uint32_t>(4);
    if (n == 0) {
      return {};
    }
    require(n);
    if (data_[pos_ + n - 1] != 0) {
      throw std::runtime_error("invalid CDR string");
    }
    std::string out(
        reinterpret_cast<const char*>(data_ + pos_),
        n - 1
    );
    pos_ += n;
    return out;
  }

  ByteView bytes_view() {
    const auto n = scalar<std::uint32_t>(4);
    require(n);
    const ByteView out{
        data_ + pos_,
        static_cast<std::size_t>(n),
    };
    pos_ += n;
    return out;
  }

 private:
  void align(std::size_t alignment) {
    const std::size_t relative = pos_ - 4;
    const std::size_t padding =
        (alignment - relative % alignment) % alignment;
    require(padding);
    pos_ += padding;
  }

  void require(std::size_t n) const {
    if (pos_ + n > size_) {
      throw std::runtime_error("truncated CDR payload");
    }
  }

  const std::uint8_t* data_{};
  std::size_t size_{};
  std::size_t pos_{};
};

PointFieldType point_field_type(std::uint8_t datatype) {
  if (datatype < 1 || datatype > 8) {
    throw std::runtime_error("unsupported PointField datatype");
  }
  return static_cast<PointFieldType>(datatype);
}

ParsedPointCloud parse_pointcloud2(
    const void* data,
    std::size_t size
) {
  CdrReader cdr(data, size);
  ParsedPointCloud out;

  out.header_sec = cdr.scalar<std::int32_t>(4);
  out.header_nsec = cdr.scalar<std::uint32_t>(4);
  out.frame_id = cdr.string();
  out.height = cdr.scalar<std::uint32_t>(4);
  out.width = cdr.scalar<std::uint32_t>(4);

  const auto field_count =
      cdr.scalar<std::uint32_t>(4);
  out.fields.reserve(field_count);
  for (std::uint32_t i = 0; i < field_count; ++i) {
    PointField field;
    field.name = cdr.string();
    field.offset = cdr.scalar<std::uint32_t>(4);
    field.datatype =
        point_field_type(cdr.scalar<std::uint8_t>(1));
    field.count = cdr.scalar<std::uint32_t>(4);
    out.fields.push_back(std::move(field));
  }

  out.is_bigendian =
      cdr.scalar<std::uint8_t>(1) != 0;
  out.point_step = cdr.scalar<std::uint32_t>(4);
  out.row_step = cdr.scalar<std::uint32_t>(4);
  const auto payload = cdr.bytes_view();
  out.data = payload.data;
  out.data_size = payload.size;
  out.is_dense =
      cdr.scalar<std::uint8_t>(1) != 0;
  return out;
}


}
