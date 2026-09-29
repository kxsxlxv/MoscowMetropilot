#include "metropilot/vehicle_geometry_config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace metropilot {
namespace {

std::string trim(const std::string& value) {
  const auto begin = std::find_if_not(
      value.begin(),
      value.end(),
      [](unsigned char c) { return std::isspace(c); }
  );
  const auto end = std::find_if_not(
      value.rbegin(),
      value.rend(),
      [](unsigned char c) { return std::isspace(c); }
  ).base();
  if (begin >= end) {
    return {};
  }
  return std::string(begin, end);
}

std::string unquote(std::string value) {
  value = trim(value);
  if (value.size() >= 2 &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    return value.substr(1, value.size() - 2);
  }
  return value;
}

std::pair<std::string, std::string> split_key_value(
    const std::string& value
) {
  const auto colon = value.find(':');
  if (colon == std::string::npos) {
    return {trim(value), {}};
  }
  return {
      trim(value.substr(0, colon)),
      trim(value.substr(colon + 1)),
  };
}

double parse_double(
    const std::string& text,
    const std::string& key
) {
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::runtime_error(
        "invalid finite numeric value for " + key
    );
  }
  return value;
}

struct ComponentDraft {
  std::string id;
  std::string primitive;
  std::string anchor;

  std::optional<double> x_min_m;
  std::optional<double> x_max_m;
  std::optional<double> x_min_relative_anchor_m;
  std::optional<double> x_max_relative_anchor_m;
  std::optional<double> y_min_m;
  std::optional<double> y_max_m;
  std::optional<double> z_min_m;
  std::optional<double> z_max_m;
};

VehicleComponentBox finalize_component(
    const ComponentDraft& draft
) {
  if (draft.id.empty()) {
    throw std::runtime_error(
        "vehicle runtime component is missing id"
    );
  }
  if (draft.primitive != "box") {
    throw std::runtime_error(
        "unsupported vehicle runtime primitive for " +
        draft.id + ": " + draft.primitive
    );
  }
  if (!draft.y_min_m.has_value() ||
      !draft.y_max_m.has_value() ||
      !draft.z_min_m.has_value() ||
      !draft.z_max_m.has_value()) {
    throw std::runtime_error(
        "incomplete Y/Z extents for vehicle component " +
        draft.id
    );
  }

  ComponentAnchor anchor;
  std::optional<double> x_min;
  std::optional<double> x_max;
  if (draft.anchor == "body_chord") {
    anchor = ComponentAnchor::BodyChord;
    x_min = draft.x_min_m;
    x_max = draft.x_max_m;
  } else if (draft.anchor == "front_pivot") {
    anchor = ComponentAnchor::FrontPivot;
    x_min = draft.x_min_relative_anchor_m;
    x_max = draft.x_max_relative_anchor_m;
  } else if (draft.anchor == "rear_pivot") {
    anchor = ComponentAnchor::RearPivot;
    x_min = draft.x_min_relative_anchor_m;
    x_max = draft.x_max_relative_anchor_m;
  } else {
    throw std::runtime_error(
        "unsupported vehicle component anchor for " +
        draft.id + ": " + draft.anchor
    );
  }

  if (!x_min.has_value() || !x_max.has_value()) {
    throw std::runtime_error(
        "incomplete longitudinal extents for vehicle component " +
        draft.id
    );
  }

  if (!(*x_max > *x_min) ||
      !(*draft.y_max_m > *draft.y_min_m) ||
      !(*draft.z_max_m > *draft.z_min_m)) {
    throw std::runtime_error(
        "non-positive vehicle component extent for " +
        draft.id
    );
  }

  VehicleComponentBox out;
  out.name = draft.id;
  out.anchor = anchor;
  out.center_longitudinal_m =
      0.5 * (*x_min + *x_max);
  out.center_lateral_m =
      0.5 * (*draft.y_min_m + *draft.y_max_m);
  out.half_length_m =
      0.5 * (*x_max - *x_min);
  out.half_width_m =
      0.5 * (*draft.y_max_m - *draft.y_min_m);
  out.z_min_m = *draft.z_min_m;
  out.z_max_m = *draft.z_max_m;
  return out;
}

}  // namespace

VehicleSweepGeometry parse_vehicle_sweep_geometry_yaml(
    const std::string& yaml_text
) {
  std::optional<double> pivot_base_m;
  std::optional<double> body_length_m;
  std::optional<double> body_width_m;
  std::optional<std::size_t> declared_component_count;

  bool in_dimensions = false;
  bool in_runtime_lod = false;
  bool in_components = false;
  std::string dimension_name;

  std::vector<VehicleComponentBox> components;
  ComponentDraft current_component{};
  bool has_current_component = false;

  auto finish_component = [&]() {
    if (has_current_component) {
      components.push_back(
          finalize_component(current_component)
      );
      current_component = ComponentDraft{};
      has_current_component = false;
    }
  };

  std::istringstream stream(yaml_text);
  std::string raw_line;
  std::size_t line_number = 0;
  while (std::getline(stream, raw_line)) {
    ++line_number;

    const auto comment = raw_line.find('#');
    if (comment != std::string::npos) {
      raw_line = raw_line.substr(0, comment);
    }
    if (trim(raw_line).empty()) {
      continue;
    }

    const std::size_t indent =
        raw_line.find_first_not_of(' ');
    if (indent == std::string::npos) {
      continue;
    }
    if (raw_line.find('\t') != std::string::npos) {
      throw std::runtime_error(
          "tabs are not supported in vehicle geometry YAML"
      );
    }

    const std::string text = trim(raw_line);

    if (indent == 2) {
      finish_component();
      in_components = false;
      dimension_name.clear();
      in_dimensions = (text == "dimensions:");
      in_runtime_lod = (text == "runtime_lod:");
      continue;
    }

    if (in_dimensions) {
      if (indent == 4 && text.back() == ':') {
        dimension_name =
            trim(text.substr(0, text.size() - 1));
        continue;
      }
      if (indent == 6) {
        const auto [key, raw_value] =
            split_key_value(text);
        if (key == "value" && !raw_value.empty()) {
          const double value =
              parse_double(raw_value, dimension_name);
          if (dimension_name == "bogie_pivot_spacing_m") {
            pivot_base_m = value;
          } else if (
              dimension_name ==
              "body_length_without_couplers_m") {
            body_length_m = value;
          } else if (
              dimension_name == "overall_width_m") {
            body_width_m = value;
          }
        }
      }
      continue;
    }

    if (!in_runtime_lod) {
      continue;
    }

    if (indent == 4) {
      const auto [key, raw_value] =
          split_key_value(text);
      if (key == "component_count" &&
          !raw_value.empty()) {
        const double count =
            parse_double(raw_value, "component_count");
        if (count < 0.0 ||
            std::floor(count) != count) {
          throw std::runtime_error(
              "component_count must be a non-negative integer"
          );
        }
        declared_component_count =
            static_cast<std::size_t>(count);
      }
      if (text == "components:") {
        in_components = true;
      }
      continue;
    }

    if (!in_components) {
      continue;
    }

    if (indent == 6 && text.rfind("- ", 0) == 0) {
      finish_component();
      current_component = ComponentDraft{};
      has_current_component = true;
      const auto [key, raw_value] =
          split_key_value(text.substr(2));
      if (key != "id" || raw_value.empty()) {
        throw std::runtime_error(
            "runtime component list item must start with id"
        );
      }
      current_component.id = unquote(raw_value);
      continue;
    }

    if (indent != 8 || !has_current_component) {
      continue;
    }

    const auto [key, raw_value] =
        split_key_value(text);
    if (raw_value.empty()) {
      continue;
    }

    if (key == "primitive") {
      current_component.primitive =
          unquote(raw_value);
    } else if (key == "anchor") {
      current_component.anchor =
          unquote(raw_value);
    } else if (key == "x_min_m") {
      current_component.x_min_m =
          parse_double(raw_value, key);
    } else if (key == "x_max_m") {
      current_component.x_max_m =
          parse_double(raw_value, key);
    } else if (key == "x_min_relative_anchor_m") {
      current_component.x_min_relative_anchor_m =
          parse_double(raw_value, key);
    } else if (key == "x_max_relative_anchor_m") {
      current_component.x_max_relative_anchor_m =
          parse_double(raw_value, key);
    } else if (key == "y_min_m") {
      current_component.y_min_m =
          parse_double(raw_value, key);
    } else if (key == "y_max_m") {
      current_component.y_max_m =
          parse_double(raw_value, key);
    } else if (key == "z_min_m") {
      current_component.z_min_m =
          parse_double(raw_value, key);
    } else if (key == "z_max_m") {
      current_component.z_max_m =
          parse_double(raw_value, key);
    }
  }

  finish_component();

  if (!pivot_base_m.has_value() ||
      !body_length_m.has_value() ||
      !body_width_m.has_value()) {
    throw std::runtime_error(
        "vehicle geometry YAML is missing required dimensions"
    );
  }
  if (!(*pivot_base_m > 0.0) ||
      !(*body_length_m >= *pivot_base_m) ||
      !(*body_width_m > 0.0)) {
    throw std::runtime_error(
        "vehicle geometry YAML has invalid core dimensions"
    );
  }
  if (components.empty()) {
    throw std::runtime_error(
        "vehicle geometry YAML contains no runtime components"
    );
  }
  if (declared_component_count.has_value() &&
      *declared_component_count != components.size()) {
    throw std::runtime_error(
        "runtime component_count does not match parsed components"
    );
  }

  VehicleSweepGeometry out;
  out.pivot_base_m = *pivot_base_m;
  out.body_length_m = *body_length_m;
  out.body_width_m = *body_width_m;
  out.components = std::move(components);
  return out;
}

VehicleSweepGeometry load_vehicle_sweep_geometry_yaml(
    const std::string& path
) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error(
        "unable to open vehicle geometry config: " + path
    );
  }

  std::ostringstream buffer;
  buffer << input.rdbuf();
  if (!input.good() && !input.eof()) {
    throw std::runtime_error(
        "unable to read vehicle geometry config: " + path
    );
  }

  return parse_vehicle_sweep_geometry_yaml(
      buffer.str()
  );
}

}  // namespace metropilot
