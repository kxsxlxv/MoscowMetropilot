#include "metropilot/sensor_mount_config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

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

std::string unquote(std::string value) {
  value = trim(value);
  if (value.size() >= 2 &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    return value.substr(1, value.size() - 2);
  }
  return value;
}

bool is_null(const std::string& value) {
  const std::string normalized = unquote(value);
  return normalized == "null" ||
         normalized == "Null" ||
         normalized == "NULL" ||
         normalized == "~";
}

std::optional<double> parse_optional_double(
    const std::string& raw_value,
    const std::string& key
) {
  if (is_null(raw_value)) {
    return std::nullopt;
  }

  const std::string value = unquote(raw_value);
  std::size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::runtime_error(
        "invalid finite numeric value for " + key
    );
  }
  return parsed;
}

bool trusted_status(const std::string& raw_status) {
  const std::string status = unquote(raw_status);
  return
      status == "known" ||
      status == "calibrated" ||
      status == "measured";
}

bool assumed_status(const std::string& raw_status) {
  return unquote(raw_status) == "assumed";
}

bool parse_bool(
    const std::string& raw_value,
    const std::string& key
) {
  const std::string value = unquote(raw_value);
  if (value == "true" || value == "True" || value == "TRUE") {
    return true;
  }
  if (value == "false" || value == "False" || value == "FALSE") {
    return false;
  }
  throw std::runtime_error(
      "invalid boolean value for " + key
  );
}

void assign_mount_value(
    SensorMountCalibration* mount,
    const std::string& field,
    const std::optional<double>& value
) {
  if (field == "height_above_top_of_rail_m") {
    mount->height_above_tor_m = value;
  } else if (
      field == "lateral_offset_from_vehicle_center_m") {
    mount->lateral_offset_m = value;
  } else if (
      field == "longitudinal_offset_from_vehicle_reference_m") {
    mount->longitudinal_offset_m = value;
  } else if (field == "roll_deg") {
    mount->roll_deg = value;
  } else if (field == "pitch_deg") {
    mount->pitch_deg = value;
  } else if (field == "yaw_deg") {
    mount->yaw_deg = value;
  }
}

void assign_adapter_value(
    NativeFrameAdapterCalibration* adapter,
    const std::string& key,
    const std::optional<double>& value
) {
  if (key == "translation_x_m") {
    adapter->translation_x_m = value;
  } else if (key == "translation_y_m") {
    adapter->translation_y_m = value;
  } else if (key == "translation_z_m") {
    adapter->translation_z_m = value;
  } else if (key == "roll_deg") {
    adapter->roll_deg = value;
  } else if (key == "pitch_deg") {
    adapter->pitch_deg = value;
  } else if (key == "yaw_deg") {
    adapter->yaw_deg = value;
  }
}

}  // namespace

SensorCalibrationConfig parse_sensor_calibration_yaml(
    const std::string& yaml_text
) {
  SensorCalibrationConfig out;

  bool in_physical_mount = false;
  bool in_native_frames = false;
  bool in_runtime_adapter = false;
  bool in_runtime_policy = false;
  std::string mount_field;
  std::unordered_map<std::string, std::string>
      mount_field_status;
  std::optional<NativeFrameAdapterCalibration> current_adapter;
  std::string current_adapter_status;
  std::vector<std::string> runtime_adapter_statuses;

  auto finish_adapter = [&]() {
    if (!current_adapter.has_value()) {
      return;
    }
    if (current_adapter->frame_id.empty()) {
      throw std::runtime_error(
          "native_pointcloud_frames entry missing frame_id"
      );
    }
    out.configured_frame_ids.push_back(
        current_adapter->frame_id
    );
    out.runtime_adapters.push_back(
        *current_adapter
    );
    runtime_adapter_statuses.push_back(
        current_adapter_status
    );
    current_adapter.reset();
    current_adapter_status.clear();
  };

  std::istringstream stream(yaml_text);
  std::string raw_line;
  while (std::getline(stream, raw_line)) {
    const auto comment = raw_line.find('#');
    if (comment != std::string::npos) {
      raw_line = raw_line.substr(0, comment);
    }
    if (trim(raw_line).empty()) {
      continue;
    }
    if (raw_line.find('\t') != std::string::npos) {
      throw std::runtime_error(
          "tabs are not supported in sensor mount YAML"
      );
    }

    const std::size_t indent =
        raw_line.find_first_not_of(' ');
    if (indent == std::string::npos) {
      continue;
    }
    const std::string text = trim(raw_line);

    if (indent == 0) {
      if (text == "runtime_policy:") {
        finish_adapter();
      }
      in_physical_mount = false;
      in_native_frames =
          (text == "native_pointcloud_frames:");
      in_runtime_adapter = false;
      in_runtime_policy =
          (text == "runtime_policy:");
      mount_field.clear();
      continue;
    }

    if (in_runtime_policy && indent == 2) {
      const auto [key, raw_value] =
          split_key_value(text);
      if (raw_value.empty()) {
        continue;
      }

      // runtime_policy also contains descriptive enum/string fields such as
      // unknown_native_frame_status: UNKNOWN. Parse booleans only for the
      // explicitly modeled boolean policy switches; other policy metadata is
      // intentionally ignored by this narrow runtime loader.
      if (key == "trust_frame_id_as_extrinsic_calibration") {
        out.trust_frame_id_as_extrinsic_calibration =
            parse_bool(raw_value, key);
      } else if (key == "require_explicit_native_frame_adapter") {
        out.require_explicit_native_frame_adapter =
            parse_bool(raw_value, key);
      } else if (key == "require_complete_physical_sensor_mount") {
        out.require_complete_physical_sensor_mount =
            parse_bool(raw_value, key);
      } else if (
          key ==
          "require_both_transforms_before_physical_swept_volume_decision") {
        out.require_both_transforms_before_physical_swept_volume_decision =
            parse_bool(raw_value, key);
      } else if (
          key ==
          "permit_research_only_adapter_hypotheses_in_submission") {
        out.permit_research_only_adapter_hypotheses_in_submission =
            parse_bool(raw_value, key);
      } else if (
          key ==
          "permit_explicit_assumed_calibration_in_submission") {
        out.permit_explicit_assumed_calibration_in_submission =
            parse_bool(raw_value, key);
      }
      continue;
    }

    if (!in_native_frames &&
        indent == 2 &&
        text == "physical_mount:") {
      in_physical_mount = true;
      mount_field.clear();
      continue;
    }

    if (in_physical_mount) {
      if (indent == 4 && text.back() == ':') {
        mount_field =
            trim(text.substr(0, text.size() - 1));
        continue;
      }
      if (indent == 6 && !mount_field.empty()) {
        const auto [key, raw_value] =
            split_key_value(text);
        if (key == "value" && !raw_value.empty()) {
          assign_mount_value(
              &out.physical_mount,
              mount_field,
              parse_optional_double(
                  raw_value,
                  "physical_mount." + mount_field
              )
          );
        } else if (
            key == "status" &&
            !raw_value.empty()) {
          mount_field_status[mount_field] =
              unquote(raw_value);
        }
      }
      continue;
    }

    if (!in_native_frames) {
      continue;
    }

    if (indent == 2 && text.rfind("- ", 0) == 0) {
      finish_adapter();
      current_adapter.emplace();
      current_adapter_status.clear();
      in_runtime_adapter = false;
      const auto [key, raw_value] =
          split_key_value(text.substr(2));
      if (key != "frame_id" || raw_value.empty()) {
        throw std::runtime_error(
            "native frame list entry must start with frame_id"
        );
      }
      current_adapter->frame_id =
          unquote(raw_value);
      continue;
    }

    if (!current_adapter.has_value()) {
      continue;
    }

    if (indent == 4) {
      if (text == "runtime_adapter:") {
        in_runtime_adapter = true;
      } else if (text.back() == ':') {
        // Explicitly leave research_candidate and
        // development_fixture_assumption outside the runtime contract.
        in_runtime_adapter = false;
      }
      continue;
    }

    if (in_runtime_adapter && indent == 6) {
      const auto [key, raw_value] =
          split_key_value(text);
      if (key == "status" && !raw_value.empty()) {
        current_adapter_status =
            unquote(raw_value);
        continue;
      }
      const bool numeric_adapter_field =
          key == "translation_x_m" ||
          key == "translation_y_m" ||
          key == "translation_z_m" ||
          key == "roll_deg" ||
          key == "pitch_deg" ||
          key == "yaw_deg";
      if (!numeric_adapter_field ||
          raw_value.empty()) {
        continue;
      }
      assign_adapter_value(
          &*current_adapter,
          key,
          parse_optional_double(
              raw_value,
              "runtime_adapter." + key
          )
      );
    }
  }

  finish_adapter();

  if (out.configured_frame_ids.empty()) {
    throw std::runtime_error(
        "sensor mount YAML contains no native_pointcloud_frames"
    );
  }

  if (runtime_adapter_statuses.size() !=
      out.runtime_adapters.size()) {
    throw std::runtime_error(
        "internal native adapter status mismatch"
    );
  }
  for (std::size_t i = 0;
       i < out.runtime_adapters.size();
       ++i) {
    const bool trusted =
        trusted_status(runtime_adapter_statuses[i]);
    const bool assumed =
        assumed_status(runtime_adapter_statuses[i]);
    const bool accepted_assumed =
        assumed &&
        out.permit_explicit_assumed_calibration_in_submission;
    if (!trusted && !accepted_assumed) {
      auto& adapter = out.runtime_adapters[i];
      adapter.translation_x_m.reset();
      adapter.translation_y_m.reset();
      adapter.translation_z_m.reset();
      adapter.roll_deg.reset();
      adapter.pitch_deg.reset();
      adapter.yaw_deg.reset();
    } else if (accepted_assumed) {
      out.uses_assumed_calibration = true;
    }
  }

  auto sanitize_mount =
      [&](const std::string& config_name,
          std::optional<double>* value) {
        const auto it =
            mount_field_status.find(config_name);
        if (it == mount_field_status.end()) {
          value->reset();
          return;
        }
        const bool trusted =
            trusted_status(it->second);
        const bool assumed =
            assumed_status(it->second);
        const bool accepted_assumed =
            assumed &&
            out.permit_explicit_assumed_calibration_in_submission;
        if (!trusted && !accepted_assumed) {
          value->reset();
        } else if (accepted_assumed) {
          out.uses_assumed_calibration = true;
        }
      };

  sanitize_mount(
      "height_above_top_of_rail_m",
      &out.physical_mount.height_above_tor_m
  );
  sanitize_mount(
      "lateral_offset_from_vehicle_center_m",
      &out.physical_mount.lateral_offset_m
  );
  sanitize_mount(
      "longitudinal_offset_from_vehicle_reference_m",
      &out.physical_mount.longitudinal_offset_m
  );
  sanitize_mount(
      "roll_deg",
      &out.physical_mount.roll_deg
  );
  sanitize_mount(
      "pitch_deg",
      &out.physical_mount.pitch_deg
  );
  sanitize_mount(
      "yaw_deg",
      &out.physical_mount.yaw_deg
  );

  std::vector<std::string> sorted = out.configured_frame_ids;
  std::sort(sorted.begin(), sorted.end());
  if (std::adjacent_find(sorted.begin(), sorted.end()) !=
      sorted.end()) {
    throw std::runtime_error(
        "sensor mount YAML contains duplicate frame_id"
    );
  }

  return out;
}

SensorCalibrationConfig load_sensor_calibration_yaml(
    const std::string& path
) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error(
        "unable to open sensor calibration config: " + path
    );
  }

  std::ostringstream buffer;
  buffer << input.rdbuf();
  if (!input.good() && !input.eof()) {
    throw std::runtime_error(
        "unable to read sensor calibration config: " + path
    );
  }

  return parse_sensor_calibration_yaml(
      buffer.str()
  );
}

NativeFrameResolution resolve_configured_native_frame_to_vehicle(
    const std::string& frame_id,
    const SensorCalibrationConfig& config
) {
  // Current safety policy is deliberately stricter than any optional
  // convenience behavior: frame_id is never an extrinsic calibration.
  return resolve_native_frame_to_vehicle(
      frame_id,
      config.runtime_adapters,
      config.physical_mount
  );
}

}  // namespace metropilot
