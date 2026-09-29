#include "replay_cloud_reader.hpp"
#include "metropilot/detector_pipeline.hpp"
#include "metropilot/precision_gate.hpp"
#include "metropilot/sensor_mount_config.hpp"
#include "metropilot/vehicle_geometry_config.hpp"
#include <sqlite3.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <memory>

using namespace metropilot;
std::string quoted(std::string s) {
  std::string out = "\"";
  for (char c : s) { if (c == '"') out += '"'; out += c; }
  return out + '"';
}
int main(int argc, char** argv) {
  try {
    if (argc != 5) throw std::runtime_error("usage: precision_replay DB_LIST OUTPUT_PREFIX SENSOR_YAML VEHICLE_YAML");
    const auto calibration = load_sensor_calibration_yaml(argv[3]);
    const auto vehicle = load_vehicle_sweep_geometry_yaml(argv[4]);
    PrecisionGate gate;
    std::ifstream list(argv[1]);
    if (!list) throw std::runtime_error("cannot read database list");
    std::ofstream frames(std::string(argv[2]) + "_frames.csv");
    std::ofstream candidates(std::string(argv[2]) + "_candidates.csv");
    if (!frames || !candidates) throw std::runtime_error("cannot write reports");
    frames << "frame,file,row_id,source_time_ns,flag,status,raw_supported,qualified,lookahead_m,processing_ms,error\n";
    candidates << "frame,candidate,disposition,x,y,z,dx,dy,dz,firings,groups,nominal,cells,above_rail_cells,structural_fraction\n";
    frames << std::setprecision(12); candidates << std::setprecision(12);
    std::string path, signature;
    std::size_t index = 0, alarms = 0, invalid = 0;
    while (std::getline(list, path)) {
      if (!path.empty() && path.back() == '\r') path.pop_back();
      if (path.empty()) continue;
      sqlite3* raw = nullptr;
      if (sqlite3_open_v2(path.c_str(), &raw, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
        const std::string error = raw ? sqlite3_errmsg(raw) : "open failed";
        if (raw) sqlite3_close(raw);
        throw std::runtime_error(error);
      }
      std::unique_ptr<sqlite3, decltype(&sqlite3_close)> db(raw, sqlite3_close);
      sqlite3_exec(raw, "PRAGMA mmap_size=2147418112", nullptr, nullptr, nullptr);
      sqlite3_stmt* stmt = nullptr;
      if (sqlite3_prepare_v2(raw,
          "SELECT m.id,length(m.data) FROM messages m JOIN topics t ON m.topic_id=t.id "
          "WHERE t.type='sensor_msgs/msg/PointCloud2' ORDER BY m.timestamp,m.id", -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error(sqlite3_errmsg(raw));
      std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)> query(stmt, sqlite3_finalize);
      sqlite3_stmt* payload_stmt = nullptr;
      if (sqlite3_prepare_v2(raw, "SELECT data FROM messages WHERE id=?", -1, &payload_stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error(sqlite3_errmsg(raw));
      std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)> payload(payload_stmt, sqlite3_finalize);
      int rc;
      while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        const auto begin = std::chrono::steady_clock::now();
        CurrentScanResult result;
        std::int64_t stamp = 0;
        bool flag = false;
        std::string error;
        try {
          if (sqlite3_column_type(stmt, 1) == SQLITE_NULL || sqlite3_column_int64(stmt, 1) > 64 * 1024 * 1024 + 65536)
            throw std::runtime_error("oversized or NULL serialized cloud");
          sqlite3_reset(payload_stmt);
          sqlite3_bind_int64(payload_stmt, 1, sqlite3_column_int64(stmt, 0));
          if (sqlite3_step(payload_stmt) != SQLITE_ROW) throw std::runtime_error(sqlite3_errmsg(raw));
          const auto cloud = replay::parse_pointcloud2(sqlite3_column_blob(payload_stmt, 0), sqlite3_column_bytes(payload_stmt, 0));
          stamp = static_cast<std::int64_t>(cloud.header_sec) * 1000000000LL + cloud.header_nsec;
          if (cloud.header_nsec >= 1000000000U || cloud.data_size > 64 * 1024 * 1024)
            throw std::runtime_error("invalid time or oversized cloud");
          std::ostringstream schema;
          schema << cloud.frame_id << '|' << cloud.height << '|' << cloud.point_step << '|' << cloud.is_bigendian;
          for (const auto& f : cloud.fields)
            schema << ';' << f.name << ':' << f.offset << ':' << static_cast<int>(f.datatype) << ':' << f.count;
          if (schema.str() != signature) { gate.reset(); signature = schema.str(); }
          const auto resolution = resolve_configured_native_frame_to_vehicle(cloud.frame_id, calibration);
          if (!resolution.native_to_vehicle_track) throw std::runtime_error("unconfigured frame: " + cloud.frame_id);
          result = detect_cloud(cloud.view(), *resolution.native_to_vehicle_track, vehicle);
          flag = gate.update(result, stamp);
        } catch (const std::exception& e) { error = e.what(); gate.reset(); }
        const double ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
        frames << index << ',' << quoted(std::filesystem::path(path).filename().string()) << ','
               << sqlite3_column_int64(stmt, 0) << ',' << stamp << ',' << flag << ','
               << static_cast<int>(result.status) << ',' << result.supported_obstacle_candidate_count << ','
               << gate.qualified_count() << ',' << result.supported_path_distance_m << ',' << ms << ',' << quoted(error) << '\n';
        for (const auto& a : result.classification.assessments) {
          const auto& c = result.candidates.at(a.candidate_index);
          if (a.disposition == CandidateDisposition::StructuralCompatible ||
              (c.non_structural_known_independent_firings < 12 && c.non_structural_nominal_spatial_cells < 12)) continue;
          candidates << index << ',' << a.candidate_index << ',' << static_cast<int>(a.disposition) << ','
              << c.centroid.x << ',' << c.centroid.y << ',' << c.centroid.z << ','
              << c.longitudinal_span_m << ',' << c.lateral_span_m << ',' << c.vertical_span_m << ','
              << c.non_structural_known_independent_firings << ',' << c.non_structural_known_firing_timestamp_groups << ','
              << c.non_structural_nominal_sweep_endpoints << ',' << c.non_structural_nominal_spatial_cells << ','
              << c.non_structural_nominal_above_rail_spatial_cells << ',' << c.soft_structural_fraction << '\n';
        }
        alarms += flag; invalid += result.status != DetectorStatus::Valid; ++index;
        if (index % 250 == 0) {
          frames.flush(); candidates.flush();
          std::cout << "frames=" << index << " alarms=" << alarms << " invalid=" << invalid << std::endl;
        }
      }
      if (rc != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(raw));
    }
    std::cout << "DONE frames=" << index << " alarms=" << alarms << " invalid=" << invalid << std::endl;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
