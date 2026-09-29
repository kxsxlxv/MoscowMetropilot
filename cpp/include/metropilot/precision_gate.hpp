#pragma once
#include "metropilot/current_scan_detector.hpp"
#include <cstdint>
#include <optional>
#include <vector>

namespace metropilot {
struct PrecisionGateConfig {
  bool allow_xyzi{true};
  std::size_t min_xyzi_spatial_cells{12};
  std::size_t min_xyzi_above_rail_cells{3};
  std::size_t min_known_above_rail_cells{3};
  std::size_t min_firings{12};
  std::size_t min_acquisition_groups{4};
  std::size_t min_nominal_endpoints{6};
  std::size_t confirmation_frames{3};
  double min_non_structural_fraction{0.5};
  double max_source_gap_s{0.3};
  double max_longitudinal_speed_mps{25.0};
  double match_longitudinal_slack_m{0.4};
  double match_lateral_m{0.35};
  double match_vertical_m{0.35};
};

// Decision persistence only: no point fusion, no ego-motion estimate. A current
// qualified candidate is always required. Clear/invalid/gap immediately reset.
class PrecisionGate {
 public:
  explicit PrecisionGate(PrecisionGateConfig config = {});
  bool update(const CurrentScanResult& result, std::int64_t source_time_ns);
  void reset();
  std::size_t qualified_count() const { return previous_.size(); }
 private:
  struct Track { Vec3 center; std::size_t streak; };
  PrecisionGateConfig config_;
  std::optional<std::int64_t> previous_time_;
  std::vector<Track> previous_;
};
}
