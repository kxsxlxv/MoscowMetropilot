#include "metropilot/precision_gate.hpp"
#include <stdexcept>
#include <limits>

using namespace metropilot;
void expect(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
CurrentScanResult candidate(double x = 10.0, double y = 0.0) {
  CurrentScanResult result;
  result.status = DetectorStatus::Valid;
  CurrentScanCandidate c;
  c.endpoint_count = 24;
  c.non_structural_known_independent_firings = 24;
  c.non_structural_known_firing_timestamp_groups = 8;
  c.non_structural_nominal_sweep_endpoints = 24;
  c.non_structural_nominal_above_rail_spatial_cells = 8;
  c.centroid = {x, y, 0.7};
  result.candidates.push_back(c);
  CandidateAssessment a;
  a.disposition = CandidateDisposition::SupportedObstacle;
  result.classification.assessments.push_back(a);
  return result;
}
int main() {
  PrecisionGate gate;
  expect(!gate.update(candidate(), 1000000000), "first frame must not alarm");
  expect(!gate.update(candidate(9), 1100000000), "second frame must not alarm");
  expect(gate.update(candidate(8), 1200000000), "moving relative obstacle should confirm");
  CurrentScanResult empty;
  empty.status = DetectorStatus::Valid;
  expect(!gate.update(empty, 1300000000), "empty frame must immediately clear");
  expect(!gate.update(candidate(), 1400000000), "clear must reset streak");
  expect(!gate.update(candidate(), 1500000000), "reconfirmation needs three frames");
  expect(!gate.update(candidate(), 1500000000), "duplicate stamp must not confirm");
  expect(!gate.update(candidate(), 2500000000), "gap must not confirm");
  expect(!gate.update(candidate(), 2400000000), "backwards time must not confirm");
  auto invalid = candidate();
  invalid.status = DetectorStatus::Unknown;
  expect(!gate.update(invalid, 2500000000), "invalid geometry must clear");
  gate.reset();
  expect(!gate.update(candidate(10, -1), 1000000000), "left candidate starts streak");
  expect(!gate.update(candidate(10, 1), 1100000000), "unrelated right candidate starts new streak");
  expect(!gate.update(candidate(10, -1), 1200000000), "alternating candidates cannot confirm");
  auto weak = candidate();
  weak.candidates[0].non_structural_known_independent_firings = 2;
  for (int i = 0; i < 5; ++i)
    expect(!gate.update(weak, 2000000000LL + i * 100000000LL), "weak candidate must never confirm");
  auto shell = candidate();
  shell.candidates[0].non_structural_nominal_sweep_endpoints = 0;
  for (int i = 0; i < 5; ++i)
    expect(!gate.update(shell, 3000000000LL + i * 100000000LL), "shell only must never confirm");
  auto xyzi = candidate();
  auto below_rail = candidate();
  below_rail.candidates[0].non_structural_nominal_above_rail_spatial_cells = 0;
  for (int i = 0; i < 5; ++i)
    expect(!gate.update(below_rail, 4000000000LL + i * 100000000LL), "known firings below rail must not alarm");
  xyzi.classification.assessments[0].disposition = CandidateDisposition::TentativeObstacle;
  xyzi.candidates[0].non_structural_known_independent_firings = 0;
  xyzi.candidates[0].non_structural_unknown_identity_endpoints = 24;
  xyzi.candidates[0].non_structural_nominal_spatial_cells = 16;
  xyzi.candidates[0].non_structural_nominal_above_rail_spatial_cells = 8;
  gate.reset();
  expect(!gate.update(xyzi, 1000000000), "XYZI needs temporal support");
  expect(!gate.update(xyzi, 1100000000), "XYZI second frame");
  expect(gate.update(xyzi, 1200000000), "XYZI spatially distributed persistent object");
  xyzi.candidates[0].non_structural_nominal_above_rail_spatial_cells = 0;
  for (int i = 0; i < 5; ++i)
    expect(!gate.update(xyzi, 1500000000LL + i * 100000000LL), "below-rail XYZI evidence must not alarm");
  xyzi.candidates[0].non_structural_nominal_above_rail_spatial_cells = 8;
  xyzi.candidates[0].non_structural_nominal_spatial_cells = 1;
  for (int i = 0; i < 5; ++i)
    expect(!gate.update(xyzi, 2000000000LL + i * 100000000LL), "duplicate points do not confirm");
  bool rejected = false;
  try {
    PrecisionGateConfig config;
    config.max_source_gap_s = std::numeric_limits<double>::quiet_NaN();
    PrecisionGate bad(config);
  } catch (const std::invalid_argument&) { rejected = true; }
  expect(rejected, "nonfinite configuration must fail at startup");
}
