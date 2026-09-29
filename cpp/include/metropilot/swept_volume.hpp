#pragma once

#include "metropilot/geometry.hpp"
#include "metropilot/sweep_geometry.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace metropilot {

enum class ComponentAnchor {
  BodyChord,
  FrontPivot,
  RearPivot,
};

struct VehicleComponentBox {
  std::string name;
  ComponentAnchor anchor{ComponentAnchor::BodyChord};

  // Offsets in the selected local track/body frame.
  double center_longitudinal_m{};
  double center_lateral_m{};

  double half_length_m{};
  double half_width_m{};

  // Vertical extents relative to the local TOR plane at the anchor.
  double z_min_m{};
  double z_max_m{};
};

struct VehicleSweepGeometry {
  // Distance between the two body pivots used for chord placement.
  double pivot_base_m{};
  // Used only for body-chord placement and analytic sanity checks.
  double body_length_m{};
  double body_width_m{};
  std::vector<VehicleComponentBox> components;
};

struct SweepMargins {
  double lateral_m{};
  double vertical_m{};
  double longitudinal_m{};
};

struct PlacedComponentBox {
  std::string name;
  std::size_t branch_index{};
  double front_chainage_m{};

  Vec3 center;
  Vec3 longitudinal_axis;
  Vec3 left_axis;
  Vec3 up_axis;

  double half_length_m{};
  double half_width_m{};
  double half_height_m{};

  // Exact world-frame AABB of this oriented component box. Built sweeps
  // populate it from OBB axis extents so membership can reject most boxes
  // before any local-frame dot products. Hand-assembled fixtures may leave
  // it unavailable and fall back to the legacy exact OBB test.
  bool aabb_available{false};
  Vec3 aabb_min;
  Vec3 aabb_max;

  // Kept for compatibility/diagnostics and manual test fixtures.
  double z_min_abs_m{};
  double z_max_abs_m{};
};

struct SweepBuildRequest {
  const PolylinePath* path{nullptr};
  std::size_t branch_index{};
  double front_start_chainage_m{};
  double front_end_chainage_m{};
  double sample_step_m{0.5};

  // Positive cant means the local left rail/left axis is higher than right.
  // The path itself supplies grade through its 3D centerline.
  double cant_rad{};
};

struct SweptVolume {
  std::vector<PlacedComponentBox> boxes;

  // Exact broad-phase bound of all oriented component boxes. Built sweeps
  // populate this from OBB axis extents; hand-assembled test fixtures may
  // leave it unavailable and automatically fall back to the legacy box scan.
  bool aabb_available{false};
  Vec3 aabb_min;
  Vec3 aabb_max;

  SweepMargins margins;
  double sampled_front_start_chainage_m{};
  double sampled_front_end_chainage_m{};
  double sample_step_m{};

  bool contains(const Vec3& point) const;
};

struct PossibleSweptVolume {
  std::vector<SweptVolume> branches;
  bool branch_ambiguous{false};

  bool contains(const Vec3& point) const;
};

SweptVolume build_swept_volume(
    const SweepBuildRequest& request,
    const VehicleSweepGeometry& geometry,
    const SweepMargins& margins
);

PossibleSweptVolume build_possible_swept_volume(
    const std::vector<SweepBuildRequest>& requests,
    const VehicleSweepGeometry& geometry,
    const SweepMargins& margins
);

}  // namespace metropilot
