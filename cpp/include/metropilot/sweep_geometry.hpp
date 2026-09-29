#pragma once

#include "metropilot/geometry.hpp"

#include <array>
#include <optional>
#include <vector>

namespace metropilot {

struct PathSample {
  double chainage_m{};
  Vec3 center{};
};

class PolylinePath {
 public:
  explicit PolylinePath(std::vector<PathSample> samples);

  double min_chainage_m() const;
  double max_chainage_m() const;
  Vec3 sample(double chainage_m) const;
  // Unit tangent in full 3D, including track grade.
  Vec3 tangent(double chainage_m) const;
  // Horizontal projection retained for plan-view analytics.
  Vec3 tangent_xy(double chainage_m) const;

 private:
  std::vector<PathSample> samples_;
};

struct ConstantRadiusBodyOffsets {
  double center_throw_m{};
  double end_throw_axis_m{};
  double end_throw_corner_m{};
};

ConstantRadiusBodyOffsets constant_radius_body_offsets(
    double radius_m,
    double pivot_base_m,
    double length_m,
    double half_width_m
);

std::optional<double> solve_rear_chainage_for_chord(
    const PolylinePath& path,
    double front_chainage_m,
    double pivot_base_m,
    double backward_scan_step_m = 0.25
);

struct PlanarBodyPlacement {
  double front_chainage_m{};
  double rear_chainage_m{};
  Vec3 front_pivot{};
  Vec3 rear_pivot{};
  Vec3 center{};
  // Full 3D chord frame. left_axis is the zero-cant lateral axis; up_axis is
  // normal to the chord/left plane. Cant is applied by the swept-volume layer.
  Vec3 longitudinal_axis{};
  Vec3 left_axis{};
  Vec3 up_axis{};
  std::array<Vec3, 4> footprint_corners{};
};

std::optional<PlanarBodyPlacement> place_planar_body(
    const PolylinePath& path,
    double front_chainage_m,
    double pivot_base_m,
    double length_m,
    double width_m
);

}  // namespace metropilot
