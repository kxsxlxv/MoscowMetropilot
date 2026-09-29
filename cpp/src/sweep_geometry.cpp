#include "metropilot/sweep_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace metropilot {
namespace {

double distance_3d(const Vec3& a, const Vec3& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Vec3 cross(const Vec3& a, const Vec3& b) {
  return Vec3{
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

Vec3 add(const Vec3& a, const Vec3& b) {
  return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 scale(const Vec3& a, double k) {
  return Vec3{a.x * k, a.y * k, a.z * k};
}

}  // namespace

PolylinePath::PolylinePath(std::vector<PathSample> samples)
    : samples_(std::move(samples)) {
  if (samples_.size() < 2) {
    throw std::invalid_argument(
        "PolylinePath requires at least two samples"
    );
  }

  for (std::size_t i = 0; i < samples_.size(); ++i) {
    const auto& sample = samples_[i];
    if (!std::isfinite(sample.chainage_m) ||
        !std::isfinite(sample.center.x) ||
        !std::isfinite(sample.center.y) ||
        !std::isfinite(sample.center.z)) {
      throw std::invalid_argument(
          "PolylinePath samples must be finite"
      );
    }
    if (i > 0 &&
        !(samples_[i - 1].chainage_m < sample.chainage_m)) {
      throw std::invalid_argument(
          "PolylinePath chainage must be strictly increasing"
      );
    }
  }
}

double PolylinePath::min_chainage_m() const {
  return samples_.front().chainage_m;
}

double PolylinePath::max_chainage_m() const {
  return samples_.back().chainage_m;
}

Vec3 PolylinePath::sample(double chainage_m) const {
  if (chainage_m < min_chainage_m() ||
      chainage_m > max_chainage_m()) {
    throw std::out_of_range(
        "chainage outside PolylinePath"
    );
  }

  const auto upper = std::upper_bound(
      samples_.begin(),
      samples_.end(),
      chainage_m,
      [](double s, const PathSample& p) {
        return s < p.chainage_m;
      }
  );

  if (upper == samples_.begin()) {
    return upper->center;
  }
  if (upper == samples_.end()) {
    return samples_.back().center;
  }

  const auto& b = *upper;
  const auto& a = *(upper - 1);
  const double t =
      (chainage_m - a.chainage_m) /
      (b.chainage_m - a.chainage_m);

  return Vec3{
      a.center.x + t * (b.center.x - a.center.x),
      a.center.y + t * (b.center.y - a.center.y),
      a.center.z + t * (b.center.z - a.center.z),
  };
}

Vec3 PolylinePath::tangent(
    double chainage_m
) const {
  if (chainage_m < min_chainage_m() ||
      chainage_m > max_chainage_m()) {
    throw std::out_of_range(
        "chainage outside PolylinePath"
    );
  }

  const auto upper = std::upper_bound(
      samples_.begin(),
      samples_.end(),
      chainage_m,
      [](double s, const PathSample& p) {
        return s < p.chainage_m;
      }
  );

  const PathSample* a = nullptr;
  const PathSample* b = nullptr;

  if (upper == samples_.begin()) {
    a = &samples_[0];
    b = &samples_[1];
  } else if (upper == samples_.end()) {
    a = &samples_[samples_.size() - 2];
    b = &samples_.back();
  } else {
    a = &*(upper - 1);
    b = &*upper;
  }

  const double dx = b->center.x - a->center.x;
  const double dy = b->center.y - a->center.y;
  const double dz = b->center.z - a->center.z;
  const double norm =
      std::sqrt(dx * dx + dy * dy + dz * dz);
  if (!(norm > 0.0)) {
    throw std::runtime_error(
        "degenerate PolylinePath 3D segment"
    );
  }

  return Vec3{
      dx / norm,
      dy / norm,
      dz / norm,
  };
}

Vec3 PolylinePath::tangent_xy(
    double chainage_m
) const {
  if (chainage_m < min_chainage_m() ||
      chainage_m > max_chainage_m()) {
    throw std::out_of_range(
        "chainage outside PolylinePath"
    );
  }

  const auto upper = std::upper_bound(
      samples_.begin(),
      samples_.end(),
      chainage_m,
      [](double s, const PathSample& p) {
        return s < p.chainage_m;
      }
  );

  const PathSample* a = nullptr;
  const PathSample* b = nullptr;

  if (upper == samples_.begin()) {
    a = &samples_[0];
    b = &samples_[1];
  } else if (upper == samples_.end()) {
    a = &samples_[samples_.size() - 2];
    b = &samples_.back();
  } else {
    a = &*(upper - 1);
    b = &*upper;
  }

  const double dx = b->center.x - a->center.x;
  const double dy = b->center.y - a->center.y;
  const double norm = std::hypot(dx, dy);
  if (!(norm > 0.0)) {
    throw std::runtime_error(
        "degenerate PolylinePath XY segment"
    );
  }

  return Vec3{dx / norm, dy / norm, 0.0};
}

ConstantRadiusBodyOffsets constant_radius_body_offsets(
    double radius_m,
    double pivot_base_m,
    double length_m,
    double half_width_m
) {
  if (!(radius_m > 0.0) ||
      !(pivot_base_m > 0.0) ||
      !(length_m >= pivot_base_m) ||
      !(half_width_m >= 0.0) ||
      pivot_base_m >= 2.0 * radius_m) {
    throw std::invalid_argument(
        "invalid constant-radius body geometry"
    );
  }

  const double half_base = pivot_base_m / 2.0;
  const double rho = std::sqrt(
      radius_m * radius_m -
      half_base * half_base
  );
  const double half_length = length_m / 2.0;

  ConstantRadiusBodyOffsets out;
  out.center_throw_m = radius_m - rho;
  out.end_throw_axis_m =
      std::sqrt(
          rho * rho +
          half_length * half_length
      ) -
      radius_m;
  out.end_throw_corner_m =
      std::sqrt(
          (rho + half_width_m) *
              (rho + half_width_m) +
          half_length * half_length
      ) -
      radius_m -
      half_width_m;
  return out;
}

std::optional<double> solve_rear_chainage_for_chord(
    const PolylinePath& path,
    double front_chainage_m,
    double pivot_base_m,
    double backward_scan_step_m
) {
  if (!(pivot_base_m > 0.0) ||
      !(backward_scan_step_m > 0.0) ||
      front_chainage_m < path.min_chainage_m() ||
      front_chainage_m > path.max_chainage_m()) {
    return std::nullopt;
  }

  const Vec3 front = path.sample(front_chainage_m);
  double hi = front_chainage_m;
  double lo = hi;

  while (lo > path.min_chainage_m()) {
    const double candidate = std::max(
        path.min_chainage_m(),
        lo - backward_scan_step_m
    );
    const double distance =
        distance_3d(front, path.sample(candidate));

    if (distance >= pivot_base_m) {
      lo = candidate;
      break;
    }

    hi = candidate;
    lo = candidate;
  }

  const double lo_distance =
      distance_3d(front, path.sample(lo));
  if (lo_distance < pivot_base_m) {
    return std::nullopt;
  }
  if (std::abs(lo_distance - pivot_base_m) < 1e-12) {
    return lo;
  }

  // lo is at/above the requested chord distance; hi is below it.
  // This is the first backward crossing, avoiding older loop solutions.
  for (int i = 0; i < 64; ++i) {
    const double mid = 0.5 * (lo + hi);
    const double distance =
        distance_3d(front, path.sample(mid));

    if (distance >= pivot_base_m) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  return 0.5 * (lo + hi);
}

std::optional<PlanarBodyPlacement> place_planar_body(
    const PolylinePath& path,
    double front_chainage_m,
    double pivot_base_m,
    double length_m,
    double width_m
) {
  if (!(length_m >= pivot_base_m) ||
      !(width_m > 0.0)) {
    return std::nullopt;
  }

  const auto rear_chainage =
      solve_rear_chainage_for_chord(
          path,
          front_chainage_m,
          pivot_base_m
      );
  if (!rear_chainage.has_value()) {
    return std::nullopt;
  }

  const Vec3 front = path.sample(front_chainage_m);
  const Vec3 rear = path.sample(*rear_chainage);
  const double dx = front.x - rear.x;
  const double dy = front.y - rear.y;
  const double dz = front.z - rear.z;
  const double chord =
      std::sqrt(dx * dx + dy * dy + dz * dz);
  const double horizontal =
      std::hypot(dx, dy);

  if (!(chord > 0.0) || !(horizontal > 0.0)) {
    return std::nullopt;
  }

  const Vec3 longitudinal{
      dx / chord,
      dy / chord,
      dz / chord,
  };
  // Zero-cant left axis is horizontal and perpendicular to the plan-view
  // chord. Since longitudinal may include grade, cross(longitudinal,left)
  // yields the corresponding pitched up axis.
  const Vec3 left{
      -dy / horizontal,
      dx / horizontal,
      0.0,
  };
  const Vec3 up = cross(longitudinal, left);
  const Vec3 center{
      0.5 * (front.x + rear.x),
      0.5 * (front.y + rear.y),
      0.5 * (front.z + rear.z),
  };

  const Vec3 half_long =
      scale(longitudinal, length_m / 2.0);
  const Vec3 half_wide =
      scale(left, width_m / 2.0);

  PlanarBodyPlacement out;
  out.front_chainage_m = front_chainage_m;
  out.rear_chainage_m = *rear_chainage;
  out.front_pivot = front;
  out.rear_pivot = rear;
  out.center = center;
  out.longitudinal_axis = longitudinal;
  out.left_axis = left;
  out.up_axis = up;
  out.footprint_corners = {
      add(add(center, half_long), half_wide),
      add(
          add(center, half_long),
          scale(half_wide, -1.0)
      ),
      add(
          add(center, scale(half_long, -1.0)),
          half_wide
      ),
      add(
          add(center, scale(half_long, -1.0)),
          scale(half_wide, -1.0)
      ),
  };
  return out;
}

}  // namespace metropilot
