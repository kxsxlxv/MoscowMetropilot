#include "metropilot/swept_volume.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace metropilot {
namespace {

Vec3 add(const Vec3& a, const Vec3& b) {
  return Vec3{
      a.x + b.x,
      a.y + b.y,
      a.z + b.z,
  };
}

Vec3 scale(const Vec3& v, double k) {
  return Vec3{
      v.x * k,
      v.y * k,
      v.z * k,
  };
}

double dot_3d(const Vec3& a, const Vec3& b) {
  return
      a.x * b.x +
      a.y * b.y +
      a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
  return Vec3{
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

double norm_3d(const Vec3& v) {
  return std::sqrt(dot_3d(v, v));
}

Vec3 normalized(const Vec3& v) {
  const double norm = norm_3d(v);
  if (!(norm > 0.0)) {
    throw std::runtime_error(
        "degenerate sweep anchor axis"
    );
  }
  return scale(v, 1.0 / norm);
}

struct AnchorFrame {
  Vec3 origin;
  Vec3 longitudinal;
  Vec3 left;
  Vec3 up;
};

AnchorFrame apply_cant(
    const Vec3& origin,
    const Vec3& longitudinal,
    const Vec3& zero_cant_left,
    double cant_rad
) {
  const Vec3 forward = normalized(longitudinal);
  const Vec3 base_left = normalized(zero_cant_left);
  const Vec3 base_up =
      normalized(cross(forward, base_left));

  const double c = std::cos(cant_rad);
  const double s = std::sin(cant_rad);
  const Vec3 left =
      normalized(
          add(
              scale(base_left, c),
              scale(base_up, s)
          )
      );
  const Vec3 up =
      normalized(cross(forward, left));

  return AnchorFrame{
      origin,
      forward,
      left,
      up,
  };
}

AnchorFrame body_anchor(
    const PlanarBodyPlacement& placement,
    double cant_rad
) {
  return apply_cant(
      placement.center,
      placement.longitudinal_axis,
      placement.left_axis,
      cant_rad
  );
}

AnchorFrame pivot_anchor(
    const PolylinePath& path,
    double chainage_m,
    const Vec3& origin,
    double cant_rad
) {
  const Vec3 tangent = path.tangent(chainage_m);
  const double horizontal =
      std::hypot(tangent.x, tangent.y);
  if (!(horizontal > 0.0)) {
    throw std::runtime_error(
        "sweep path tangent has no horizontal component"
    );
  }

  const Vec3 zero_cant_left{
      -tangent.y / horizontal,
      tangent.x / horizontal,
      0.0,
  };
  return apply_cant(
      origin,
      tangent,
      zero_cant_left,
      cant_rad
  );
}

AnchorFrame select_anchor(
    const VehicleComponentBox& component,
    const PolylinePath& path,
    const PlanarBodyPlacement& placement,
    double cant_rad
) {
  switch (component.anchor) {
    case ComponentAnchor::BodyChord:
      return body_anchor(
          placement,
          cant_rad
      );
    case ComponentAnchor::FrontPivot:
      return pivot_anchor(
          path,
          placement.front_chainage_m,
          placement.front_pivot,
          cant_rad
      );
    case ComponentAnchor::RearPivot:
      return pivot_anchor(
          path,
          placement.rear_chainage_m,
          placement.rear_pivot,
          cant_rad
      );
  }
  throw std::runtime_error("unknown component anchor");
}

Vec3 box_aabb_radius(
    const PlacedComponentBox& box
) {
  return Vec3{
      std::abs(box.longitudinal_axis.x) *
              box.half_length_m +
          std::abs(box.left_axis.x) *
              box.half_width_m +
          std::abs(box.up_axis.x) *
              box.half_height_m,
      std::abs(box.longitudinal_axis.y) *
              box.half_length_m +
          std::abs(box.left_axis.y) *
              box.half_width_m +
          std::abs(box.up_axis.y) *
              box.half_height_m,
      std::abs(box.longitudinal_axis.z) *
              box.half_length_m +
          std::abs(box.left_axis.z) *
              box.half_width_m +
          std::abs(box.up_axis.z) *
              box.half_height_m,
  };
}

void populate_box_aabb(
    PlacedComponentBox* box
) {
  const Vec3 radius = box_aabb_radius(*box);
  box->aabb_min = Vec3{
      box->center.x - radius.x,
      box->center.y - radius.y,
      box->center.z - radius.z,
  };
  box->aabb_max = Vec3{
      box->center.x + radius.x,
      box->center.y + radius.y,
      box->center.z + radius.z,
  };
  box->aabb_available = true;
  box->z_min_abs_m = box->aabb_min.z;
  box->z_max_abs_m = box->aabb_max.z;
}

void expand_sweep_aabb(
    SweptVolume* volume,
    const PlacedComponentBox& box
) {
  if (!box.aabb_available) {
    return;
  }

  if (!volume->aabb_available) {
    volume->aabb_min = box.aabb_min;
    volume->aabb_max = box.aabb_max;
    volume->aabb_available = true;
    return;
  }

  volume->aabb_min.x =
      std::min(volume->aabb_min.x, box.aabb_min.x);
  volume->aabb_min.y =
      std::min(volume->aabb_min.y, box.aabb_min.y);
  volume->aabb_min.z =
      std::min(volume->aabb_min.z, box.aabb_min.z);
  volume->aabb_max.x =
      std::max(volume->aabb_max.x, box.aabb_max.x);
  volume->aabb_max.y =
      std::max(volume->aabb_max.y, box.aabb_max.y);
  volume->aabb_max.z =
      std::max(volume->aabb_max.z, box.aabb_max.z);
}

void validate_geometry(
    const VehicleSweepGeometry& geometry
) {
  if (!(geometry.pivot_base_m > 0.0) ||
      !(geometry.body_length_m >=
        geometry.pivot_base_m) ||
      !(geometry.body_width_m > 0.0)) {
    throw std::invalid_argument(
        "invalid vehicle sweep geometry"
    );
  }

  for (const auto& component : geometry.components) {
    if (!(component.half_length_m > 0.0) ||
        !(component.half_width_m > 0.0) ||
        !(component.z_max_m >
          component.z_min_m)) {
      throw std::invalid_argument(
          "invalid vehicle component box"
      );
    }
  }
}

}  // namespace

bool SweptVolume::contains(const Vec3& point) const {
  if (aabb_available &&
      (point.x < aabb_min.x ||
       point.x > aabb_max.x ||
       point.y < aabb_min.y ||
       point.y > aabb_max.y ||
       point.z < aabb_min.z ||
       point.z > aabb_max.z)) {
    return false;
  }

  for (const auto& box : boxes) {
    if (box.aabb_available) {
      if (point.x < box.aabb_min.x ||
          point.x > box.aabb_max.x ||
          point.y < box.aabb_min.y ||
          point.y > box.aabb_max.y ||
          point.z < box.aabb_min.z ||
          point.z > box.aabb_max.z) {
        continue;
      }
    } else if (point.z < box.z_min_abs_m ||
               point.z > box.z_max_abs_m) {
      continue;
    }

    const Vec3 delta{
        point.x - box.center.x,
        point.y - box.center.y,
        point.z - box.center.z,
    };
    const double longitudinal =
        dot_3d(delta, box.longitudinal_axis);
    const double lateral =
        dot_3d(delta, box.left_axis);
    const double vertical =
        dot_3d(delta, box.up_axis);

    if (std::abs(longitudinal) <=
            box.half_length_m &&
        std::abs(lateral) <=
            box.half_width_m &&
        std::abs(vertical) <=
            box.half_height_m) {
      return true;
    }
  }
  return false;
}

bool PossibleSweptVolume::contains(
    const Vec3& point
) const {
  for (const auto& branch : branches) {
    if (branch.contains(point)) {
      return true;
    }
  }
  return false;
}

SweptVolume build_swept_volume(
    const SweepBuildRequest& request,
    const VehicleSweepGeometry& geometry,
    const SweepMargins& margins
) {
  validate_geometry(geometry);

  if (request.path == nullptr) {
    throw std::invalid_argument(
        "SweepBuildRequest path is null"
    );
  }
  if (!(request.sample_step_m > 0.0) ||
      request.front_end_chainage_m <
          request.front_start_chainage_m ||
      !std::isfinite(request.cant_rad)) {
    throw std::invalid_argument(
        "invalid sweep sampling interval"
    );
  }
  if (margins.lateral_m < 0.0 ||
      margins.vertical_m < 0.0 ||
      margins.longitudinal_m < 0.0) {
    throw std::invalid_argument(
        "sweep margins must be non-negative"
    );
  }

  SweptVolume out;
  out.margins = margins;
  out.sampled_front_start_chainage_m =
      request.front_start_chainage_m;
  out.sampled_front_end_chainage_m =
      request.front_end_chainage_m;
  out.sample_step_m = request.sample_step_m;

  // Half a sample step closes the longitudinal gap between adjacent sampled
  // placements. Additional uncertainty is supplied explicitly in margins.
  const double sampling_bridge_m =
      0.5 * request.sample_step_m;

  const double epsilon =
      request.sample_step_m * 1e-9 + 1e-12;

  for (double front_chainage =
           request.front_start_chainage_m;
       front_chainage <=
           request.front_end_chainage_m + epsilon;
       front_chainage += request.sample_step_m) {
    const double clamped_front =
        std::min(
            front_chainage,
            request.front_end_chainage_m
        );

    const auto placement =
        place_planar_body(
            *request.path,
            clamped_front,
            geometry.pivot_base_m,
            geometry.body_length_m,
            geometry.body_width_m
        );
    if (!placement.has_value()) {
      throw std::runtime_error(
          "unable to place vehicle on supplied path"
      );
    }

    for (const auto& component :
         geometry.components) {
      const auto anchor =
          select_anchor(
              component,
              *request.path,
              *placement,
              request.cant_rad
          );

      const double center_vertical_m =
          0.5 * (
              component.z_min_m +
              component.z_max_m
          );
      const Vec3 center =
          add(
              add(
                  add(
                      anchor.origin,
                      scale(
                          anchor.longitudinal,
                          component.center_longitudinal_m
                      )
                  ),
                  scale(
                      anchor.left,
                      component.center_lateral_m
                  )
              ),
              scale(
                  anchor.up,
                  center_vertical_m
              )
          );

      PlacedComponentBox box;
      box.name = component.name;
      box.branch_index = request.branch_index;
      box.front_chainage_m = clamped_front;
      box.center = center;
      box.longitudinal_axis =
          anchor.longitudinal;
      box.left_axis = anchor.left;
      box.up_axis = anchor.up;
      box.half_length_m =
          component.half_length_m +
          margins.longitudinal_m +
          sampling_bridge_m;
      box.half_width_m =
          component.half_width_m +
          margins.lateral_m;
      box.half_height_m =
          0.5 * (
              component.z_max_m -
              component.z_min_m
          ) +
          margins.vertical_m;

      populate_box_aabb(&box);
      expand_sweep_aabb(&out, box);
      out.boxes.push_back(std::move(box));
    }

    if (clamped_front >=
        request.front_end_chainage_m) {
      break;
    }
  }

  return out;
}

PossibleSweptVolume build_possible_swept_volume(
    const std::vector<SweepBuildRequest>& requests,
    const VehicleSweepGeometry& geometry,
    const SweepMargins& margins
) {
  PossibleSweptVolume out;
  out.branch_ambiguous = requests.size() > 1;
  out.branches.reserve(requests.size());

  for (const auto& request : requests) {
    out.branches.push_back(
        build_swept_volume(
            request,
            geometry,
            margins
        )
    );
  }

  return out;
}

}  // namespace metropilot
