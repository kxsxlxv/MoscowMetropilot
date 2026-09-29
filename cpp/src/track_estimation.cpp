#include "metropilot/track_estimation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace metropilot {
namespace {

double median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t mid = values.size() / 2;
  std::nth_element(
      values.begin(),
      values.begin() + static_cast<std::ptrdiff_t>(mid),
      values.end()
  );
  const double hi = values[mid];
  if (values.size() % 2 == 1) {
    return hi;
  }
  const double lo = *std::max_element(
      values.begin(),
      values.begin() + static_cast<std::ptrdiff_t>(mid)
  );
  return 0.5 * (lo + hi);
}

bool finite_point(const Vec3& p) {
  return std::isfinite(p.x) &&
         std::isfinite(p.y) &&
         std::isfinite(p.z);
}

struct Peak {
  std::size_t bin{};
  double center_y{};
  std::size_t count{};
};

bool solve3x3(
    double a[3][4],
    double* x0,
    double* x1,
    double* x2
) {
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 3; ++row) {
      if (std::abs(a[row][col]) >
          std::abs(a[pivot][col])) {
        pivot = row;
      }
    }
    if (std::abs(a[pivot][col]) < 1e-12) {
      return false;
    }
    if (pivot != col) {
      for (int k = col; k < 4; ++k) {
        std::swap(a[pivot][k], a[col][k]);
      }
    }

    const double inv = 1.0 / a[col][col];
    for (int k = col; k < 4; ++k) {
      a[col][k] *= inv;
    }

    for (int row = 0; row < 3; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = a[row][col];
      for (int k = col; k < 4; ++k) {
        a[row][k] -= factor * a[col][k];
      }
    }
  }

  *x0 = a[0][3];
  *x1 = a[1][3];
  *x2 = a[2][3];
  return true;
}

struct SparsePrediction {
  double center_y_m{};
  double separation_m{};
  double center_z_m{};
};

bool predict_sparse_support(
    const std::vector<RailSupportPair>& pairs,
    double query_x_m,
    SparsePrediction* prediction
) {
  if (pairs.size() < 4 || prediction == nullptr) {
    return false;
  }

  // Keep the model local. Older support from a previous alignment segment
  // should not dominate the sparse continuation.
  const std::size_t first =
      pairs.size() > 12 ? pairs.size() - 12 : 0;

  double s0 = 0.0;
  double s1 = 0.0;
  double s2 = 0.0;
  double s3 = 0.0;
  double s4 = 0.0;
  double cy0 = 0.0;
  double cy1 = 0.0;
  double cy2 = 0.0;
  double cz0 = 0.0;
  double cz1 = 0.0;

  std::vector<double> separations;
  separations.reserve(
      std::min<std::size_t>(5, pairs.size() - first)
  );

  for (std::size_t i = first; i < pairs.size(); ++i) {
    const auto& pair = pairs[i];
    const double x = pair.forward_m;
    const double y =
        0.5 * (pair.right_y_m + pair.left_y_m);
    const double z =
        0.5 * (pair.right_z_m + pair.left_z_m);
    const double x2 = x * x;

    s0 += 1.0;
    s1 += x;
    s2 += x2;
    s3 += x2 * x;
    s4 += x2 * x2;
    cy0 += y;
    cy1 += x * y;
    cy2 += x2 * y;
    cz0 += z;
    cz1 += x * z;

    if (i + 5 >= pairs.size()) {
      separations.push_back(pair.separation_m);
    }
  }

  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  double matrix[3][4] = {
      {s4, s3, s2, cy2},
      {s3, s2, s1, cy1},
      {s2, s1, s0, cy0},
  };
  if (!solve3x3(matrix, &a, &b, &c)) {
    return false;
  }

  const double vertical_det =
      s2 * s0 - s1 * s1;
  if (std::abs(vertical_det) < 1e-12) {
    return false;
  }
  const double vertical_m =
      (cz1 * s0 - cz0 * s1) / vertical_det;
  const double vertical_c =
      (s2 * cz0 - s1 * cz1) / vertical_det;

  prediction->center_y_m =
      a * query_x_m * query_x_m +
      b * query_x_m +
      c;
  prediction->separation_m =
      median(std::move(separations));
  prediction->center_z_m =
      vertical_m * query_x_m +
      vertical_c;

  return
      std::isfinite(prediction->center_y_m) &&
      std::isfinite(prediction->separation_m) &&
      std::isfinite(prediction->center_z_m);
}

double spread(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  const auto [lo, hi] =
      std::minmax_element(values.begin(), values.end());
  return *hi - *lo;
}

}  // namespace

RailSupportExtractor::RailSupportExtractor(
    RailSupportConfig config
) : config_(std::move(config)) {
  if (!(config_.forward_max_m >
        config_.forward_min_m) ||
      !(config_.forward_bin_m > 0.0) ||
      !(config_.lateral_abs_max_m > 0.0) ||
      !(config_.tor_z_max_m >
        config_.tor_z_min_m) ||
      !(config_.lateral_histogram_bin_m > 0.0) ||
      !(config_.refine_half_width_m > 0.0) ||
      !(config_.max_pair_separation_m >
        config_.min_pair_separation_m) ||
      !(config_.sparse_forward_max_m >=
        config_.forward_max_m) ||
      !(config_.sparse_near_bin_m > 0.0) ||
      !(config_.sparse_medium_bin_m > 0.0) ||
      !(config_.sparse_far_bin_m > 0.0) ||
      !(config_.sparse_lateral_abs_max_m > 0.0) ||
      !(config_.sparse_lateral_base_window_m > 0.0) ||
      !(config_.sparse_lateral_max_window_m >=
        config_.sparse_lateral_base_window_m) ||
      !(config_.sparse_vertical_base_window_m > 0.0) ||
      !(config_.sparse_vertical_max_window_m >=
        config_.sparse_vertical_base_window_m) ||
      config_.sparse_min_seed_pairs < 4 ||
      config_.sparse_min_points_per_rail < 2 ||
      config_.sparse_min_forward_spread_m < 0.0) {
    throw std::invalid_argument(
        "invalid RailSupportConfig"
    );
  }
}

RailSupportResult RailSupportExtractor::extract(
    const std::vector<Vec3>& points
) const {
  RailSupportResult out;

  const std::size_t forward_bins =
      static_cast<std::size_t>(
          std::ceil(
              (config_.forward_max_m -
               config_.forward_min_m) /
              config_.forward_bin_m
          )
      );

  std::vector<std::vector<const Vec3*>> buckets(
      forward_bins
  );
  std::vector<const Vec3*> sparse_points;
  sparse_points.reserve(points.size() / 8 + 1);

  for (const auto& p : points) {
    if (finite_point(p) &&
        p.x >= config_.forward_min_m &&
        p.x < config_.sparse_forward_max_m &&
        std::abs(p.y) <=
            config_.sparse_lateral_abs_max_m) {
      sparse_points.push_back(&p);
    }
    if (!finite_point(p) ||
        p.x < config_.forward_min_m ||
        p.x >= config_.forward_max_m ||
        std::abs(p.y) >
            config_.lateral_abs_max_m ||
        p.z < config_.tor_z_min_m ||
        p.z > config_.tor_z_max_m) {
      continue;
    }

    const auto index =
        static_cast<std::size_t>(
            (p.x - config_.forward_min_m) /
            config_.forward_bin_m
        );
    if (index < buckets.size()) {
      buckets[index].push_back(&p);
    }
  }

  const double lateral_min =
      -config_.lateral_abs_max_m;
  const std::size_t lateral_bins =
      static_cast<std::size_t>(
          std::ceil(
              2.0 * config_.lateral_abs_max_m /
              config_.lateral_histogram_bin_m
          )
      );

  std::size_t consecutive_missing = 0;

  for (std::size_t bin_index = 0;
       bin_index < buckets.size();
       ++bin_index) {
    ++out.visited_bins;
    const auto& bucket = buckets[bin_index];

    std::vector<std::size_t> histogram(
        lateral_bins,
        0
    );
    for (const auto* p : bucket) {
      const auto h =
          static_cast<std::size_t>(
              (p->y - lateral_min) /
              config_.lateral_histogram_bin_m
          );
      if (h < histogram.size()) {
        ++histogram[h];
      }
    }

    std::vector<Peak> peaks;
    if (histogram.size() >= 3) {
      for (std::size_t i = 1;
           i + 1 < histogram.size();
           ++i) {
        if (histogram[i] <
            config_.min_peak_points) {
          continue;
        }
        if (histogram[i] >= histogram[i - 1] &&
            histogram[i] >= histogram[i + 1]) {
          peaks.push_back(
              Peak{
                  i,
                  lateral_min +
                      (static_cast<double>(i) + 0.5) *
                          config_.lateral_histogram_bin_m,
                  histogram[i],
              }
          );
        }
      }
    }

    std::sort(
        peaks.begin(),
        peaks.end(),
        [](const Peak& a, const Peak& b) {
          return a.count > b.count;
        }
    );
    if (peaks.size() > 64) {
      peaks.resize(64);
    }

    const double nominal_forward =
        config_.forward_min_m +
        (static_cast<double>(bin_index) + 0.5) *
            config_.forward_bin_m;

    double predicted_right =
        config_.initial_right_y_m;
    double predicted_left =
        config_.initial_left_y_m;

    if (!out.pairs.empty()) {
      predicted_right =
          out.pairs.back().right_y_m;
      predicted_left =
          out.pairs.back().left_y_m;
    }

    if (out.pairs.size() >= 2) {
      const auto& a =
          out.pairs[out.pairs.size() - 2];
      const auto& b =
          out.pairs[out.pairs.size() - 1];
      const double dx =
          b.forward_m - a.forward_m;
      if (std::abs(dx) > 1e-6) {
        const double query_dx =
            nominal_forward - b.forward_m;
        predicted_right =
            b.right_y_m +
            (b.right_y_m - a.right_y_m) /
                dx * query_dx;
        predicted_left =
            b.left_y_m +
            (b.left_y_m - a.left_y_m) /
                dx * query_dx;
      }
    }

    bool found = false;
    Peak best_right;
    Peak best_left;
    double best_score =
        -std::numeric_limits<double>::infinity();
    double best_error{};

    for (const auto& right : peaks) {
      for (const auto& left : peaks) {
        if (!(right.center_y < left.center_y)) {
          continue;
        }

        const double separation =
            left.center_y - right.center_y;
        if (separation <
                config_.min_pair_separation_m ||
            separation >
                config_.max_pair_separation_m) {
          continue;
        }

        const double prediction_error =
            std::abs(
                right.center_y - predicted_right
            ) +
            std::abs(
                left.center_y - predicted_left
            );
        if (prediction_error >
            config_.max_pair_prediction_error_m) {
          continue;
        }

        const double score =
            static_cast<double>(
                right.count + left.count
            ) -
            config_.continuity_penalty_points_per_m *
                prediction_error;

        if (!found || score > best_score) {
          found = true;
          best_right = right;
          best_left = left;
          best_score = score;
          best_error = prediction_error;
        }
      }
    }

    if (!found) {
      ++out.rejected_or_missing_bins;
      ++consecutive_missing;
      if (consecutive_missing >
          config_.max_consecutive_missing_bins) {
        break;
      }
      continue;
    }

    std::vector<double> right_x;
    std::vector<double> right_y;
    std::vector<double> right_z;
    std::vector<double> left_x;
    std::vector<double> left_y;
    std::vector<double> left_z;

    for (const auto* p : bucket) {
      if (std::abs(
              p->y - best_right.center_y
          ) <= config_.refine_half_width_m) {
        right_x.push_back(p->x);
        right_y.push_back(p->y);
        right_z.push_back(p->z);
      }
      if (std::abs(
              p->y - best_left.center_y
          ) <= config_.refine_half_width_m) {
        left_x.push_back(p->x);
        left_y.push_back(p->y);
        left_z.push_back(p->z);
      }
    }

    if (right_y.size() <
            config_.min_peak_points ||
        left_y.size() <
            config_.min_peak_points) {
      ++out.rejected_or_missing_bins;
      ++consecutive_missing;
      if (consecutive_missing >
          config_.max_consecutive_missing_bins) {
        break;
      }
      continue;
    }

    RailSupportPair pair;
    pair.right_y_m = median(right_y);
    pair.left_y_m = median(left_y);
    pair.right_z_m = median(right_z);
    pair.left_z_m = median(left_z);
    pair.right_support_points = right_y.size();
    pair.left_support_points = left_y.size();
    pair.separation_m =
        pair.left_y_m - pair.right_y_m;
    pair.prediction_error_m = best_error;

    std::vector<double> combined_x =
        std::move(right_x);
    combined_x.insert(
        combined_x.end(),
        left_x.begin(),
        left_x.end()
    );
    pair.forward_m = median(combined_x);

    if (out.pairs.size() >=
        config_.separation_baseline_min_pairs) {
      std::vector<double> separations;
      separations.reserve(out.pairs.size());
      for (const auto& accepted : out.pairs) {
        separations.push_back(
            accepted.separation_m
        );
      }
      const double baseline =
          median(std::move(separations));
      if (std::abs(
              pair.separation_m - baseline
          ) > config_.max_separation_drift_m) {
        ++out.rejected_or_missing_bins;
        ++consecutive_missing;
        if (consecutive_missing >
            config_.max_consecutive_missing_bins) {
          break;
        }
        continue;
      }
    }

    out.pairs.push_back(pair);
    out.supported_lookahead_m =
        std::max(
            out.supported_lookahead_m,
            pair.forward_m
        );
    consecutive_missing = 0;
  }

  if (config_.enable_sparse_extension &&
      out.pairs.size() >= config_.sparse_min_seed_pairs &&
      out.supported_lookahead_m <
          config_.sparse_forward_max_m) {
    const double dense_guard =
        out.pairs.back().forward_m +
        0.5 * config_.forward_bin_m;
    double bin_start =
        config_.sparse_near_bin_m *
        std::ceil(
            dense_guard /
            config_.sparse_near_bin_m
        );

    std::size_t sparse_missing = 0;

    while (bin_start <
           config_.sparse_forward_max_m) {
      double bin_width =
          config_.sparse_near_bin_m;
      if (bin_start >=
          config_.sparse_far_start_m) {
        bin_width =
            config_.sparse_far_bin_m;
      } else if (
          bin_start >=
          config_.sparse_medium_start_m) {
        bin_width =
            config_.sparse_medium_bin_m;
      }

      const double bin_end =
          std::min(
              config_.sparse_forward_max_m,
              bin_start + bin_width
          );
      const double query_x =
          0.5 * (bin_start + bin_end);
      ++out.sparse_extension_bins;

      SparsePrediction prediction;
      if (!predict_sparse_support(
              out.pairs,
              query_x,
              &prediction)) {
        break;
      }

      if (std::abs(prediction.center_y_m) >
          config_.sparse_lateral_abs_max_m) {
        break;
      }

      const double lateral_window =
          std::min(
              config_.sparse_lateral_max_window_m,
              config_.sparse_lateral_base_window_m +
                  config_.sparse_lateral_window_per_m *
                      query_x
          );
      const double vertical_window =
          std::min(
              config_.sparse_vertical_max_window_m,
              config_.sparse_vertical_base_window_m +
                  config_.sparse_vertical_window_per_m *
                      query_x
          );

      const double predicted_right =
          prediction.center_y_m -
          0.5 * prediction.separation_m;
      const double predicted_left =
          prediction.center_y_m +
          0.5 * prediction.separation_m;

      std::vector<double> right_x;
      std::vector<double> right_y;
      std::vector<double> right_z;
      std::vector<double> left_x;
      std::vector<double> left_y;
      std::vector<double> left_z;

      for (const auto* point : sparse_points) {
        if (point->x < bin_start ||
            point->x >= bin_end ||
            std::abs(
                point->z -
                prediction.center_z_m
            ) > vertical_window) {
          continue;
        }

        if (std::abs(
                point->y - predicted_right
            ) <= lateral_window) {
          right_x.push_back(point->x);
          right_y.push_back(point->y);
          right_z.push_back(point->z);
        }
        if (std::abs(
                point->y - predicted_left
            ) <= lateral_window) {
          left_x.push_back(point->x);
          left_y.push_back(point->y);
          left_z.push_back(point->z);
        }
      }

      bool accepted = false;
      if (right_y.size() >=
              config_.sparse_min_points_per_rail &&
          left_y.size() >=
              config_.sparse_min_points_per_rail &&
          (spread(right_x) >=
               config_.sparse_min_forward_spread_m ||
           right_y.size() >
               config_.sparse_min_points_per_rail) &&
          (spread(left_x) >=
               config_.sparse_min_forward_spread_m ||
           left_y.size() >
               config_.sparse_min_points_per_rail)) {
        RailSupportPair pair;
        pair.forward_m =
            median([&]() {
              std::vector<double> x = right_x;
              x.insert(
                  x.end(),
                  left_x.begin(),
                  left_x.end()
              );
              return x;
            }());
        pair.right_y_m = median(right_y);
        pair.left_y_m = median(left_y);
        pair.right_z_m = median(right_z);
        pair.left_z_m = median(left_z);
        pair.right_support_points =
            right_y.size();
        pair.left_support_points =
            left_y.size();
        pair.separation_m =
            pair.left_y_m - pair.right_y_m;
        pair.prediction_error_m =
            std::abs(
                pair.right_y_m -
                predicted_right
            ) +
            std::abs(
                pair.left_y_m -
                predicted_left
            );
        pair.sparse_extension = true;

        const double center_z =
            0.5 * (
                pair.right_z_m +
                pair.left_z_m
            );
        const bool separation_ok =
            pair.separation_m >=
                config_.min_pair_separation_m &&
            pair.separation_m <=
                config_.max_pair_separation_m &&
            std::abs(
                pair.separation_m -
                prediction.separation_m
            ) <=
                config_.sparse_max_separation_drift_m;
        const bool vertical_ok =
            std::abs(
                center_z -
                prediction.center_z_m
            ) <= vertical_window;

        if (separation_ok && vertical_ok) {
          out.pairs.push_back(pair);
          out.supported_lookahead_m =
              std::max(
                  out.supported_lookahead_m,
                  pair.forward_m
              );
          ++out.sparse_extension_pairs;
          sparse_missing = 0;
          accepted = true;
        }
      }

      if (!accepted) {
        ++sparse_missing;
        if (sparse_missing >
            config_.sparse_max_consecutive_missing_bins) {
          break;
        }
      }

      bin_start = bin_end;
    }
  }

  return out;
}

LocalTrackEstimator::LocalTrackEstimator(
    LocalTrackEstimatorConfig config
) : config_(std::move(config)) {}

LocalTrackHypothesis LocalTrackEstimator::estimate(
    const RailSupportResult& supports
) const {
  LocalTrackHypothesis out;
  out.supported_lookahead_m =
      supports.supported_lookahead_m;

  if (supports.pairs.size() <
      config_.min_support_pairs) {
    out.reason = "insufficient rail-support pairs";
    return out;
  }
  if (supports.supported_lookahead_m <
      config_.min_supported_lookahead_m) {
    out.reason = "insufficient supported lookahead";
    return out;
  }

  double s0 = 0.0;
  double s1 = 0.0;
  double s2 = 0.0;
  double s3 = 0.0;
  double s4 = 0.0;
  double ty0 = 0.0;
  double ty1 = 0.0;
  double ty2 = 0.0;
  double tz0 = 0.0;
  double tz1 = 0.0;

  std::vector<double> separations;
  std::vector<double> cants;
  std::size_t total_support_points = 0;

  struct FitPoint {
    double x{};
    double y{};
    double z{};
    double w{};
  };
  std::vector<FitPoint> fit_points;
  fit_points.reserve(supports.pairs.size());

  for (const auto& pair : supports.pairs) {
    const double x = pair.forward_m;
    const double y =
        0.5 * (
            pair.left_y_m +
            pair.right_y_m
        );
    const double z =
        0.5 * (
            pair.left_z_m +
            pair.right_z_m
        );
    const double w =
        static_cast<double>(
            pair.left_support_points +
            pair.right_support_points
        );

    const double x2 = x * x;
    s0 += w;
    s1 += w * x;
    s2 += w * x2;
    s3 += w * x2 * x;
    s4 += w * x2 * x2;
    ty0 += w * y;
    ty1 += w * x * y;
    ty2 += w * x2 * y;
    tz0 += w * z;
    tz1 += w * x * z;

    fit_points.push_back(
        FitPoint{x, y, z, w}
    );
    separations.push_back(pair.separation_m);
    cants.push_back(
        std::atan2(
            pair.left_z_m - pair.right_z_m,
            pair.separation_m
        )
    );
    total_support_points +=
        pair.left_support_points +
        pair.right_support_points;
  }

  double matrix[3][4] = {
      {s4, s3, s2, ty2},
      {s3, s2, s1, ty1},
      {s2, s1, s0, ty0},
  };
  if (!solve3x3(
          matrix,
          &out.lateral_a_inv_m,
          &out.lateral_b,
          &out.lateral_c_m)) {
    out.reason = "singular lateral path fit";
    return out;
  }

  const double det = s2 * s0 - s1 * s1;
  if (std::abs(det) < 1e-12) {
    out.reason = "singular vertical path fit";
    return out;
  }
  out.vertical_m =
      (tz1 * s0 - tz0 * s1) / det;
  out.vertical_c_m =
      (s2 * tz0 - s1 * tz1) / det;

  double lateral_sq = 0.0;
  double vertical_sq = 0.0;
  double weight_sum = 0.0;

  out.centerline_samples.reserve(
      fit_points.size()
  );
  double chainage = 0.0;
  Vec3 previous{};
  bool have_previous = false;

  for (const auto& p : fit_points) {
    const double y_fit =
        out.lateral_a_inv_m * p.x * p.x +
        out.lateral_b * p.x +
        out.lateral_c_m;
    const double z_fit =
        out.vertical_m * p.x +
        out.vertical_c_m;
    const double dy = p.y - y_fit;
    const double dz = p.z - z_fit;

    lateral_sq += p.w * dy * dy;
    vertical_sq += p.w * dz * dz;
    weight_sum += p.w;

    // Downstream sweep geometry receives the smoothed fitted centerline,
    // not raw midpoint jitter from individual support bins.
    const Vec3 center{p.x, y_fit, z_fit};
    if (!have_previous) {
      chainage = std::hypot(center.x, center.y);
      have_previous = true;
    } else {
      chainage += std::hypot(
          center.x - previous.x,
          center.y - previous.y
      );
    }
    out.centerline_samples.push_back(
        PathSample{chainage, center}
    );
    previous = center;
  }

  out.quality.lateral_rms_m =
      std::sqrt(lateral_sq / weight_sum);
  out.quality.vertical_rms_m =
      std::sqrt(vertical_sq / weight_sum);
  out.quality.support_pair_count =
      supports.pairs.size();
  out.quality.total_support_points =
      total_support_points;

  const double separation_median =
      median(separations);
  std::vector<double> abs_deviation;
  abs_deviation.reserve(separations.size());
  for (const double separation : separations) {
    abs_deviation.push_back(
        std::abs(
            separation - separation_median
        )
    );
  }
  out.quality.rail_separation_median_m =
      separation_median;
  out.quality.rail_separation_mad_m =
      median(std::move(abs_deviation));

  out.heading_rad = std::atan(out.lateral_b);
  out.curvature_inv_m =
      2.0 * out.lateral_a_inv_m /
      std::pow(
          1.0 +
              out.lateral_b *
                  out.lateral_b,
          1.5
      );
  out.grade =
      out.vertical_m /
      std::sqrt(
          1.0 +
          out.lateral_b *
              out.lateral_b
      );
  out.cant_rad = median(std::move(cants));

  if (out.quality.lateral_rms_m >
      config_.max_lateral_rms_m) {
    out.reason = "lateral path fit residual too large";
    return out;
  }
  if (out.quality.vertical_rms_m >
      config_.max_vertical_rms_m) {
    out.reason = "vertical path fit residual too large";
    return out;
  }

  out.valid = true;
  out.reason = "ok";
  return out;
}


LocalTrackHypothesisExtractor::LocalTrackHypothesisExtractor(
    LocalTrackHypothesisSetConfig config
) : config_(std::move(config)),
    primary_extractor_(config_.rail_support),
    estimator_(config_.track_fit) {
  if (!(config_.branch_forward_max_m >
        config_.branch_forward_min_m) ||
      !(config_.branch_bin_m > 0.0) ||
      !(config_.branch_lateral_abs_max_m > 0.0) ||
      !(config_.branch_vertical_window_m > 0.0) ||
      !(config_.branch_peak_merge_distance_m > 0.0) ||
      !(config_.branch_min_lateral_divergence_m > 0.0) ||
      !(config_.branch_seed_max_lateral_divergence_m >=
        config_.branch_min_lateral_divergence_m) ||
      !(config_.branch_min_final_lateral_divergence_m >=
        config_.branch_min_lateral_divergence_m) ||
      config_.branch_min_divergence_growth_m < 0.0 ||
      !(config_.branch_max_prediction_error_m > 0.0) ||
      !(config_.branch_max_separation_drift_m > 0.0) ||
      config_.branch_min_pairs < 2 ||
      config_.branch_min_span_m < 0.0) {
    throw std::invalid_argument(
        "invalid LocalTrackHypothesisSetConfig"
    );
  }
}

LocalTrackHypothesisSet LocalTrackHypothesisExtractor::estimate(
    const std::vector<Vec3>& points
) const {
  LocalTrackHypothesisSet out;

  const auto primary_support =
      primary_extractor_.extract(points);
  const auto primary_track =
      estimator_.estimate(primary_support);

  TrackBranchHypothesis primary;
  primary.primary = true;
  primary.branch_onset_m = 0.0;
  primary.branch_final_offset_m = 0.0;
  primary.support = primary_support;
  primary.track = primary_track;
  out.branches.push_back(std::move(primary));

  if (!primary_track.valid ||
      config_.max_secondary_hypotheses == 0) {
    return out;
  }

  const double scan_start =
      std::max(
          config_.branch_forward_min_m,
          config_.rail_support.forward_min_m
      );
  const double scan_end =
      std::min(
          {
              config_.branch_forward_max_m,
              config_.rail_support.sparse_forward_max_m,
              primary_track.supported_lookahead_m,
          }
      );
  if (!(scan_end > scan_start)) {
    return out;
  }

  const double baseline_separation =
      primary_track.quality.rail_separation_median_m;
  if (!std::isfinite(baseline_separation)) {
    return out;
  }

  auto primary_center_y =
      [&](double x) {
        return
            primary_track.lateral_a_inv_m * x * x +
            primary_track.lateral_b * x +
            primary_track.lateral_c_m;
      };
  auto primary_center_z =
      [&](double x) {
        return
            primary_track.vertical_m * x +
            primary_track.vertical_c_m;
      };

  struct BranchPeak {
    double y{};
    std::size_t count{};
  };
  struct BranchCandidate {
    RailSupportPair pair;
    double center_y{};
    double center_z{};
    double offset_from_primary{};
    std::size_t support_points{};
  };
  struct BranchChain {
    std::vector<RailSupportPair> pairs;
    std::size_t missing_bins{};
    double first_offset{};
  };

  const std::size_t branch_bucket_count =
      static_cast<std::size_t>(
          std::ceil(
              (scan_end - scan_start) /
              config_.branch_bin_m
          )
      );
  std::vector<std::vector<const Vec3*>>
      branch_buckets(branch_bucket_count);

  // One O(N) pass per frame. The previous implementation rescanned the full
  // cloud for every 2 m branch bin, which made turnout extraction O(N*bins).
  for (const auto& point : points) {
    if (!finite_point(point) ||
        point.x < scan_start ||
        point.x >= scan_end ||
        std::abs(point.y) >
            config_.branch_lateral_abs_max_m ||
        std::abs(
            point.z -
            primary_center_z(point.x)
        ) >
            config_.branch_vertical_window_m) {
      continue;
    }

    const auto bin_index =
        static_cast<std::size_t>(
            (point.x - scan_start) /
            config_.branch_bin_m
        );
    if (bin_index < branch_buckets.size()) {
      branch_buckets[bin_index].push_back(
          &point
      );
    }
  }

  auto candidates_in_bin =
      [&](std::size_t branch_bin_index) {
        const auto& bucket =
            branch_buckets[branch_bin_index];

        const double hist_bin =
            config_.rail_support.lateral_histogram_bin_m;
        const double lateral_min =
            -config_.branch_lateral_abs_max_m;
        const std::size_t lateral_bins =
            static_cast<std::size_t>(
                std::ceil(
                    2.0 *
                    config_.branch_lateral_abs_max_m /
                    hist_bin
                )
            );

        std::vector<std::size_t> histogram(
            lateral_bins,
            0
        );
        for (const auto* point : bucket) {
          const auto index =
              static_cast<std::size_t>(
                  (point->y - lateral_min) /
                  hist_bin
              );
          if (index < histogram.size()) {
            ++histogram[index];
          }
        }

        std::vector<BranchPeak> peaks;
        if (histogram.size() >= 3) {
          for (std::size_t i = 1;
               i + 1 < histogram.size();
               ++i) {
            if (histogram[i] <
                config_.rail_support.min_peak_points) {
              continue;
            }
            if (histogram[i] >= histogram[i - 1] &&
                histogram[i] >= histogram[i + 1]) {
              peaks.push_back(
                  BranchPeak{
                      lateral_min +
                          (static_cast<double>(i) + 0.5) *
                              hist_bin,
                      histogram[i],
                  }
              );
            }
          }
        }

        // Consolidate local maxima that belong to the same physical lateral
        // ridge. This is especially important in double-track curves: a broad
        // rail/track feature can otherwise contribute multiple nearby peaks,
        // and arbitrary combinations can form a fake smoothly diverging pair.
        std::sort(
            peaks.begin(),
            peaks.end(),
            [](const BranchPeak& a,
               const BranchPeak& b) {
              return a.y < b.y;
            }
        );

        std::vector<BranchPeak> merged_peaks;
        for (std::size_t i = 0;
             i < peaks.size();) {
          std::size_t j = i + 1;
          BranchPeak representative = peaks[i];

          while (j < peaks.size() &&
                 peaks[j].y -
                         peaks[j - 1].y <=
                     config_
                         .branch_peak_merge_distance_m) {
            if (peaks[j].count >
                representative.count) {
              representative = peaks[j];
            }
            ++j;
          }

          merged_peaks.push_back(
              representative
          );
          i = j;
        }

        std::sort(
            merged_peaks.begin(),
            merged_peaks.end(),
            [](const BranchPeak& a,
               const BranchPeak& b) {
              return a.count > b.count;
            }
        );
        if (merged_peaks.size() > 64) {
          merged_peaks.resize(64);
        }

        std::vector<BranchCandidate> candidates;
        for (const auto& right : merged_peaks) {
          for (const auto& left : merged_peaks) {
            if (!(right.y < left.y)) {
              continue;
            }

            const double coarse_separation =
                left.y - right.y;
            if (coarse_separation <
                    config_.rail_support
                        .min_pair_separation_m ||
                coarse_separation >
                    config_.rail_support
                        .max_pair_separation_m ||
                std::abs(
                    coarse_separation -
                    baseline_separation
                ) >
                    config_.branch_max_separation_drift_m +
                    2.0 * hist_bin) {
              continue;
            }

            std::vector<double> right_x;
            std::vector<double> right_y;
            std::vector<double> right_z;
            std::vector<double> left_x;
            std::vector<double> left_y;
            std::vector<double> left_z;

            for (const auto* point : bucket) {
              if (std::abs(point->y - right.y) <=
                  config_.rail_support.refine_half_width_m) {
                right_x.push_back(point->x);
                right_y.push_back(point->y);
                right_z.push_back(point->z);
              }
              if (std::abs(point->y - left.y) <=
                  config_.rail_support.refine_half_width_m) {
                left_x.push_back(point->x);
                left_y.push_back(point->y);
                left_z.push_back(point->z);
              }
            }

            if (right_y.size() <
                    config_.rail_support.min_peak_points ||
                left_y.size() <
                    config_.rail_support.min_peak_points) {
              continue;
            }

            RailSupportPair pair;
            pair.right_y_m = median(right_y);
            pair.left_y_m = median(left_y);
            pair.right_z_m = median(right_z);
            pair.left_z_m = median(left_z);
            pair.right_support_points = right_y.size();
            pair.left_support_points = left_y.size();
            pair.separation_m =
                pair.left_y_m - pair.right_y_m;

            std::vector<double> combined_x = right_x;
            combined_x.insert(
                combined_x.end(),
                left_x.begin(),
                left_x.end()
            );
            pair.forward_m = median(std::move(combined_x));

            const double center_y =
                0.5 *
                (pair.right_y_m + pair.left_y_m);
            const double center_z =
                0.5 *
                (pair.right_z_m + pair.left_z_m);
            const double offset =
                center_y -
                primary_center_y(pair.forward_m);

            if (std::abs(offset) <
                    config_
                        .branch_min_lateral_divergence_m ||
                std::abs(
                    pair.separation_m -
                    baseline_separation
                ) >
                    config_
                        .branch_max_separation_drift_m ||
                std::abs(
                    center_z -
                    primary_center_z(pair.forward_m)
                ) >
                    config_.branch_vertical_window_m) {
              continue;
            }

            pair.prediction_error_m =
                std::abs(offset);

            candidates.push_back(
                BranchCandidate{
                    pair,
                    center_y,
                    center_z,
                    offset,
                    pair.right_support_points +
                        pair.left_support_points,
                }
            );
          }
        }

        std::sort(
            candidates.begin(),
            candidates.end(),
            [&](const BranchCandidate& a,
                const BranchCandidate& b) {
              const double a_sep =
                  std::abs(
                      a.pair.separation_m -
                      baseline_separation
                  );
              const double b_sep =
                  std::abs(
                      b.pair.separation_m -
                      baseline_separation
                  );
              if (a_sep != b_sep) {
                return a_sep < b_sep;
              }
              return
                  a.support_points >
                  b.support_points;
            }
        );

        std::vector<BranchCandidate> deduplicated;
        for (const auto& candidate : candidates) {
          bool duplicate = false;
          for (const auto& accepted : deduplicated) {
            if (std::abs(
                    candidate.center_y -
                    accepted.center_y
                ) <
                std::max(
                    0.04,
                    2.0 * hist_bin
                )) {
              duplicate = true;
              break;
            }
          }
          if (!duplicate) {
            deduplicated.push_back(candidate);
          }
          if (deduplicated.size() >= 32) {
            break;
          }
        }
        return deduplicated;
      };

  std::vector<BranchChain> active;
  std::vector<BranchChain> finished;

  for (double bin_start = scan_start;
       bin_start < scan_end;
       bin_start += config_.branch_bin_m) {
    const double bin_end =
        std::min(
            scan_end,
            bin_start + config_.branch_bin_m
        );
    const auto branch_bin_index =
        static_cast<std::size_t>(
            (bin_start - scan_start) /
            config_.branch_bin_m
        );
    if (branch_bin_index >=
        branch_buckets.size()) {
      break;
    }
    const auto candidates =
        candidates_in_bin(
            branch_bin_index
        );
    std::vector<bool> used(
        candidates.size(),
        false
    );
    std::vector<BranchChain> next;
    next.reserve(
        active.size() + candidates.size()
    );

    for (auto chain : active) {
      double predicted_center =
          0.5 *
          (chain.pairs.back().right_y_m +
           chain.pairs.back().left_y_m);

      if (chain.pairs.size() >= 2) {
        const auto& a =
            chain.pairs[chain.pairs.size() - 2];
        const auto& b =
            chain.pairs[chain.pairs.size() - 1];
        const double ax =
            0.5 * (a.right_y_m + a.left_y_m);
        const double bx =
            0.5 * (b.right_y_m + b.left_y_m);
        const double dx =
            b.forward_m - a.forward_m;
        if (std::abs(dx) > 1e-6) {
          const double query_x =
              0.5 * (bin_start + bin_end);
          predicted_center =
              bx +
              (bx - ax) / dx *
                  (query_x - b.forward_m);
        }
      }

      std::size_t best_index =
          candidates.size();
      double best_score =
          std::numeric_limits<double>::infinity();

      for (std::size_t i = 0;
           i < candidates.size();
           ++i) {
        const auto& candidate = candidates[i];
        if (chain.first_offset *
                candidate.offset_from_primary <
            0.0) {
          continue;
        }

        const double center_error =
            std::abs(
                candidate.center_y -
                predicted_center
            );
        const double separation_error =
            std::abs(
                candidate.pair.separation_m -
                chain.pairs.back().separation_m
            );

        if (center_error >
                config_
                    .branch_max_prediction_error_m ||
            separation_error >
                config_
                    .branch_max_separation_drift_m) {
          continue;
        }

        const double score =
            center_error +
            2.0 * separation_error -
            0.001 *
                static_cast<double>(
                    candidate.support_points
                );
        if (score < best_score) {
          best_score = score;
          best_index = i;
        }
      }

      if (best_index < candidates.size()) {
        chain.pairs.push_back(
            candidates[best_index].pair
        );
        chain.missing_bins = 0;
        used[best_index] = true;
        next.push_back(std::move(chain));
      } else {
        ++chain.missing_bins;
        if (chain.missing_bins <=
            config_.branch_max_missing_bins) {
          next.push_back(std::move(chain));
        } else {
          finished.push_back(std::move(chain));
        }
      }
    }

    for (std::size_t i = 0;
         i < candidates.size();
         ++i) {
      if (used[i] ||
          std::abs(
              candidates[i].offset_from_primary
          ) >
              config_
                  .branch_seed_max_lateral_divergence_m) {
        continue;
      }
      BranchChain chain;
      chain.pairs.push_back(candidates[i].pair);
      chain.first_offset =
          candidates[i].offset_from_primary;
      next.push_back(std::move(chain));
    }

    std::sort(
        next.begin(),
        next.end(),
        [](const BranchChain& a,
           const BranchChain& b) {
          if (a.pairs.size() != b.pairs.size()) {
            return a.pairs.size() > b.pairs.size();
          }
          const auto center =
              [](const RailSupportPair& pair) {
                return
                    0.5 *
                    (pair.right_y_m +
                     pair.left_y_m);
              };
          return
              std::abs(center(a.pairs.back())) >
              std::abs(center(b.pairs.back()));
        }
    );

    const std::size_t max_active =
        std::max<std::size_t>(
            8,
            config_.max_secondary_hypotheses * 8
        );
    if (next.size() > max_active) {
      for (std::size_t i = max_active;
           i < next.size();
           ++i) {
        finished.push_back(
            std::move(next[i])
        );
      }
      next.resize(max_active);
    }
    active = std::move(next);
  }

  for (auto& chain : active) {
    finished.push_back(std::move(chain));
  }

  std::sort(
      finished.begin(),
      finished.end(),
      [](const BranchChain& a,
         const BranchChain& b) {
        const double a_span =
            a.pairs.size() >= 2
                ? a.pairs.back().forward_m -
                      a.pairs.front().forward_m
                : 0.0;
        const double b_span =
            b.pairs.size() >= 2
                ? b.pairs.back().forward_m -
                      b.pairs.front().forward_m
                : 0.0;
        if (a_span != b_span) {
          return a_span > b_span;
        }
        return a.pairs.size() > b.pairs.size();
      }
  );

  std::vector<double> accepted_final_offsets;
  for (const auto& chain : finished) {
    if (out.branches.size() >=
        1 + config_.max_secondary_hypotheses) {
      break;
    }
    if (chain.pairs.size() <
        config_.branch_min_pairs) {
      continue;
    }

    const double span =
        chain.pairs.back().forward_m -
        chain.pairs.front().forward_m;
    if (span < config_.branch_min_span_m) {
      continue;
    }

    const double first_center =
        0.5 *
        (chain.pairs.front().right_y_m +
         chain.pairs.front().left_y_m);
    const double final_center =
        0.5 *
        (chain.pairs.back().right_y_m +
         chain.pairs.back().left_y_m);
    const double first_offset =
        first_center -
        primary_center_y(
            chain.pairs.front().forward_m
        );
    const double final_offset =
        final_center -
        primary_center_y(
            chain.pairs.back().forward_m
        );

    if (first_offset * final_offset <= 0.0 ||
        std::abs(final_offset) <
            config_
                .branch_min_final_lateral_divergence_m ||
        std::abs(final_offset) -
                std::abs(first_offset) <
            config_
                .branch_min_divergence_growth_m) {
      continue;
    }

    bool duplicate = false;
    for (const double accepted_offset :
         accepted_final_offsets) {
      if (std::abs(
              accepted_offset - final_offset
          ) < 0.20) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      continue;
    }

    RailSupportResult branch_support;
    for (const auto& pair :
         primary_support.pairs) {
      if (pair.forward_m >=
          chain.pairs.front().forward_m) {
        break;
      }
      branch_support.pairs.push_back(pair);
    }
    branch_support.pairs.insert(
        branch_support.pairs.end(),
        chain.pairs.begin(),
        chain.pairs.end()
    );
    branch_support.supported_lookahead_m =
        chain.pairs.back().forward_m;
    branch_support.visited_bins =
        primary_support.visited_bins;
    branch_support.rejected_or_missing_bins =
        primary_support.rejected_or_missing_bins;

    const auto branch_track =
        estimator_.estimate(branch_support);
    if (!branch_track.valid) {
      continue;
    }

    TrackBranchHypothesis branch;
    branch.primary = false;
    branch.branch_onset_m =
        chain.pairs.front().forward_m;
    branch.branch_final_offset_m =
        final_offset;
    branch.support =
        std::move(branch_support);
    branch.track = branch_track;
    out.branches.push_back(std::move(branch));
    accepted_final_offsets.push_back(
        final_offset
    );
  }

  out.branch_ambiguous =
      out.branches.size() > 1;
  return out;
}




}  // namespace metropilot
