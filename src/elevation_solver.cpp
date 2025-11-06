#include "elevation_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

float binTop(float min_height, float bin_width, int bin) noexcept {
  return min_height + (static_cast<float>(bin) + 1.0f) * bin_width;
}

float binCenter(float min_height, float bin_width, int bin) noexcept {
  return min_height + (static_cast<float>(bin) + 0.5f) * bin_width;
}

float binBottom(float min_height, float bin_width, int bin) noexcept {
  return min_height + static_cast<float>(bin) * bin_width;
}

int linearIndex(int r, int c, int cols) noexcept { return r * cols + c; }

float binMaxHeight(const std::vector<float> &peaks, int linear, int bins,
                   int bin) {
  if (linear < 0 || bin < 0)
    return std::numeric_limits<float>::quiet_NaN();
  const std::size_t idx =
      static_cast<std::size_t>(linear) * static_cast<std::size_t>(bins) +
      static_cast<std::size_t>(bin);
  if (idx >= peaks.size())
    return std::numeric_limits<float>::quiet_NaN();
  return peaks[idx];
}

int findGroundBin(const uint16_t *cell_hist, int bins, int total_points,
                  int empty_threshold, bool &gap_found) {
  gap_found = false;
  if (total_points <= 0 || bins <= 0)
    return -1;

  std::vector<bool> has_above(static_cast<std::size_t>(bins), false);
  bool any_above = false;
  for (int b = bins - 1; b >= 0; --b) {
    has_above[b] = any_above;
    if (static_cast<int>(cell_hist[b]) > empty_threshold) {
      any_above = true;
    }
  }

  int last_non_empty = -1;
  for (int b = 0; b < bins; ++b) {
    if (static_cast<int>(cell_hist[b]) > empty_threshold) {
      last_non_empty = b;
    } else if (last_non_empty >= 0 && has_above[b]) {
      gap_found = true;
      return last_non_empty;
    }
  }

  return last_non_empty;
}

int findCeilingBin(const uint16_t *cell_hist, int bins, int ground_bin,
                   bool gap_found, int window_bins, int min_density) {
  const int start_bin = std::max(ground_bin + 1, 0);
  const int end_bin = bins - window_bins;
  for (int start = start_bin; start <= end_bin; ++start) {
    int window_sum = 0;
    for (int k = 0; k < window_bins; ++k) {
      window_sum += cell_hist[start + k];
    }
    if (window_sum >= min_density)
      return start;
  }

  if (!gap_found)
    return ground_bin;

  return -1;
}

int computeMaxGap(const uint16_t *cell_hist, int ground_bin, int ceiling_bin,
                  int empty_threshold) {
  int max_gap = 0;
  int current_gap = 0;
  for (int b = ground_bin + 1; b < ceiling_bin; ++b) {
    if (static_cast<int>(cell_hist[b]) <= empty_threshold) {
      current_gap++;
      max_gap = std::max(max_gap, current_gap);
    } else {
      current_gap = 0;
    }
  }
  return max_gap;
}

float computeClusterRatio(const uint16_t *cell_hist, int bins, int ceiling_bin,
                          int total_points) {
  if (total_points <= 0)
    return 1.0f;
  int cluster_points = 0;
  for (int b = ceiling_bin; b < bins; ++b) {
    cluster_points += cell_hist[b];
  }
  return static_cast<float>(cluster_points) / static_cast<float>(total_points);
}

int countNeighborSupport(const std::vector<float> &ceiling_buffer,
                         const std::vector<bool> &ceiling_found, int rows,
                         int cols, int r, int c, float ceiling_z,
                         float tolerance) {
  int support = 0;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0)
        continue;
      const int nr = r + dr;
      const int nc = c + dc;
      if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
        continue;

      const int nidx = linearIndex(nr, nc, cols);
      if (!ceiling_found[nidx])
        continue;

      const float neighbor = ceiling_buffer[nidx];
      if (!std::isfinite(neighbor))
        continue;

      if (std::fabs(neighbor - ceiling_z) <= tolerance)
        support++;
    }
  }
  return support;
}

} // namespace

ElevationSolverResult
ElevationSolver::solve(const ElevationSolverContext &ctx,
                       const ElevationSolverParams &params,
                       const rclcpp::Logger &logger) {
  ElevationSolverResult result;

  if (ctx.rows <= 0 || ctx.cols <= 0 || ctx.bins <= 0 ||
      ctx.bin_width <= 0.0f || ctx.counts == nullptr || ctx.peaks == nullptr) {
    RCLCPP_WARN(logger,
                "ElevationSolver received invalid context configuration.");
    return result;
  }

  const auto &histogram = *ctx.counts;
  const auto &hist_max = *ctx.peaks;
  const std::size_t cells =
      static_cast<std::size_t>(ctx.rows) * static_cast<std::size_t>(ctx.cols);
  const std::size_t expected_size = cells * static_cast<std::size_t>(ctx.bins);

  if (histogram.size() != expected_size || hist_max.size() != expected_size) {
    RCLCPP_WARN(logger,
                "Histogram buffers have unexpected size count=%zu max=%zu, "
                "expected %zu",
                histogram.size(), hist_max.size(), expected_size);
    return result;
  }

  result.ground.assign(cells, std::numeric_limits<float>::quiet_NaN());
  result.ceiling.assign(cells, std::numeric_limits<float>::quiet_NaN());
  result.clearance.assign(cells, std::numeric_limits<float>::infinity());
  result.float_mask.assign(cells, static_cast<uint8_t>(0));

  std::vector<float> ceiling_buffer(cells,
                                    std::numeric_limits<float>::quiet_NaN());
  std::vector<bool> ceiling_found(cells, false);
  std::vector<float> ratio_buffer(cells, 1.0f);
  std::vector<int> gap_buffer(cells, 0);

  const int bins = ctx.bins;
  const float min_height = ctx.min_height;
  const float bin_width = ctx.bin_width;

  // Legacy path bypasses advanced heuristics.
  if (params.use_legacy_vertical) {
    for (int r = 0; r < ctx.rows; ++r) {
      for (int c = 0; c < ctx.cols; ++c) {
        const int linear = linearIndex(r, c, ctx.cols);
        const uint16_t *cell_hist =
            &histogram[static_cast<std::size_t>(linear) *
                       static_cast<std::size_t>(bins)];

        int min_bin = -1;
        int max_bin = -1;
        for (int b = 0; b < bins; ++b) {
          if (cell_hist[b] > 0) {
            if (min_bin < 0)
              min_bin = b;
            max_bin = b;
          }
        }
        if (min_bin < 0)
          continue;

        float ground_z = binMaxHeight(hist_max, linear, bins, min_bin);
        if (!std::isfinite(ground_z)) {
          ground_z = binBottom(min_height, bin_width, min_bin);
        }
        float ceiling_z = binMaxHeight(hist_max, linear, bins, max_bin);
        if (!std::isfinite(ceiling_z)) {
          ceiling_z = binTop(min_height, bin_width, max_bin);
        }

        result.ground[linear] = ground_z;
        result.ceiling[linear] = ceiling_z;
        result.clearance[linear] = std::max(ceiling_z - ground_z, 0.0f);
      }
    }
    return result;
  }

  const int empty_threshold = std::max(0, params.gap_empty_count_threshold);

  for (int r = 0; r < ctx.rows; ++r) {
    for (int c = 0; c < ctx.cols; ++c) {
      const int linear = linearIndex(r, c, ctx.cols);
      const uint16_t *cell_hist = &histogram[static_cast<std::size_t>(linear) *
                                             static_cast<std::size_t>(bins)];

      int total_points = 0;
      for (int b = 0; b < bins; ++b) {
        total_points += cell_hist[b];
      }
      if (total_points < std::max(1, params.min_points))
        continue;

      bool gap_found = false;
      const int ground_bin = findGroundBin(cell_hist, bins, total_points,
                                           empty_threshold, gap_found);
      if (ground_bin < 0)
        continue;

      float ground_peak = binMaxHeight(hist_max, linear, bins, ground_bin);
      const float ground_z = std::isfinite(ground_peak)
                                 ? ground_peak
                                 : binCenter(min_height, bin_width, ground_bin);

      result.ground[linear] = ground_z;

      const int ceiling_bin =
          findCeilingBin(cell_hist, bins, ground_bin, gap_found,
                         std::max(1, params.ceiling_window_bins),
                         std::max(1, params.ceiling_min_points));

      if (ceiling_bin < 0) {
        int legacy_max_bin = -1;
        for (int b = bins - 1; b >= 0; --b) {
          if (cell_hist[b] > 0) {
            legacy_max_bin = b;
            break;
          }
        }
        if (legacy_max_bin >= 0) {
          float fallback_peak =
              binMaxHeight(hist_max, linear, bins, legacy_max_bin);
          const float fallback_ceiling =
              std::isfinite(fallback_peak)
                  ? fallback_peak
                  : binTop(min_height, bin_width, legacy_max_bin);

          ceiling_buffer[linear] = fallback_ceiling;
          ceiling_found[linear] = true;
          gap_buffer[linear] = 0;
          ratio_buffer[linear] = 1.0f;

          result.ceiling[linear] = fallback_ceiling;
          result.clearance[linear] =
              std::max(fallback_ceiling - ground_z, 0.0f);
        } else {
          result.clearance[linear] = std::numeric_limits<float>::infinity();
        }
        continue;
      }

      float ceiling_peak = binMaxHeight(hist_max, linear, bins, ceiling_bin);
      const float ceiling_z =
          std::isfinite(ceiling_peak)
              ? ceiling_peak
              : binCenter(min_height, bin_width, ceiling_bin);

      ceiling_buffer[linear] = ceiling_z;
      ceiling_found[linear] = true;

      gap_buffer[linear] =
          computeMaxGap(cell_hist, ground_bin, ceiling_bin, empty_threshold);

      ratio_buffer[linear] =
          computeClusterRatio(cell_hist, bins, ceiling_bin, total_points);

      result.ceiling[linear] = ceiling_z;
      result.clearance[linear] = std::max(ceiling_z - ground_z, 0.0f);
    }
  }

  const int required_gap = std::max(0, params.gap_empty_bins);
  const float ratio_threshold = params.float_ratio_threshold;
  const int neighbor_requirement = std::max(0, params.neighbor_min_support);
  const float neighbor_tol = std::max(0.0f, params.neighbor_height_tolerance);

  for (int r = 0; r < ctx.rows; ++r) {
    for (int c = 0; c < ctx.cols; ++c) {
      const int linear = linearIndex(r, c, ctx.cols);
      const float ground_z = result.ground[linear];
      if (!std::isfinite(ground_z))
        continue;

      if (!ceiling_found[linear]) {
        result.clearance[linear] = std::numeric_limits<float>::infinity();
        result.ceiling[linear] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      const float ceiling_z = ceiling_buffer[linear];
      const int max_gap = gap_buffer[linear];
      const float ratio = ratio_buffer[linear];

      const bool gap_condition =
          (required_gap == 0) ? true : (max_gap >= required_gap);
      const bool sparse_condition = ratio <= ratio_threshold;

      const int neighbor_support =
          countNeighborSupport(ceiling_buffer, ceiling_found, ctx.rows,
                               ctx.cols, r, c, ceiling_z, neighbor_tol);

      const bool weak_neighbor_support =
          neighbor_requirement > 0 && neighbor_support < neighbor_requirement;

      const bool treat_as_float =
          gap_condition && sparse_condition &&
          (neighbor_requirement == 0 ? true : weak_neighbor_support);

      if (treat_as_float) {
        result.float_mask[linear] = 1;
        result.ceiling[linear] = std::numeric_limits<float>::quiet_NaN();
        result.clearance[linear] = std::numeric_limits<float>::infinity();
      } else {
        result.float_mask[linear] = 0;
        result.ceiling[linear] = ceiling_z;
        result.clearance[linear] = std::max(ceiling_z - ground_z, 0.0f);
      }
    }
  }

  return result;
}
