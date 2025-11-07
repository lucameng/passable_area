#ifndef PASSABLE_AREA_ELEVATION_SOLVER_HPP_
#define PASSABLE_AREA_ELEVATION_SOLVER_HPP_

#include <cstdint>
#include <limits>
#include <vector>

#include <rclcpp/rclcpp.hpp>

struct ElevationSolverParams {
  bool use_legacy_elevation = false;
  int min_points = 1;
  int ceiling_window_bins = 2;
  int ceiling_min_points = 1;
  int gap_empty_bins = 2;
  int gap_empty_count_threshold = 0;
  int ground_min_count = 1;
  float float_ratio_threshold = 0.5f;
  int neighbor_min_support = 1;
  float neighbor_height_tolerance = 0.25f;
};

struct ElevationSolverContext {
  int rows = 0;
  int cols = 0;
  int bins = 0;
  float min_height = 0.0f;
  float bin_width = 0.0f;
  const std::vector<uint16_t> *counts = nullptr;
  const std::vector<float> *peaks = nullptr;
  const std::vector<uint8_t> *region_mask = nullptr;
};

struct ElevationSolverRegion {
  bool enabled = false;
  float min_x = 0.0f;
  float max_x = 0.0f;
  float min_y = 0.0f;
  float max_y = 0.0f;
};

struct ElevationSolverResult {
  std::vector<float> ground;
  std::vector<float> ceiling;
  std::vector<float> clearance;
  std::vector<uint8_t> float_mask;
};

class ElevationSolver {
public:
  static ElevationSolverResult
  solve(const ElevationSolverContext &context,
        const ElevationSolverParams &params,
        const rclcpp::Logger &logger = rclcpp::get_logger("ElevationSolver"));
};

#endif // PASSABLE_AREA_ELEVATION_SOLVER_HPP_