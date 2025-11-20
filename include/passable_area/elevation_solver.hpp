#ifndef ELEVATION_SOLVER_HPP_
#define ELEVATION_SOLVER_HPP_

#include "common_types.hpp"

#include <limits>

#include <rclcpp/rclcpp.hpp>

class ElevationSolver {
public:
  static ElevationSolverResult
  solve(const ElevationSolverContext &context,
        const ElevationSolverParams &params,
        const rclcpp::Logger &logger = rclcpp::get_logger("ElevationSolver"));
};

#endif // PASSABLE_AREA_ELEVATION_SOLVER_HPP_
