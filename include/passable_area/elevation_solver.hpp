#ifndef ELEVATION_SOLVER_HPP_
#define ELEVATION_SOLVER_HPP_

#include "common_types.hpp"

#include <limits>

class ElevationSolver {
public:
  static ElevationSolverResult
  solve(const ElevationSolverContext &context,
        const ElevationSolverParams &params);
};

#endif // PASSABLE_AREA_ELEVATION_SOLVER_HPP_
