#ifndef PASSABLE_AREA_CORE_TRAVERSABILITY_SOLVER_HPP_
#define PASSABLE_AREA_CORE_TRAVERSABILITY_SOLVER_HPP_

#include "passable_area/core/mapping/local_terrain_map.hpp"
#include "passable_area/core/types/config_types.hpp"

namespace passable_area::core {

class TraversabilitySolver {
public:
  explicit TraversabilitySolver(const Config &config) : config_(config) {}

  void update(LocalTerrainMap &map) const;

private:
  Config config_;
};

} // namespace passable_area::core

#endif
