#ifndef PASSABLE_AREA_CORE_TERRAIN_FEATURE_UPDATER_HPP_
#define PASSABLE_AREA_CORE_TERRAIN_FEATURE_UPDATER_HPP_

#include "passable_area/core/mapping/local_terrain_map.hpp"
#include "passable_area/core/types/config_types.hpp"

#include <vector>

namespace passable_area::core {

class TerrainFeatureUpdater {
public:
  explicit TerrainFeatureUpdater(const Config &config) : config_(config) {}

  void update(const std::vector<int> &dirty_cells, LocalTerrainMap &map) const;

private:
  Config config_;
};

} // namespace passable_area::core

#endif
