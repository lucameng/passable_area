#ifndef PASSABLE_AREA_CORE_MAPPING_DROPOUT_AWARE_MAP_UPDATER_HPP_
#define PASSABLE_AREA_CORE_MAPPING_DROPOUT_AWARE_MAP_UPDATER_HPP_

#include "passable_area/core/candidate_types.hpp"
#include "passable_area/core/mapping/local_terrain_map.hpp"
#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/frame_types.hpp"

#include <vector>

namespace passable_area::core {

class DropoutAwareMapUpdater {
public:
  explicit DropoutAwareMapUpdater(const Config &config) : config_(config) {}

  std::vector<int> update(const FrontendOutput &frontend_output,
                          const FrameObservability &observability,
                          LocalTerrainMap &map) const;

private:
  Config config_;
};

} // namespace passable_area::core

#endif
