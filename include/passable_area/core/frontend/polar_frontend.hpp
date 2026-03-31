#ifndef PASSABLE_AREA_CORE_FRONTEND_POLAR_FRONTEND_HPP_
#define PASSABLE_AREA_CORE_FRONTEND_POLAR_FRONTEND_HPP_

#include "passable_area/core/frontend/candidate_types.hpp"
#include "passable_area/core/mapping/local_terrain_map.hpp"
#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/frame_types.hpp"

namespace passable_area::core {

class PolarFrontend {
public:
  explicit PolarFrontend(const Config &config) : config_(config) {}

  FrontendOutput run(const FrameInput &frame, const FrameObservability &observability,
                     const LocalTerrainMap &map) const;

private:
  Config config_;
};

} // namespace passable_area::core

#endif
