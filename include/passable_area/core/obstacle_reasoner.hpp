#ifndef PASSABLE_AREA_CORE_OBSTACLE_REASONER_HPP_
#define PASSABLE_AREA_CORE_OBSTACLE_REASONER_HPP_

#include "passable_area/core/mapping/terrain_layers.hpp"
#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/obstacle_types.hpp"

#include <vector>

namespace passable_area::core {

struct ObstacleReasonerOutput {
  std::vector<uint8_t> block_reason;
  std::vector<uint8_t> protrusion_stage;
  std::vector<uint8_t> overhead_stage;
  std::vector<uint8_t> obstacle_point_publish_status;
};

class ObstacleReasoner {
public:
  explicit ObstacleReasoner(const Config &config) : config_(config) {}

  ObstacleReasonerOutput evaluate(const TerrainLayers &layers) const;

private:
  Config config_;
};

} // namespace passable_area::core

#endif
