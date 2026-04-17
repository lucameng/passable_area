#ifndef PASSABLE_AREA_CORE_MAPPING_TERRAIN_LAYERS_HPP_
#define PASSABLE_AREA_CORE_MAPPING_TERRAIN_LAYERS_HPP_

#include "passable_area/core/types/state_types.hpp"

#include <vector>

namespace passable_area::core {

struct TerrainLayers {
  std::vector<float> support_height;
  std::vector<float> support_confidence;
  std::vector<float> overhead_height;
  std::vector<float> overhead_confidence;
  std::vector<float> obstacle_evidence;
  std::vector<float> coverage_confidence;
  std::vector<float> slope;
  std::vector<float> step_up;
  std::vector<float> step_down;
  std::vector<float> roughness;
  std::vector<float> clearance;
  std::vector<float> support_continuity;
  std::vector<uint8_t> support_state;
  std::vector<uint8_t> obstacle_publishable;
  std::vector<int8_t> passability_state;
  std::vector<int8_t> traversal_cost;
  std::vector<uint8_t> last_sector_state;
  std::vector<uint16_t> last_observed_age;
  std::vector<uint16_t> last_reliable_age;
};

} // namespace passable_area::core

#endif
