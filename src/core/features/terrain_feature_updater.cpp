#include "passable_area/core/features/terrain_feature_updater.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace passable_area::core {

void TerrainFeatureUpdater::update(const std::vector<int> &dirty_cells,
                                   LocalTerrainMap &map) const {
  auto &layers = map.layers();
  std::unordered_set<int> expanded;
  for (const int cell : dirty_cells) {
    expanded.insert(cell);
    const int row = cell / map.cols();
    const int col = cell % map.cols();
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        const int nr = row + dr;
        const int nc = col + dc;
        if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
          continue;
        }
        expanded.insert(nr * map.cols() + nc);
      }
    }
  }

  for (const int cell : expanded) {
    const float h = layers.support_height[cell];
    if (!std::isfinite(h)) {
      layers.slope[cell] = 0.0f;
      layers.step_up[cell] = 0.0f;
      layers.step_down[cell] = 0.0f;
      layers.roughness[cell] = 0.0f;
      layers.clearance[cell] = std::numeric_limits<float>::quiet_NaN();
      layers.support_continuity[cell] = 0.0f;
      continue;
    }

    float max_up = 0.0f;
    float max_down = 0.0f;
    float rough = 0.0f;
    int valid_neighbors = 0;
    const int row = cell / map.cols();
    const int col = cell % map.cols();
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0) {
          continue;
        }
        const int nr = row + dr;
        const int nc = col + dc;
        if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
          continue;
        }
        const int neighbor = nr * map.cols() + nc;
        const float nh = layers.support_height[neighbor];
        if (!std::isfinite(nh)) {
          continue;
        }
        const float dz = nh - h;
        max_up = std::max(max_up, dz);
        max_down = std::max(max_down, -dz);
        rough += dz * dz;
        ++valid_neighbors;
      }
    }
    layers.step_up[cell] = max_up;
    layers.step_down[cell] = max_down;
    layers.roughness[cell] =
        valid_neighbors > 0
            ? std::sqrt(rough / static_cast<float>(valid_neighbors))
            : 0.0f;
    layers.slope[cell] = std::atan(max_up / std::max(map.resolution(), 1e-3f)) *
                         180.0f / static_cast<float>(M_PI);
    layers.clearance[cell] = std::isfinite(layers.overhead_height[cell])
                                 ? layers.overhead_height[cell] - h
                                 : std::numeric_limits<float>::infinity();
    layers.support_continuity[cell] = static_cast<float>(valid_neighbors) /
                                      8.0f * layers.support_confidence[cell];
  }
}

} // namespace passable_area::core
