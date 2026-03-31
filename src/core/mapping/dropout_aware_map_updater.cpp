#include "passable_area/core/mapping/dropout_aware_map_updater.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace passable_area::core {

std::vector<int> DropoutAwareMapUpdater::update(const FrontendOutput &frontend_output,
                                                const FrameObservability &observability,
                                                LocalTerrainMap &map) const {
  map.ageCells();
  auto &layers = map.layers();
  std::unordered_set<int> dirty_set;
  dirty_set.reserve(frontend_output.support_candidates.size() +
                    frontend_output.obstacle_candidates.size());
  std::vector<uint8_t> touched_support(map.size(), 0);
  std::vector<uint8_t> touched_obstacle(map.size(), 0);

  const auto sector_state_for_cell = [&](int cell) {
    const auto cell_center = map.indexToWorld(cell);
    const float angle = std::atan2(cell_center.y() - map.center().y(), cell_center.x() - map.center().x());
    const float normalized = angle + static_cast<float>(M_PI);
    const int sector = std::clamp(
        static_cast<int>(std::floor(normalized /
                                    (2.0f * static_cast<float>(M_PI) /
                                     static_cast<float>(observability.sectors.size())))),
        0, static_cast<int>(observability.sectors.size()) - 1);
    return observability.sectors[sector];
  };

  for (const auto &candidate : frontend_output.support_candidates) {
    const auto sector = sector_state_for_cell(candidate.cell);
    if (sector.state == ObservabilityState::kBlindByStructure) {
      continue;
    }
    layers.support_height[candidate.cell] = candidate.z;
    layers.support_confidence[candidate.cell] =
        std::clamp(layers.support_confidence[candidate.cell] +
                       config_.persistence.support_confidence_gain * candidate.confidence,
                   0.0f, 1.0f);
    layers.coverage_confidence[candidate.cell] =
        std::max(layers.coverage_confidence[candidate.cell], sector.coverage_confidence);
    layers.support_state[candidate.cell] = static_cast<uint8_t>(SupportState::kObserved);
    layers.last_observed_age[candidate.cell] = 0;
    if (sector.state == ObservabilityState::kObserved) {
      layers.last_reliable_age[candidate.cell] = 0;
    }
    layers.last_sector_state[candidate.cell] = static_cast<uint8_t>(sector.state);
    touched_support[candidate.cell] = 1U;
    dirty_set.insert(candidate.cell);
  }

  for (const auto &candidate : frontend_output.obstacle_candidates) {
    const auto sector = sector_state_for_cell(candidate.cell);
    if (sector.state == ObservabilityState::kBlindByStructure) {
      continue;
    }
    layers.overhead_height[candidate.cell] = candidate.z;
    layers.overhead_confidence[candidate.cell] =
        std::clamp(layers.overhead_confidence[candidate.cell] + config_.persistence.obstacle_evidence_gain,
                   0.0f, 1.0f);
    layers.obstacle_evidence[candidate.cell] =
        std::clamp(layers.obstacle_evidence[candidate.cell] +
                       config_.persistence.obstacle_evidence_gain * candidate.evidence,
                   0.0f, 1.0f);
    layers.coverage_confidence[candidate.cell] =
        std::max(layers.coverage_confidence[candidate.cell], sector.coverage_confidence);
    layers.last_sector_state[candidate.cell] = static_cast<uint8_t>(sector.state);
    touched_obstacle[candidate.cell] = 1U;
    dirty_set.insert(candidate.cell);
  }

  for (int cell = 0; cell < map.size(); ++cell) {
    const auto sector = sector_state_for_cell(cell);
    layers.coverage_confidence[cell] = std::max(layers.coverage_confidence[cell] * 0.92f,
                                                sector.coverage_confidence * 0.85f);
    if (!touched_support[cell]) {
      float support_decay = 0.0f;
      if (sector.state == ObservabilityState::kObserved) {
        support_decay = config_.persistence.support_confidence_decay * 0.8f;
      } else if (sector.state == ObservabilityState::kPartiallyObserved) {
        support_decay = config_.persistence.support_confidence_decay * 0.2f;
      } else if (sector.state == ObservabilityState::kMissingByDropout) {
        support_decay = config_.persistence.support_confidence_decay * 0.05f;
      }
      layers.support_confidence[cell] =
          std::max(0.0f, layers.support_confidence[cell] - support_decay);
    }
    if (!touched_obstacle[cell]) {
      float obstacle_decay = 0.0f;
      if (sector.state == ObservabilityState::kObserved) {
        obstacle_decay = config_.persistence.obstacle_evidence_decay * 0.4f;
      } else if (sector.state == ObservabilityState::kPartiallyObserved) {
        obstacle_decay = config_.persistence.obstacle_evidence_decay * 0.1f;
      } else if (sector.state == ObservabilityState::kMissingByDropout) {
        obstacle_decay = config_.persistence.obstacle_evidence_decay * 0.03f;
      }
      layers.obstacle_evidence[cell] =
          std::max(0.0f, layers.obstacle_evidence[cell] - obstacle_decay);
    }

    if (touched_support[cell]) {
      continue;
    }

    const bool persistent_allowed =
        layers.last_reliable_age[cell] <=
        static_cast<uint16_t>(std::max(1, config_.persistence.support_persistence_frames));
    if (layers.support_confidence[cell] >= config_.observability.min_support_confidence &&
        persistent_allowed) {
      layers.support_state[cell] = static_cast<uint8_t>(SupportState::kPersistent);
    } else if (layers.support_confidence[cell] <=
                   config_.observability.min_support_confidence * 0.5f ||
               !persistent_allowed) {
      layers.support_state[cell] = static_cast<uint8_t>(SupportState::kNone);
      layers.support_height[cell] = std::numeric_limits<float>::quiet_NaN();
      layers.overhead_height[cell] = layers.obstacle_evidence[cell] > 0.1f
                                         ? layers.overhead_height[cell]
                                         : std::numeric_limits<float>::quiet_NaN();
    }
  }

  std::vector<int> dirty_cells(dirty_set.begin(), dirty_set.end());
  return dirty_cells;
}

} // namespace passable_area::core
