#include "passable_area/core/mapping/dropout_aware_map_updater.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_set>

namespace passable_area::core {
namespace {

void ClearObstacleLayer(TerrainLayers &layers, int cell) {
  layers.overhead_height[cell] = std::numeric_limits<float>::quiet_NaN();
  layers.overhead_confidence[cell] = 0.0f;
}

float ObstacleEvidenceGainScaleForSemantic(const ObstacleCandidate &candidate,
                                           const Config &config) {
  switch (candidate.semantic) {
  case ObstacleCandidateSemantic::kConfirmedFacade:
    return std::max(
        1.0f,
        config.persistence.confirmed_facade_obstacle_evidence_gain_scale);
  case ObstacleCandidateSemantic::kDefault:
  default:
    return 1.0f;
  }
}

float ObstacleEvidenceForSemantic(const ObstacleCandidate &candidate,
                                  const Config &config) {
  switch (candidate.semantic) {
  case ObstacleCandidateSemantic::kConfirmedFacade:
    return std::max(candidate.evidence,
                    config.persistence.confirmed_facade_obstacle_min_evidence);
  case ObstacleCandidateSemantic::kDefault:
  default:
    return candidate.evidence;
  }
}

bool IsStrongObstacleCell(int cell, const TerrainLayers &layers,
                          const Config &config) {
  return layers.obstacle_evidence[cell] >= config.obstacle_points_min_evidence &&
         std::isfinite(layers.overhead_height[cell]);
}

bool IsCellHeightPublishable(int cell, const TerrainLayers &layers,
                             const Config &config) {
  if (!std::isfinite(layers.overhead_height[cell])) {
    return false;
  }
  if (!std::isfinite(layers.support_height[cell])) {
    return true;
  }
  return layers.overhead_height[cell] >=
         layers.support_height[cell] + config.obstacle_points_min_height;
}

bool AreCellsStructurallyConnected(int lhs_cell, int rhs_cell,
                                   const TerrainLayers &layers,
                                   const Config &config) {
  if (!std::isfinite(layers.overhead_height[lhs_cell]) ||
      !std::isfinite(layers.overhead_height[rhs_cell])) {
    return false;
  }

  const float overhead_tolerance =
      std::max({config.map.resolution, config.geometry.max_step_up,
                config.geometry.support_anchor_reobserve_tolerance});
  if (std::abs(layers.overhead_height[lhs_cell] -
               layers.overhead_height[rhs_cell]) > overhead_tolerance) {
    return false;
  }

  const bool lhs_has_support = std::isfinite(layers.support_height[lhs_cell]);
  const bool rhs_has_support = std::isfinite(layers.support_height[rhs_cell]);
  if (lhs_has_support != rhs_has_support) {
    return false;
  }
  if (!lhs_has_support) {
    return true;
  }

  const float support_tolerance =
      std::max({config.map.resolution, config.geometry.max_step_down,
                config.geometry.support_anchor_reobserve_tolerance});
  return std::abs(layers.support_height[lhs_cell] -
                  layers.support_height[rhs_cell]) <= support_tolerance;
}

std::vector<uint8_t> BuildObstaclePublishableMask(
    const FrontendOutput &frontend_output, const std::vector<uint8_t> &previous,
    const TerrainLayers &layers, const LocalTerrainMap &map,
    const Config &config) {
  std::vector<uint8_t> confirmed_facade_candidate(
      static_cast<size_t>(map.size()), 0U);
  std::vector<uint8_t> obstacle_candidate_cell(static_cast<size_t>(map.size()),
                                               0U);
  for (const auto &candidate : frontend_output.obstacle_candidates) {
    if (candidate.cell < 0 || candidate.cell >= map.size()) {
      continue;
    }
    obstacle_candidate_cell[static_cast<size_t>(candidate.cell)] = 1U;
    if (candidate.semantic == ObstacleCandidateSemantic::kConfirmedFacade) {
      confirmed_facade_candidate[static_cast<size_t>(candidate.cell)] = 1U;
    }
  }
  if (frontend_output.obstacle_candidate_cell.size() ==
      static_cast<size_t>(map.size())) {
    obstacle_candidate_cell = frontend_output.obstacle_candidate_cell;
  }

  std::vector<uint8_t> publishable(static_cast<size_t>(map.size()), 0U);
  std::deque<int> queue;
  queue.clear();

  for (int cell = 0; cell < map.size(); ++cell) {
    if (!IsStrongObstacleCell(cell, layers, config) ||
        !IsCellHeightPublishable(cell, layers, config) ||
        obstacle_candidate_cell[static_cast<size_t>(cell)] == 0U) {
      continue;
    }
    const bool structural_seed =
        confirmed_facade_candidate[static_cast<size_t>(cell)] != 0U ||
        (frontend_output.facade_upper_edge_aligned_with_supported_neighbors
                 .size() == static_cast<size_t>(map.size()) &&
         frontend_output.facade_upper_edge_aligned_with_supported_neighbors
                 [static_cast<size_t>(cell)] != 0U) ||
        previous[static_cast<size_t>(cell)] != 0U;
    if (!structural_seed) {
      continue;
    }
    publishable[static_cast<size_t>(cell)] = 1U;
    queue.push_back(cell);
  }

  while (!queue.empty()) {
    const int cell = queue.front();
    queue.pop_front();
    const int row = cell / map.cols();
    const int col = cell % map.cols();
    for (int row_offset = -1; row_offset <= 1; ++row_offset) {
      for (int col_offset = -1; col_offset <= 1; ++col_offset) {
        if (row_offset == 0 && col_offset == 0) {
          continue;
        }
        const int neighbor_row = row + row_offset;
        const int neighbor_col = col + col_offset;
        if (neighbor_row < 0 || neighbor_row >= map.rows() || neighbor_col < 0 ||
            neighbor_col >= map.cols()) {
          continue;
        }
        const int neighbor = neighbor_row * map.cols() + neighbor_col;
        if (publishable[static_cast<size_t>(neighbor)] != 0U ||
            !IsStrongObstacleCell(neighbor, layers, config) ||
            !IsCellHeightPublishable(neighbor, layers, config) ||
            !AreCellsStructurallyConnected(cell, neighbor, layers, config)) {
          continue;
        }

        const bool current_candidate =
            obstacle_candidate_cell[static_cast<size_t>(neighbor)] != 0U;
        const bool previously_publishable =
            previous[static_cast<size_t>(neighbor)] != 0U;
        if (!current_candidate && !previously_publishable) {
          continue;
        }

        publishable[static_cast<size_t>(neighbor)] = 1U;
        queue.push_back(neighbor);
      }
    }
  }

  return publishable;
}

} // namespace

std::vector<int>
DropoutAwareMapUpdater::update(const FrontendOutput &frontend_output,
                               const FrameObservability &observability,
                               LocalTerrainMap &map) const {
  map.ageCells();
  auto &layers = map.layers();
  const std::vector<uint8_t> previous_publishable = layers.obstacle_publishable;
  std::unordered_set<int> dirty_set;
  dirty_set.reserve(frontend_output.support_candidates.size() +
                    frontend_output.obstacle_candidates.size());
  std::vector<uint8_t> touched_support(map.size(), 0);
  std::vector<uint8_t> touched_obstacle(map.size(), 0);

  const auto sector_state_for_cell = [&](int cell) {
    const auto cell_center = map.indexToMap(cell);
    const float angle = std::atan2(cell_center.y() - map.center().y(),
                                   cell_center.x() - map.center().x());
    const float normalized = angle + static_cast<float>(M_PI);
    const int sector = std::clamp(
        static_cast<int>(std::floor(
            normalized / (2.0f * static_cast<float>(M_PI) /
                          static_cast<float>(observability.sectors.size())))),
        0, static_cast<int>(observability.sectors.size()) - 1);
    return observability.sectors[sector];
  };

  for (const auto &candidate : frontend_output.support_candidates) {
    const auto sector = sector_state_for_cell(candidate.cell);
    layers.support_height[candidate.cell] = candidate.z;
    layers.support_confidence[candidate.cell] = std::clamp(
        layers.support_confidence[candidate.cell] +
            config_.persistence.support_confidence_gain * candidate.confidence,
        0.0f, 1.0f);
    layers.coverage_confidence[candidate.cell] = std::max(
        layers.coverage_confidence[candidate.cell], sector.coverage_confidence);
    layers.support_state[candidate.cell] =
        static_cast<uint8_t>(SupportState::kObserved);
    layers.last_observed_age[candidate.cell] = 0;
    if (sector.state == ObservabilityState::kObserved) {
      layers.last_reliable_age[candidate.cell] = 0;
    }
    layers.last_sector_state[candidate.cell] =
        static_cast<uint8_t>(sector.state);
    touched_support[candidate.cell] = 1U;
    dirty_set.insert(candidate.cell);
  }

  for (const auto &candidate : frontend_output.obstacle_candidates) {
    const auto sector = sector_state_for_cell(candidate.cell);
    const float evidence_gain =
        config_.persistence.obstacle_evidence_gain *
        ObstacleEvidenceGainScaleForSemantic(candidate, config_);
    const float candidate_evidence =
        ObstacleEvidenceForSemantic(candidate, config_);
    layers.overhead_height[candidate.cell] = candidate.z;
    layers.overhead_confidence[candidate.cell] = std::clamp(
        layers.overhead_confidence[candidate.cell] + evidence_gain, 0.0f, 1.0f);
    layers.obstacle_evidence[candidate.cell] =
        std::clamp(layers.obstacle_evidence[candidate.cell] +
                       evidence_gain * candidate_evidence,
                   0.0f, 1.0f);
    layers.coverage_confidence[candidate.cell] = std::max(
        layers.coverage_confidence[candidate.cell], sector.coverage_confidence);
    layers.last_sector_state[candidate.cell] =
        static_cast<uint8_t>(sector.state);
    touched_obstacle[candidate.cell] = 1U;
    dirty_set.insert(candidate.cell);
  }

  for (int cell = 0; cell < map.size(); ++cell) {
    const auto sector = sector_state_for_cell(cell);
    layers.coverage_confidence[cell] =
        std::max(layers.coverage_confidence[cell] * 0.92f,
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
      const bool support_reobserved = touched_support[cell] != 0U;
      if (support_reobserved && sector.state == ObservabilityState::kObserved) {
        obstacle_decay = config_.persistence.obstacle_clear_observed_decay;
      } else if (support_reobserved &&
                 sector.state == ObservabilityState::kPartiallyObserved) {
        obstacle_decay = config_.persistence.obstacle_clear_observed_decay *
                         config_.persistence.obstacle_clear_partial_decay_scale;
      } else if (sector.state == ObservabilityState::kObserved) {
        obstacle_decay = config_.persistence.obstacle_evidence_decay * 0.4f;
      } else if (sector.state == ObservabilityState::kPartiallyObserved) {
        obstacle_decay = config_.persistence.obstacle_evidence_decay * 0.1f;
      } else if (sector.state == ObservabilityState::kMissingByDropout) {
        obstacle_decay = config_.persistence.obstacle_evidence_decay * 0.03f;
      }
      layers.obstacle_evidence[cell] =
          std::max(0.0f, layers.obstacle_evidence[cell] - obstacle_decay);
      if (obstacle_decay > 0.0f) {
        layers.overhead_confidence[cell] =
            std::max(0.0f, layers.overhead_confidence[cell] - obstacle_decay);
      }
    }

    if (touched_support[cell]) {
      if (!touched_obstacle[cell] &&
          layers.obstacle_evidence[cell] <=
              config_.persistence.obstacle_height_clear_threshold) {
        ClearObstacleLayer(layers, cell);
      }
      continue;
    }

    const bool persistent_allowed =
        layers.last_reliable_age[cell] <=
        static_cast<uint16_t>(
            std::max(1, config_.persistence.support_persistence_frames));
    if (layers.support_confidence[cell] >=
            config_.observability.min_support_confidence &&
        persistent_allowed) {
      layers.support_state[cell] =
          static_cast<uint8_t>(SupportState::kPersistent);
    } else if (layers.support_confidence[cell] <=
                   config_.observability.min_support_confidence * 0.5f ||
               !persistent_allowed) {
      layers.support_state[cell] = static_cast<uint8_t>(SupportState::kNone);
      layers.support_height[cell] = std::numeric_limits<float>::quiet_NaN();
      if (layers.obstacle_evidence[cell] <=
          config_.persistence.obstacle_height_clear_threshold) {
        ClearObstacleLayer(layers, cell);
      }
    }
  }

  layers.obstacle_publishable = BuildObstaclePublishableMask(
      frontend_output, previous_publishable, layers, map, config_);

  std::vector<int> dirty_cells(dirty_set.begin(), dirty_set.end());
  return dirty_cells;
}

} // namespace passable_area::core
