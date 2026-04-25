#include "passable_area/core/mapping/dropout_aware_map_updater.hpp"
#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace passable_area::core {
namespace {

void ClearObstacleLayer(TerrainLayers &layers, int cell) {
  layers.overhead_height[cell] = std::numeric_limits<float>::quiet_NaN();
  layers.overhead_confidence[cell] = 0.0f;
  layers.protrusion_height[cell] = std::numeric_limits<float>::quiet_NaN();
  layers.protrusion_evidence[cell] = 0.0f;
  layers.overhead_evidence[cell] = 0.0f;
}

float NormalizeAngle(float angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace

std::vector<int>
DropoutAwareMapUpdater::update(const FrontendOutput &frontend_output,
                               const FrameObservability &observability,
                               const Pose3D &base_pose_in_map,
                               LocalTerrainMap &map) const {
  map.ageCells();
  auto &layers = map.layers();
  std::unordered_set<int> dirty_set;
  dirty_set.reserve(frontend_output.support_candidates.size() +
                    frontend_output.protrusion_candidates.size() +
                    frontend_output.overhead_candidates.size());
  std::vector<uint8_t> touched_support(map.size(), 0);
  std::vector<uint8_t> touched_obstacle(map.size(), 0);

  const float base_yaw = YawFromQuaternion(base_pose_in_map.orientation);
  const auto sector_state_for_cell = [&](int cell) {
    if (observability.sectors.empty()) {
      return SectorObservability{};
    }
    const auto cell_center = map.indexToMap(cell);
    const float map_angle =
        std::atan2(cell_center.y() - base_pose_in_map.position.y(),
                   cell_center.x() - base_pose_in_map.position.x());
    const float base_angle = NormalizeAngle(map_angle - base_yaw);
    const float normalized = base_angle + static_cast<float>(M_PI);
    const int sector = std::clamp(
        static_cast<int>(std::floor(
            normalized / (2.0f * static_cast<float>(M_PI) /
                          static_cast<float>(observability.sectors.size())))),
        0, static_cast<int>(observability.sectors.size()) - 1);
    return observability.sectors[sector];
  };

  for (const auto &candidate : frontend_output.support_candidates) {
    const auto sector = sector_state_for_cell(candidate.cell);
    layers.coverage_confidence[candidate.cell] = std::max(
        layers.coverage_confidence[candidate.cell], sector.coverage_confidence);
    layers.last_sector_state[candidate.cell] =
        static_cast<uint8_t>(sector.state);

    const bool current_support_trusted =
        std::isfinite(layers.support_height[candidate.cell]) &&
        layers.support_confidence[candidate.cell] >=
            config_.observability.min_support_confidence;
    const bool support_band_independently_observed =
        candidate.support_sample_count >=
            config_.observability.min_points_per_sector &&
        candidate.support_sample_count > candidate.obstacle_sample_count;
    const bool obstacle_overlap_lifts_support =
        candidate.obstacle_overlap && current_support_trusted &&
        candidate.z > layers.support_height[candidate.cell] &&
        !support_band_independently_observed;
    if (obstacle_overlap_lifts_support) {
      layers.support_state[candidate.cell] =
          static_cast<uint8_t>(SupportState::kPersistent);
      if (sector.state == ObservabilityState::kObserved) {
        layers.last_reliable_age[candidate.cell] = 0;
      }
      touched_support[candidate.cell] = 1U;
    } else {
      layers.support_height[candidate.cell] = candidate.z;
      layers.support_confidence[candidate.cell] =
          std::clamp(layers.support_confidence[candidate.cell] +
                         config_.persistence.support_confidence_gain *
                             candidate.confidence,
                     0.0f, 1.0f);
      layers.support_state[candidate.cell] =
          static_cast<uint8_t>(SupportState::kObserved);
      layers.last_observed_age[candidate.cell] = 0;
      if (sector.state == ObservabilityState::kObserved) {
        layers.last_reliable_age[candidate.cell] = 0;
      }
      touched_support[candidate.cell] = 1U;
    }
    dirty_set.insert(candidate.cell);
  }

  for (const auto &candidate : frontend_output.protrusion_candidates) {
    const auto sector = sector_state_for_cell(candidate.cell);
    const float evidence_gain = config_.persistence.obstacle_evidence_gain *
                                std::max(1.0f, candidate.gain_scale);
    layers.protrusion_height[candidate.cell] = candidate.z;
    layers.protrusion_evidence[candidate.cell] =
        std::clamp(layers.protrusion_evidence[candidate.cell] +
                       evidence_gain * candidate.evidence,
                   0.0f, 1.0f);
    layers.obstacle_evidence[candidate.cell] =
        std::max(layers.protrusion_evidence[candidate.cell],
                 layers.overhead_evidence[candidate.cell]);
    layers.coverage_confidence[candidate.cell] = std::max(
        layers.coverage_confidence[candidate.cell], sector.coverage_confidence);
    layers.last_sector_state[candidate.cell] =
        static_cast<uint8_t>(sector.state);
    touched_obstacle[candidate.cell] = 1U;
    dirty_set.insert(candidate.cell);
  }

  for (const auto &candidate : frontend_output.overhead_candidates) {
    const auto sector = sector_state_for_cell(candidate.cell);
    const float evidence_gain = config_.persistence.obstacle_evidence_gain *
                                std::max(1.0f, candidate.gain_scale);
    layers.overhead_height[candidate.cell] = candidate.z;
    layers.overhead_confidence[candidate.cell] = std::clamp(
        layers.overhead_confidence[candidate.cell] + evidence_gain, 0.0f, 1.0f);
    layers.overhead_evidence[candidate.cell] =
        std::clamp(layers.overhead_evidence[candidate.cell] +
                       evidence_gain * candidate.evidence,
                   0.0f, 1.0f);
    layers.obstacle_evidence[candidate.cell] =
        std::max(layers.protrusion_evidence[candidate.cell],
                 layers.overhead_evidence[candidate.cell]);
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
      } else if (sector.state == ObservabilityState::kObserved) {
        obstacle_decay = config_.persistence.obstacle_evidence_decay * 0.4f;
      } else if (sector.state == ObservabilityState::kMissingByDropout) {
        obstacle_decay = config_.persistence.obstacle_evidence_decay * 0.03f;
      }
      if (obstacle_decay > 0.0f) {
        layers.overhead_confidence[cell] =
            std::max(0.0f, layers.overhead_confidence[cell] - obstacle_decay);
        layers.protrusion_evidence[cell] =
            std::max(0.0f, layers.protrusion_evidence[cell] - obstacle_decay);
        layers.overhead_evidence[cell] =
            std::max(0.0f, layers.overhead_evidence[cell] - obstacle_decay);
      }
      layers.obstacle_evidence[cell] = std::max(
          layers.protrusion_evidence[cell], layers.overhead_evidence[cell]);
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

  std::vector<int> dirty_cells(dirty_set.begin(), dirty_set.end());
  return dirty_cells;
}

} // namespace passable_area::core
