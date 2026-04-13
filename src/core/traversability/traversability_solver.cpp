#include "passable_area/core/traversability/traversability_solver.hpp"

#include <algorithm>
#include <cmath>

namespace passable_area::core {

void TraversabilitySolver::update(LocalTerrainMap &map) const {
  auto &layers = map.layers();
  const float stale_threshold_frames =
      std::max(1.0f, config_.observability.stale_to_unknown_time /
                         std::max(map.framePeriodSec(), 1e-3f));
  for (int cell = 0; cell < map.size(); ++cell) {
    const float coverage = layers.coverage_confidence[cell];
    const float support_confidence = layers.support_confidence[cell];
    const float clearance = layers.clearance[cell];
    const float obstacle = layers.obstacle_evidence[cell];
    const float continuity = layers.support_continuity[cell];
    const float slope = layers.slope[cell];
    const float step_up = layers.step_up[cell];
    const float step_down = layers.step_down[cell];
    const float roughness = layers.roughness[cell];
    const auto support_state =
        static_cast<SupportState>(layers.support_state[cell]);
    const bool stale = static_cast<float>(layers.last_reliable_age[cell]) >
                       stale_threshold_frames;

    PassabilityState state = PassabilityState::kUnknown;
    if (coverage < 0.15f || support_state == SupportState::kNone ||
        support_confidence < config_.observability.min_support_confidence ||
        stale) {
      state = PassabilityState::kUnknown;
    } else if (std::isfinite(clearance) &&
               clearance < config_.geometry.min_clearance) {
      state = PassabilityState::kImpassable;
    } else if (continuity < 0.3f && obstacle > 0.4f) {
      state = PassabilityState::kImpassable;
    } else if (slope <= config_.geometry.max_support_slope_deg &&
               step_up <= config_.geometry.max_step_up &&
               step_down <= config_.geometry.max_step_down &&
               roughness <= config_.geometry.max_support_roughness &&
               (!std::isfinite(clearance) ||
                clearance >= config_.geometry.min_clearance)) {
      state = PassabilityState::kPassable;
    }

    layers.passability_state[cell] = static_cast<int8_t>(state);
    if (state == PassabilityState::kUnknown) {
      layers.traversal_cost[cell] = -1;
      continue;
    }
    if (state == PassabilityState::kImpassable) {
      layers.traversal_cost[cell] = 100;
      continue;
    }

    const float slope_ratio = std::clamp(
        slope / std::max(config_.geometry.max_support_slope_deg, 1.0f), 0.0f,
        1.0f);
    const float step_ratio = std::clamp(
        std::max(step_up / std::max(config_.geometry.max_step_up, 1e-3f),
                 step_down / std::max(config_.geometry.max_step_down, 1e-3f)),
        0.0f, 1.0f);
    const float rough_ratio = std::clamp(
        roughness / std::max(config_.geometry.max_support_roughness, 1e-3f),
        0.0f, 1.0f);
    const float cost =
        10.0f +
        90.0f * (0.4f * slope_ratio + 0.35f * step_ratio + 0.25f * rough_ratio);
    layers.traversal_cost[cell] =
        static_cast<int8_t>(std::clamp(cost, 1.0f, 99.0f));
  }
}

} // namespace passable_area::core
