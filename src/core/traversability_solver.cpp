#include "passable_area/core/traversability_solver.hpp"

#include <algorithm>
#include <cmath>

namespace passable_area::core {
namespace {

constexpr float kUnknownCoverageThreshold = 0.15f;
constexpr float kBasePassableCost = 10.0f;
constexpr float kCostRange = 90.0f;
constexpr float kSlopeCostWeight = 0.40f;
constexpr float kStepCostWeight = 0.35f;
constexpr float kRoughnessCostWeight = 0.25f;

bool IsBlockingReason(BlockReason reason) {
  return reason == BlockReason::kProtrusion ||
         reason == BlockReason::kLowClearance ||
         reason == BlockReason::kGeometryFailure ||
         reason == BlockReason::kMixed;
}

} // namespace

void TraversabilitySolver::update(
    LocalTerrainMap &map, const ObstacleReasonerOutput &reasoner_output) const {
  auto &layers = map.layers();
  const float stale_threshold_frames =
      std::max(1.0f, config_.observability.stale_to_unknown_time /
                         std::max(map.framePeriodSec(), 1e-3f));
  for (int cell = 0; cell < map.size(); ++cell) {
    const float coverage = layers.coverage_confidence[cell];
    const float support_confidence = layers.support_confidence[cell];
    const float slope = layers.slope[cell];
    const float step_up = layers.step_up[cell];
    const float step_down = layers.step_down[cell];
    const float roughness = layers.roughness[cell];
    const BlockReason block_reason =
        static_cast<size_t>(cell) < reasoner_output.block_reason.size()
            ? static_cast<BlockReason>(
                  reasoner_output.block_reason[static_cast<size_t>(cell)])
            : BlockReason::kNone;
    const auto support_state =
        static_cast<SupportState>(layers.support_state[cell]);
    const bool stale = static_cast<float>(layers.last_reliable_age[cell]) >
                       stale_threshold_frames;

    PassabilityState state = PassabilityState::kUnknown;
    if (coverage < kUnknownCoverageThreshold ||
        support_state == SupportState::kNone ||
        support_confidence < config_.observability.min_support_confidence ||
        stale) {
      state = PassabilityState::kUnknown;
    } else if (IsBlockingReason(block_reason)) {
      state = PassabilityState::kImpassable;
    } else {
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
    const float cost = kBasePassableCost +
                       kCostRange * (kSlopeCostWeight * slope_ratio +
                                     kStepCostWeight * step_ratio +
                                     kRoughnessCostWeight * rough_ratio);
    layers.traversal_cost[cell] =
        static_cast<int8_t>(std::clamp(cost, 1.0f, 99.0f));
  }
}

} // namespace passable_area::core
