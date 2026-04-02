#include "passable_area/core/observability/frame_observability_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace passable_area::core {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float NormalizeAngle(float angle) {
  while (angle > kPi) {
    angle -= 2.0f * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0f * kPi;
  }
  return angle;
}

} // namespace

FrameObservability FrameObservabilityEstimator::estimate(const ProcessedFrame &frame) const {
  FrameObservability result;
  result.sectors.resize(config_.observability.sector_count);
  result.base_point_count = static_cast<uint32_t>(frame.cloud_in_base.size());
  result.odom_point_count = static_cast<uint32_t>(frame.cloud_in_odom.size());
  if (frame.cloud_in_base.empty()) {
    result.frame_partial = true;
    result.rear_dropout = true;
    for (auto &sector : result.sectors) {
      sector.state = ObservabilityState::kMissingByDropout;
    }
    return result;
  }

  std::vector<int> counts(config_.observability.sector_count, 0);
  const float angle_per_sector = 2.0f * kPi / static_cast<float>(config_.observability.sector_count);
  for (const auto &point : frame.cloud_in_base) {
    const float angle = std::atan2(point.y, point.x);
    const int sector = std::clamp(
        static_cast<int>(std::floor((NormalizeAngle(angle) + kPi) / angle_per_sector)), 0,
        config_.observability.sector_count - 1);
    ++counts[sector];
  }

  int front_points = 0;
  int rear_points = 0;
  int current_rear_gap = 0;
  int max_rear_gap = 0;
  std::vector<int> rear_gap_sectors;
  std::vector<int> current_gap_indices;

  for (int sector = 0; sector < config_.observability.sector_count; ++sector) {
    const float angle = -kPi + (static_cast<float>(sector) + 0.5f) * angle_per_sector;
    auto &state = result.sectors[sector];
    const bool rear = std::abs(angle) > kPi / 2.0f;
    const float coverage = std::min(
        1.0f, static_cast<float>(counts[sector]) /
                  static_cast<float>(std::max(config_.observability.min_points_per_sector, 1)));
    state.coverage_confidence = coverage;
    if (rear) {
      rear_points += counts[sector];
    } else {
      front_points += counts[sector];
    }
    if (counts[sector] == 0) {
      state.state = ObservabilityState::kPartiallyObserved;
      result.frame_partial = true;
      if (rear) {
        ++current_rear_gap;
        max_rear_gap = std::max(max_rear_gap, current_rear_gap);
        current_gap_indices.push_back(sector);
        if (current_rear_gap == max_rear_gap) {
          rear_gap_sectors = current_gap_indices;
        }
      }
      continue;
    }

    current_rear_gap = 0;
    current_gap_indices.clear();
    if (coverage < config_.observability.partial_sector_ratio) {
      state.state = ObservabilityState::kPartiallyObserved;
      result.frame_partial = true;
    } else {
      state.state = ObservabilityState::kObserved;
    }
  }

  const bool rear_starved = rear_points * 2 < std::max(front_points, 1);
  if (rear_starved && max_rear_gap >= config_.observability.dropout_sector_gap_threshold) {
    result.rear_dropout = true;
    result.frame_partial = true;
    for (const int sector : rear_gap_sectors) {
      result.sectors[sector].state = ObservabilityState::kMissingByDropout;
    }
  }
  return result;
}

} // namespace passable_area::core
