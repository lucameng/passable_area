#include "passable_area/core/frame_observability_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace passable_area::core {
namespace {

constexpr float kPi = 3.14159265358979323846f;

struct EmptyGap {
  int length = 0;
  std::vector<int> sectors;
};

float NormalizeAngle(float angle) {
  while (angle > kPi) {
    angle -= 2.0f * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0f * kPi;
  }
  return angle;
}

bool IsFrontAngle(float angle) { return std::abs(angle) <= kPi / 2.0f; }

std::vector<int> OrderedHemisphereSectors(int sector_count, bool front) {
  std::vector<int> front_sectors;
  std::vector<int> rear_positive_sectors;
  std::vector<int> rear_negative_sectors;
  front_sectors.reserve(static_cast<size_t>(sector_count));
  rear_positive_sectors.reserve(static_cast<size_t>(sector_count));
  rear_negative_sectors.reserve(static_cast<size_t>(sector_count));

  const float angle_per_sector =
      2.0f * kPi / static_cast<float>(sector_count);
  for (int sector = 0; sector < sector_count; ++sector) {
    const float angle =
        -kPi + (static_cast<float>(sector) + 0.5f) * angle_per_sector;
    if (IsFrontAngle(angle)) {
      front_sectors.push_back(sector);
    } else if (angle > 0.0f) {
      rear_positive_sectors.push_back(sector);
    } else {
      rear_negative_sectors.push_back(sector);
    }
  }

  if (front) {
    return front_sectors;
  }

  std::vector<int> rear_sectors;
  rear_sectors.reserve(rear_positive_sectors.size() +
                       rear_negative_sectors.size());
  rear_sectors.insert(rear_sectors.end(), rear_positive_sectors.begin(),
                      rear_positive_sectors.end());
  rear_sectors.insert(rear_sectors.end(), rear_negative_sectors.begin(),
                      rear_negative_sectors.end());
  return rear_sectors;
}

EmptyGap LongestEmptyGap(const std::vector<int> &ordered_sectors,
                         const std::vector<int> &counts) {
  EmptyGap best;
  EmptyGap current;
  for (const int sector : ordered_sectors) {
    if (counts[sector] == 0) {
      ++current.length;
      current.sectors.push_back(sector);
      if (current.length > best.length) {
        best = current;
      }
      continue;
    }
    current = EmptyGap{};
  }
  return best;
}

} // namespace

FrameObservability
FrameObservabilityEstimator::estimate(const ProcessedFrame &frame) const {
  FrameObservability result;
  result.sectors.resize(config_.observability.sector_count);
  result.base_point_count = static_cast<uint32_t>(frame.cloud_in_base.size());
  result.map_point_count = static_cast<uint32_t>(frame.cloud_in_map.size());
  if (frame.cloud_in_base.empty()) {
    result.frame_partial = true;
    result.front_dropout = true;
    result.rear_dropout = true;
    for (auto &sector : result.sectors) {
      sector.state = ObservabilityState::kMissingByDropout;
    }
    return result;
  }

  std::vector<int> counts(config_.observability.sector_count, 0);
  const float angle_per_sector =
      2.0f * kPi / static_cast<float>(config_.observability.sector_count);
  for (const auto &point : frame.cloud_in_base) {
    const float angle = std::atan2(point.y, point.x);
    const int sector =
        std::clamp(static_cast<int>(std::floor((NormalizeAngle(angle) + kPi) /
                                               angle_per_sector)),
                   0, config_.observability.sector_count - 1);
    ++counts[sector];
  }

  int front_points = 0;
  int rear_points = 0;

  for (int sector = 0; sector < config_.observability.sector_count; ++sector) {
    const float angle =
        -kPi + (static_cast<float>(sector) + 0.5f) * angle_per_sector;
    auto &state = result.sectors[sector];
    const bool front = IsFrontAngle(angle);
    const float coverage = std::min(
        1.0f, static_cast<float>(counts[sector]) /
                  static_cast<float>(std::max(
                      config_.observability.min_points_per_sector, 1)));
    state.coverage_confidence = coverage;
    if (front) {
      front_points += counts[sector];
    } else {
      rear_points += counts[sector];
    }
    if (counts[sector] == 0) {
      state.state = ObservabilityState::kObserved;
      result.frame_partial = true;
      continue;
    }

    if (coverage < config_.observability.partial_sector_ratio) {
      state.state = ObservabilityState::kObserved;
      result.frame_partial = true;
    } else {
      state.state = ObservabilityState::kObserved;
    }
  }

  const EmptyGap front_gap =
      LongestEmptyGap(OrderedHemisphereSectors(
                          config_.observability.sector_count, true),
                      counts);
  const EmptyGap rear_gap =
      LongestEmptyGap(OrderedHemisphereSectors(
                          config_.observability.sector_count, false),
                      counts);
  const bool front_starved = front_points * 2 < std::max(rear_points, 1);
  const bool rear_starved = rear_points * 2 < std::max(front_points, 1);
  if (front_starved &&
      front_gap.length >= config_.observability.dropout_sector_gap_threshold) {
    result.front_dropout = true;
    result.frame_partial = true;
    for (const int sector : front_gap.sectors) {
      result.sectors[sector].state = ObservabilityState::kMissingByDropout;
    }
  }
  if (rear_starved &&
      rear_gap.length >= config_.observability.dropout_sector_gap_threshold) {
    result.rear_dropout = true;
    result.frame_partial = true;
    for (const int sector : rear_gap.sectors) {
      result.sectors[sector].state = ObservabilityState::kMissingByDropout;
    }
  }
  return result;
}

} // namespace passable_area::core
