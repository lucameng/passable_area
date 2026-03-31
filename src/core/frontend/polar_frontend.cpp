#include "passable_area/core/frontend/polar_frontend.hpp"
#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace passable_area::core {
namespace {

struct CellStats {
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  int count = 0;
};

float NormalizeAngle(float angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace

FrontendOutput PolarFrontend::run(const ProcessedFrame &frame,
                                  const FrameObservability &observability,
                                  const LocalTerrainMap &map) const {
  FrontendOutput output;
  std::unordered_map<int, CellStats> stats_by_cell;
  stats_by_cell.reserve(frame.gravity_samples.size() / 4U + 1U);

  const float sector_size =
      2.0f * static_cast<float>(M_PI) / static_cast<float>(observability.sectors.size());
  const float yaw = YawFromQuaternion(frame.base_pose_in_local.orientation);

  for (const auto &sample : frame.gravity_samples) {
    int cell = -1;
    if (!map.worldToIndex(sample.point_in_gravity.x, sample.point_in_gravity.y, cell)) {
      continue;
    }
    const float sample_body_angle = std::atan2(sample.point_in_base.y, sample.point_in_base.x);
    const int sector = std::clamp(
        static_cast<int>(std::floor((sample_body_angle + static_cast<float>(M_PI)) / sector_size)), 0,
        static_cast<int>(observability.sectors.size()) - 1);
    if (observability.sectors[sector].state == ObservabilityState::kBlindByStructure) {
      continue;
    }
    auto &stats = stats_by_cell[cell];
    stats.min_z = std::min(stats.min_z, sample.point_in_gravity.z);
    stats.max_z = std::max(stats.max_z, sample.point_in_gravity.z);
    ++stats.count;
  }

  output.support_candidates.reserve(stats_by_cell.size());
  output.obstacle_candidates.reserve(stats_by_cell.size());
  output.ambiguous_candidates.reserve(stats_by_cell.size());

  for (const auto &[cell, stats] : stats_by_cell) {
    if (stats.count == 0) {
      continue;
    }
    const Eigen::Vector2f center = map.indexToWorld(cell);
    // Use the cell-center body angle as the stable sector representative for this cell.
    const float representative_base_angle =
        NormalizeAngle(std::atan2(center.y() - frame.base_pose_in_local.position.y(),
                                  center.x() - frame.base_pose_in_local.position.x()) -
                       yaw);
    const int sector = std::clamp(
        static_cast<int>(
            std::floor((representative_base_angle + static_cast<float>(M_PI)) / sector_size)),
        0,
        static_cast<int>(observability.sectors.size()) - 1);
    const auto sector_state = observability.sectors[sector].state;
    const float vertical_span = stats.max_z - stats.min_z;
    const float coverage = observability.sectors[sector].coverage_confidence;

    if (sector_state != ObservabilityState::kMissingByDropout) {
      output.support_candidates.push_back(
          SupportCandidate{cell, stats.min_z, std::clamp(coverage, 0.0f, 1.0f)});
    }
    if (vertical_span > config_.geometry.max_step_up * 0.75f) {
      output.obstacle_candidates.push_back(
          ObstacleCandidate{cell, stats.max_z, std::clamp(vertical_span, 0.0f, 1.0f)});
    } else if (sector_state == ObservabilityState::kPartiallyObserved) {
      output.ambiguous_candidates.push_back(AmbiguousCandidate{cell, stats.min_z});
    }
  }
  return output;
}

} // namespace passable_area::core
