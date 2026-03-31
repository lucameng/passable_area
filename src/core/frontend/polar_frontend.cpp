#include "passable_area/core/frontend/polar_frontend.hpp"

#include <Eigen/Geometry>

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

} // namespace

FrontendOutput PolarFrontend::run(const FrameInput &frame,
                                  const FrameObservability &observability,
                                  const LocalTerrainMap &map) const {
  FrontendOutput output;
  std::unordered_map<int, CellStats> stats_by_cell;
  stats_by_cell.reserve(frame.merged_cloud.size() / 4U + 1U);

  const float sector_size =
      2.0f * static_cast<float>(M_PI) / static_cast<float>(observability.sectors.size());
  const Eigen::Quaternionf inv_orientation = frame.base_pose_in_local.orientation.conjugate().normalized();

  for (const auto &point : frame.merged_cloud) {
    int cell = -1;
    if (!map.worldToIndex(point.x, point.y, cell)) {
      continue;
    }
    const Eigen::Vector3f local_point(point.x, point.y, point.z);
    const Eigen::Vector3f base_point =
        inv_orientation * (local_point - frame.base_pose_in_local.position);
    const float angle = std::atan2(base_point.y(), base_point.x());
    const int sector = std::clamp(
        static_cast<int>(std::floor((angle + static_cast<float>(M_PI)) / sector_size)), 0,
        static_cast<int>(observability.sectors.size()) - 1);
    const auto sector_state = observability.sectors[sector].state;
    if (sector_state == ObservabilityState::kBlindByStructure) {
      continue;
    }
    auto &stats = stats_by_cell[cell];
    stats.min_z = std::min(stats.min_z, point.z);
    stats.max_z = std::max(stats.max_z, point.z);
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
    const Eigen::Vector3f base_point(center.x(), center.y(), 0.0f);
    const Eigen::Vector3f relative = inv_orientation * (base_point - frame.base_pose_in_local.position);
    const float angle = std::atan2(relative.y(), relative.x());
    const int sector = std::clamp(
        static_cast<int>(std::floor((angle + static_cast<float>(M_PI)) / sector_size)), 0,
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
