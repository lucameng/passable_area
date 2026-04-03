#include "passable_area/core/frontend/polar_frontend.hpp"
#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
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
  output.upper_support_cell.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_suspicious.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_rejected_by_neighbor_support.assign(static_cast<size_t>(map.size()), 0U);
  output.neighbor_upper_support_count.assign(static_cast<size_t>(map.size()), 0);
  std::unordered_map<int, CellStats> stats_by_cell;
  stats_by_cell.reserve(frame.odom_samples.size() / 4U + 1U);

  const float sector_size =
      2.0f * static_cast<float>(M_PI) / static_cast<float>(observability.sectors.size());
  const float yaw = YawFromQuaternion(frame.base_pose_in_odom.orientation);

  for (const auto &sample : frame.odom_samples) {
    int cell = -1;
    if (!map.odomToIndex(sample.point_in_odom.x, sample.point_in_odom.y, cell)) {
      continue;
    }
    auto &stats = stats_by_cell[cell];
    stats.min_z = std::min(stats.min_z, sample.point_in_odom.z);
    stats.max_z = std::max(stats.max_z, sample.point_in_odom.z);
    ++stats.count;
  }

  output.support_candidates.reserve(stats_by_cell.size());
  output.obstacle_candidates.reserve(stats_by_cell.size());
  output.ambiguous_candidates.reserve(stats_by_cell.size());

  std::unordered_set<int> suspicious_cells;
  suspicious_cells.reserve(stats_by_cell.size());
  const auto &layers = map.layers();
  const float upper_height_threshold = config_.geometry.upper_min_height_above_support;
  const float suspicious_vertical_span = config_.geometry.max_step_up * 0.75f;
  std::unordered_map<int, float> support_ref_by_cell;
  support_ref_by_cell.reserve(stats_by_cell.size());

  for (auto &[cell, stats] : stats_by_cell) {
    if (stats.count == 0) {
      continue;
    }
    const Eigen::Vector2f center = map.indexToOdom(cell);
    // Use the cell-center body angle as the stable sector representative for this cell.
    const float representative_base_angle =
        NormalizeAngle(std::atan2(center.y() - frame.base_pose_in_odom.position.y(),
                                  center.x() - frame.base_pose_in_odom.position.x()) -
                       yaw);
    const int sector = std::clamp(
        static_cast<int>(
            std::floor((representative_base_angle + static_cast<float>(M_PI)) / sector_size)),
        0,
        static_cast<int>(observability.sectors.size()) - 1);
    const auto sector_state = observability.sectors[sector].state;
    const float vertical_span = stats.max_z - stats.min_z;
    const float coverage = observability.sectors[sector].coverage_confidence;
    const float historical_support = layers.support_height[static_cast<size_t>(cell)];
    const float support_ref =
        std::isfinite(historical_support) ? historical_support : stats.min_z;
    support_ref_by_cell.emplace(cell, support_ref);

    if (sector_state != ObservabilityState::kMissingByDropout) {
      output.support_candidates.push_back(
          SupportCandidate{cell, stats.min_z, std::clamp(coverage, 0.0f, 1.0f)});
    }
    if (vertical_span > suspicious_vertical_span) {
      output.obstacle_suspicious[static_cast<size_t>(cell)] = 1U;
      suspicious_cells.insert(cell);
    } else if (sector_state == ObservabilityState::kPartiallyObserved) {
      output.ambiguous_candidates.push_back(AmbiguousCandidate{cell, stats.min_z});
    }
  }

  for (const auto &sample : frame.odom_samples) {
    int cell = -1;
    if (!map.odomToIndex(sample.point_in_odom.x, sample.point_in_odom.y, cell)) {
      continue;
    }
    if (output.upper_support_cell[static_cast<size_t>(cell)] != 0U) {
      continue;
    }
    const auto support_ref_it = support_ref_by_cell.find(cell);
    if (support_ref_it == support_ref_by_cell.end() || !std::isfinite(support_ref_it->second)) {
      continue;
    }
    if (sample.point_in_odom.z >= support_ref_it->second + upper_height_threshold) {
      output.upper_support_cell[static_cast<size_t>(cell)] = 1U;
    }
  }

  const int min_neighbor_upper_support_cells =
      std::max(1, config_.geometry.min_neighbor_upper_support_cells);
  for (const int cell : suspicious_cells) {
    const int row = cell / map.cols();
    const int col = cell % map.cols();
    int support_count = 0;
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        const int nr = row + dr;
        const int nc = col + dc;
        if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
          continue;
        }
        const int neighbor = nr * map.cols() + nc;
        support_count += output.upper_support_cell[static_cast<size_t>(neighbor)] != 0U ? 1 : 0;
      }
    }
    output.neighbor_upper_support_count[static_cast<size_t>(cell)] =
        static_cast<int8_t>(std::clamp(support_count, 0, 9));
    if (support_count >= min_neighbor_upper_support_cells) {
      const auto stats_it = stats_by_cell.find(cell);
      if (stats_it != stats_by_cell.end()) {
        const float vertical_span = stats_it->second.max_z - stats_it->second.min_z;
        output.obstacle_candidates.push_back(
            ObstacleCandidate{cell, stats_it->second.max_z, std::clamp(vertical_span, 0.0f, 1.0f)});
      }
    } else {
      output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(cell)] = 1U;
    }
  }
  return output;
}

} // namespace passable_area::core
