#include "passable_area/core/polar_frontend.hpp"
#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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

void AccumulateCellStats(CellStats &stats, float z) {
  stats.min_z = std::min(stats.min_z, z);
  stats.max_z = std::max(stats.max_z, z);
  ++stats.count;
}

int CountActiveCells(const std::vector<CellStats> &stats_by_cell) {
  return static_cast<int>(std::count_if(
      stats_by_cell.begin(), stats_by_cell.end(),
      [](const auto &stats) { return stats.count > 0; }));
}

} // namespace

FrontendOutput PolarFrontend::run(const ProcessedFrame &frame,
                                  const FrameObservability &observability,
                                  const LocalTerrainMap &map) const {
  FrontendOutput output;
  const size_t cell_count = static_cast<size_t>(map.size());
  const float nan = std::numeric_limits<float>::quiet_NaN();
  output.support_anchor_used.assign(cell_count, nan);
  output.support_anchor_origin.assign(cell_count,
                                      static_cast<uint8_t>(
                                          SupportAnchorOrigin::kNone));
  output.support_anchor_authority.assign(
      cell_count, static_cast<uint8_t>(SupportAnchorAuthority::kInvalid));
  output.anchor_leak_suppression_enabled.assign(cell_count, 0U);
  output.sub_support_leak_count.assign(cell_count, 0U);
  output.anchor_below_observation_count.assign(cell_count, 0U);
  output.stale_anchor_residual_filtered_count.assign(cell_count, 0U);
  output.raw_sample_min_z.assign(cell_count, nan);
  output.raw_sample_max_z.assign(cell_count, nan);
  output.raw_sample_count.assign(cell_count, 0U);
  output.filtered_sample_min_z.assign(cell_count, nan);
  output.filtered_sample_max_z.assign(cell_count, nan);
  output.filtered_sample_count.assign(cell_count, 0U);
  output.raw_upper_support_cell.assign(cell_count, 0U);
  output.explanation_adjusted_upper_support_cell.assign(cell_count, 0U);
  output.upper_support_cell.assign(cell_count, 0U);
  output.obstacle_local_triggered.assign(cell_count, 0U);
  output.obstacle_upper_patch_confirmed.assign(cell_count, 0U);
  output.obstacle_explanation_rejected.assign(cell_count, 0U);
  output.obstacle_suspicious.assign(cell_count, 0U);
  output.obstacle_candidate_cell.assign(cell_count, 0U);
  output.obstacle_rejected_by_neighbor_support.assign(cell_count, 0U);
  output.neighbor_upper_support_count.assign(cell_count, 0);
  output.aligned_neighbor_support_count.assign(cell_count, 0);
  output.explanation_decision.assign(
      cell_count,
      static_cast<uint8_t>(FrontendExplanationDecision::kNone));
  output.facade_lower_upper_coexisting.assign(cell_count, 0U);
  output.facade_upper_edge_aligned_with_supported_neighbors.assign(cell_count,
                                                                   0U);

  std::vector<CellStats> stats_by_cell(cell_count);
  for (const auto &sample : frame.map_samples) {
    int cell = -1;
    if (!map.mapToIndex(sample.point_in_map.x, sample.point_in_map.y, cell)) {
      continue;
    }
    AccumulateCellStats(stats_by_cell[static_cast<size_t>(cell)],
                        sample.point_in_map.z);
  }

  for (size_t cell = 0; cell < cell_count; ++cell) {
    const auto &stats = stats_by_cell[cell];
    if (stats.count == 0) {
      continue;
    }
    output.raw_sample_min_z[cell] = stats.min_z;
    output.raw_sample_max_z[cell] = stats.max_z;
    output.raw_sample_count[cell] = static_cast<uint16_t>(std::clamp(
        stats.count, 0, static_cast<int>(std::numeric_limits<uint16_t>::max())));
    output.filtered_sample_min_z[cell] = stats.min_z;
    output.filtered_sample_max_z[cell] = stats.max_z;
    output.filtered_sample_count[cell] = output.raw_sample_count[cell];
  }

  const int active_cell_count = CountActiveCells(stats_by_cell);
  output.support_candidates.reserve(static_cast<size_t>(active_cell_count));
  output.obstacle_candidates.reserve(static_cast<size_t>(active_cell_count));

  const float suspicious_vertical_span = config_.geometry.max_step_up * 0.75f;
  const float yaw = YawFromQuaternion(frame.base_pose_in_map.orientation);
  const size_t sector_count =
      std::max<size_t>(observability.sectors.size(), static_cast<size_t>(1));
  const float sector_size =
      2.0f * static_cast<float>(M_PI) / static_cast<float>(sector_count);

  for (int cell = 0; cell < map.size(); ++cell) {
    const auto &stats = stats_by_cell[static_cast<size_t>(cell)];
    if (stats.count == 0) {
      continue;
    }

    ObservabilityState sector_state = ObservabilityState::kObserved;
    float coverage = 0.0f;
    if (!observability.sectors.empty()) {
      const Eigen::Vector2f center = map.indexToMap(cell);
      const float representative_base_angle = NormalizeAngle(
          std::atan2(center.y() - frame.base_pose_in_map.position.y(),
                     center.x() - frame.base_pose_in_map.position.x()) -
          yaw);
      const int sector = std::clamp(
          static_cast<int>(std::floor((representative_base_angle +
                                       static_cast<float>(M_PI)) /
                                      sector_size)),
          0, static_cast<int>(observability.sectors.size()) - 1);
      sector_state = observability.sectors[static_cast<size_t>(sector)].state;
      coverage = observability.sectors[static_cast<size_t>(sector)]
                     .coverage_confidence;
    }

    if (sector_state != ObservabilityState::kMissingByDropout) {
      output.support_candidates.push_back(SupportCandidate{
          cell, stats.min_z, std::clamp(coverage, 0.0f, 1.0f)});
    }

    const float vertical_span = stats.max_z - stats.min_z;
    if (vertical_span > suspicious_vertical_span) {
      output.obstacle_local_triggered[static_cast<size_t>(cell)] = 1U;
      output.obstacle_upper_patch_confirmed[static_cast<size_t>(cell)] = 1U;
      output.obstacle_suspicious[static_cast<size_t>(cell)] = 1U;
      output.obstacle_candidate_cell[static_cast<size_t>(cell)] = 1U;
      output.obstacle_candidates.push_back(ObstacleCandidate{
          cell, stats.max_z, std::clamp(vertical_span, 0.0f, 1.0f), 1.0f});
    }
  }

  return output;
}

} // namespace passable_area::core
