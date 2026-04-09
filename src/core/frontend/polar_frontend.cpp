#include "passable_area/core/frontend/polar_frontend.hpp"
#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace passable_area::core {
namespace {

struct CellStats {
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  int count = 0;
};

float ComputeWallLikeObstacleGainScale(const CellStats &stats,
                                       float vertical_span,
                                       float relative_upper_z,
                                       int neighbor_upper_support_count) {
  if (stats.count < 10 || vertical_span < 0.5f || relative_upper_z < 0.0f) {
    return 1.0f;
  }

  const float count_bonus = std::clamp((static_cast<float>(stats.count) - 10.0f) * 0.02f,
                                       0.0f,
                                       0.55f);
  const float span_bonus = std::clamp((vertical_span - 0.5f) * 0.5f, 0.0f, 0.2f);
  const float neighbor_bonus = std::clamp(
      (static_cast<float>(neighbor_upper_support_count) - 2.0f) * 0.1f, 0.0f, 0.2f);
  return 1.0f + count_bonus + span_bonus + neighbor_bonus;
}

float NormalizeAngle(float angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool IsValidSupportAnchorCell(const TerrainLayers &layers, int cell, const Config &config) {
  if (cell < 0 || cell >= static_cast<int>(layers.support_height.size())) {
    return false;
  }
  if (!std::isfinite(layers.support_height[static_cast<size_t>(cell)])) {
    return false;
  }
  if (layers.support_confidence[static_cast<size_t>(cell)] <
      config.observability.min_support_confidence) {
    return false;
  }
  if (static_cast<SupportState>(layers.support_state[static_cast<size_t>(cell)]) ==
      SupportState::kNone) {
    return false;
  }
  return layers.last_reliable_age[static_cast<size_t>(cell)] <=
         static_cast<uint16_t>(std::max(1, config.persistence.support_persistence_frames));
}

float ResolveNeighborSupportAnchor(int cell, const LocalTerrainMap &map, const TerrainLayers &layers,
                                   const Config &config) {
  if (IsValidSupportAnchorCell(layers, cell, config)) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  const int row = cell / map.cols();
  const int col = cell % map.cols();
  std::vector<float> neighbor_heights;
  neighbor_heights.reserve(9);
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      const int nr = row + dr;
      const int nc = col + dc;
      if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
        continue;
      }
      const int neighbor = nr * map.cols() + nc;
      if (!IsValidSupportAnchorCell(layers, neighbor, config)) {
        continue;
      }
      neighbor_heights.push_back(layers.support_height[static_cast<size_t>(neighbor)]);
    }
  }
  if (neighbor_heights.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const auto middle = neighbor_heights.begin() + static_cast<std::ptrdiff_t>(neighbor_heights.size() / 2U);
  std::nth_element(neighbor_heights.begin(), middle, neighbor_heights.end());
  return *middle;
}

float ResolveNeighborSupportConsensusHeight(int cell,
                                            const LocalTerrainMap &map,
                                            const TerrainLayers &layers,
                                            const Config &config) {
  const int row = cell / map.cols();
  const int col = cell % map.cols();
  std::vector<float> neighbor_heights;
  neighbor_heights.reserve(8);
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0) {
        continue;
      }
      const int nr = row + dr;
      const int nc = col + dc;
      if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
        continue;
      }
      const int neighbor = nr * map.cols() + nc;
      if (!IsValidSupportAnchorCell(layers, neighbor, config)) {
        continue;
      }
      neighbor_heights.push_back(layers.support_height[static_cast<size_t>(neighbor)]);
    }
  }
  if (neighbor_heights.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const auto middle = neighbor_heights.begin() +
                      static_cast<std::ptrdiff_t>(neighbor_heights.size() / 2U);
  std::nth_element(neighbor_heights.begin(), middle, neighbor_heights.end());
  return *middle;
}

int CountNeighborAnchorsAlignedToHeight(int cell,
                                        const LocalTerrainMap &map,
                                        const TerrainLayers &layers,
                                        const Config &config,
                                        float target_z,
                                        float tolerance) {
  if (!std::isfinite(target_z)) {
    return 0;
  }

  const int row = cell / map.cols();
  const int col = cell % map.cols();
  int match_count = 0;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0) {
        continue;
      }
      const int nr = row + dr;
      const int nc = col + dc;
      if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
        continue;
      }
      const int neighbor = nr * map.cols() + nc;
      if (!IsValidSupportAnchorCell(layers, neighbor, config)) {
        continue;
      }
      if (std::abs(layers.support_height[static_cast<size_t>(neighbor)] - target_z) <= tolerance) {
        ++match_count;
      }
    }
  }
  return match_count;
}

int CountValidNeighborAnchors(int cell,
                              const LocalTerrainMap &map,
                              const TerrainLayers &layers,
                              const Config &config) {
  const int row = cell / map.cols();
  const int col = cell % map.cols();
  int count = 0;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0) {
        continue;
      }
      const int nr = row + dr;
      const int nc = col + dc;
      if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
        continue;
      }
      const int neighbor = nr * map.cols() + nc;
      if (IsValidSupportAnchorCell(layers, neighbor, config)) {
        ++count;
      }
    }
  }
  return count;
}

float ResolveNeighborUpperLayerConsensusHeight(int cell,
                                               const LocalTerrainMap &map,
                                               const std::vector<float> &support_anchor_by_cell,
                                               const std::vector<int> &upper_band_count_by_cell,
                                               const std::vector<float> &min_upper_band_z_by_cell,
                                               const std::vector<uint16_t> &sub_support_leak_count,
                                               float upper_height_threshold,
                                               float max_step_up) {
  const int row = cell / map.cols();
  const int col = cell % map.cols();
  std::vector<float> neighbor_heights;
  neighbor_heights.reserve(8);
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0) {
        continue;
      }
      const int nr = row + dr;
      const int nc = col + dc;
      if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
        continue;
      }
      const int neighbor = nr * map.cols() + nc;
      const float neighbor_support_anchor = support_anchor_by_cell[static_cast<size_t>(neighbor)];
      const float neighbor_min_upper_band_z = min_upper_band_z_by_cell[static_cast<size_t>(neighbor)];
      if (!std::isfinite(neighbor_support_anchor) || !std::isfinite(neighbor_min_upper_band_z) ||
          upper_band_count_by_cell[static_cast<size_t>(neighbor)] == 0 ||
          sub_support_leak_count[static_cast<size_t>(neighbor)] != 0U) {
        continue;
      }
      const float neighbor_upper_gap = neighbor_min_upper_band_z - neighbor_support_anchor;
      if (neighbor_upper_gap < upper_height_threshold || neighbor_upper_gap > max_step_up) {
        continue;
      }
      neighbor_heights.push_back(neighbor_min_upper_band_z);
    }
  }
  if (neighbor_heights.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const auto middle = neighbor_heights.begin() +
                      static_cast<std::ptrdiff_t>(neighbor_heights.size() / 2U);
  std::nth_element(neighbor_heights.begin(), middle, neighbor_heights.end());
  return *middle;
}

int CountNeighborUpperLayersAlignedToHeight(int cell,
                                            const LocalTerrainMap &map,
                                            const std::vector<float> &support_anchor_by_cell,
                                            const std::vector<int> &upper_band_count_by_cell,
                                            const std::vector<float> &min_upper_band_z_by_cell,
                                            const std::vector<uint16_t> &sub_support_leak_count,
                                            float target_z,
                                            float tolerance,
                                            float upper_height_threshold,
                                            float max_step_up) {
  if (!std::isfinite(target_z)) {
    return 0;
  }

  const int row = cell / map.cols();
  const int col = cell % map.cols();
  int match_count = 0;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0) {
        continue;
      }
      const int nr = row + dr;
      const int nc = col + dc;
      if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
        continue;
      }
      const int neighbor = nr * map.cols() + nc;
      const float neighbor_support_anchor = support_anchor_by_cell[static_cast<size_t>(neighbor)];
      const float neighbor_min_upper_band_z = min_upper_band_z_by_cell[static_cast<size_t>(neighbor)];
      if (!std::isfinite(neighbor_support_anchor) || !std::isfinite(neighbor_min_upper_band_z) ||
          upper_band_count_by_cell[static_cast<size_t>(neighbor)] == 0 ||
          sub_support_leak_count[static_cast<size_t>(neighbor)] != 0U) {
        continue;
      }
      const float neighbor_upper_gap = neighbor_min_upper_band_z - neighbor_support_anchor;
      if (neighbor_upper_gap < upper_height_threshold || neighbor_upper_gap > max_step_up) {
        continue;
      }
      if (std::abs(neighbor_min_upper_band_z - target_z) <= tolerance) {
        ++match_count;
      }
    }
  }
  return match_count;
}

bool ShouldRejectBelowRobotStairMix(float relative_support_ref,
                                    float relative_upper_z,
                                    int support_count,
                                    float max_step_down,
                                    float upper_height_threshold,
                                    uint16_t sub_support_leak_count,
                                    bool stale_lower_anchor_mix) {
  return std::isfinite(relative_support_ref) && relative_support_ref <= -max_step_down &&
         relative_upper_z <= -upper_height_threshold && support_count >= 3 &&
         (sub_support_leak_count > 0U || stale_lower_anchor_mix);
}

bool ShouldRejectBelowRobotGroundLayerMix(float relative_support_ref,
                                          float relative_upper_z,
                                          float vertical_span,
                                          int aligned_neighbor_support_count,
                                          float max_step_down,
                                          float upper_height_threshold,
                                          uint16_t sub_support_leak_count,
                                          bool stale_lower_anchor_mix) {
  // Downstairs ground-mix only trusts local anchored upper-support structure; dense upper bands alone do not
  // override a below-robot ground-layer interpretation.
  const bool has_reinforcing_support_structure = aligned_neighbor_support_count > 0;
  return std::isfinite(relative_support_ref) && relative_support_ref <= -max_step_down &&
         relative_upper_z <= -upper_height_threshold && vertical_span <= max_step_down &&
         !has_reinforcing_support_structure && sub_support_leak_count == 0U &&
         !stale_lower_anchor_mix;
}

int CountAscendingNeighborSupportRefs(int cell,
                                      const LocalTerrainMap &map,
                                      const std::unordered_map<int, float> &support_ref_by_cell,
                                      float support_ref,
                                      float upper_height_threshold,
                                      float max_step_up) {
  if (!std::isfinite(support_ref)) {
    return 0;
  }

  const int row = cell / map.cols();
  const int col = cell % map.cols();
  // Neighbor support must rise clearly above the current support before we call it an ascending stair trend.
  const float min_ascending_support_delta = 0.5f * upper_height_threshold;
  int ascending_stair_support_count = 0;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      const int nr = row + dr;
      const int nc = col + dc;
      if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
        continue;
      }
      if (dr == 0 && dc == 0) {
        continue;
      }
      const int neighbor = nr * map.cols() + nc;
      const auto neighbor_support_it = support_ref_by_cell.find(neighbor);
      if (neighbor_support_it == support_ref_by_cell.end()) {
        continue;
      }
      const float neighbor_support = neighbor_support_it->second;
      if (!std::isfinite(neighbor_support)) {
        continue;
      }
      if (neighbor_support >= support_ref + min_ascending_support_delta &&
          neighbor_support <= support_ref + max_step_up) {
        ++ascending_stair_support_count;
      }
    }
  }
  return ascending_stair_support_count;
}

bool ShouldRejectBelowRobotUpstairGroundMix(bool has_support_anchor,
                                            float relative_support_anchor,
                                            float relative_support_ref,
                                            float relative_upper_z,
                                            float vertical_span,
                                            int ascending_stair_support_count,
                                            int aligned_neighbor_support_count,
                                            int min_neighbor_upper_support_cells,
                                            float upper_height_threshold,
                                            float max_step_up,
                                            uint16_t sub_support_leak_count) {
  const float min_below_robot_support_depth = -0.5f * upper_height_threshold;
  const float max_ground_mix_span = max_step_up + upper_height_threshold;
  const float max_below_robot_upper_z = 0.0f;
  const bool anchor_is_compatible_with_below_robot_mix =
      !has_support_anchor ||
      (std::isfinite(relative_support_anchor) &&
       relative_support_anchor <= min_below_robot_support_depth);
  // Upstairs ground-mix is explained by an ascending support trend, with below-robot anchors still allowed.
  return anchor_is_compatible_with_below_robot_mix && std::isfinite(relative_support_ref) &&
         relative_support_ref <= min_below_robot_support_depth &&
         relative_upper_z <= max_below_robot_upper_z &&
         vertical_span <= max_ground_mix_span &&
         ascending_stair_support_count >= std::max(1, min_neighbor_upper_support_cells) &&
         aligned_neighbor_support_count == 0 && sub_support_leak_count == 0U;
}

bool ShouldUseElevatedEffectiveSupportRef(bool has_support_anchor,
                                          float relative_support_anchor,
                                          float relative_support_ref,
                                          float relative_effective_support_candidate_z,
                                          float candidate_gap_above_support_ref,
                                          int upper_band_count,
                                          int anchor_reobserve_count,
                                          int candidate_reobserve_count,
                                          int upper_layer_neighbor_match_count,
                                          int ascending_stair_support_count,
                                          int min_neighbor_upper_support_cells,
                                          float support_anchor_reobserve_tolerance,
                                          float max_step_up,
                                          uint16_t sub_support_leak_count,
                                          bool stale_lower_anchor_mix) {
  const float min_below_robot_support_depth = -0.1f;
  const bool below_robot_layered_structure =
      has_support_anchor && std::isfinite(relative_support_anchor) &&
      relative_support_anchor <= min_below_robot_support_depth &&
      std::isfinite(relative_support_ref) &&
      relative_support_ref <= min_below_robot_support_depth &&
      std::isfinite(relative_effective_support_candidate_z) &&
      relative_effective_support_candidate_z <= 0.0f;
  const bool non_stair_trend =
      ascending_stair_support_count < std::max(1, min_neighbor_upper_support_cells);
  // This is a frontend-local temporary explanation ref, not a new persisted support estimate.
  return below_robot_layered_structure &&
         candidate_gap_above_support_ref > 0.5f * support_anchor_reobserve_tolerance &&
         candidate_gap_above_support_ref <= max_step_up &&
         upper_band_count > 0 && anchor_reobserve_count >= 2 &&
         candidate_reobserve_count >= 2 && upper_layer_neighbor_match_count >= 2 &&
         non_stair_trend && sub_support_leak_count == 0U &&
         !stale_lower_anchor_mix;
}

} // namespace

FrontendOutput PolarFrontend::run(const ProcessedFrame &frame,
                                  const FrameObservability &observability,
                                  const LocalTerrainMap &map) const {
  FrontendOutput output;
  output.support_anchor_used.assign(static_cast<size_t>(map.size()),
                                    std::numeric_limits<float>::quiet_NaN());
  output.sub_support_leak_count.assign(static_cast<size_t>(map.size()), 0U);
  output.upper_support_cell.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_suspicious.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_candidate_cell.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_rejected_by_neighbor_support.assign(static_cast<size_t>(map.size()), 0U);
  output.neighbor_upper_support_count.assign(static_cast<size_t>(map.size()), 0);
  std::vector<std::vector<size_t>> sample_indices_by_cell(static_cast<size_t>(map.size()));

  const float sector_size =
      2.0f * static_cast<float>(M_PI) / static_cast<float>(observability.sectors.size());
  const float yaw = YawFromQuaternion(frame.base_pose_in_odom.orientation);

  for (size_t sample_index = 0; sample_index < frame.odom_samples.size(); ++sample_index) {
    const auto &sample = frame.odom_samples[sample_index];
    int cell = -1;
    if (!map.odomToIndex(sample.point_in_odom.x, sample.point_in_odom.y, cell)) {
      continue;
    }
    sample_indices_by_cell[static_cast<size_t>(cell)].push_back(sample_index);
  }

  std::unordered_map<int, CellStats> stats_by_cell;
  stats_by_cell.reserve(frame.odom_samples.size() / 4U + 1U);
  output.support_candidates.reserve(stats_by_cell.size());
  output.obstacle_candidates.reserve(stats_by_cell.size());
  output.ambiguous_candidates.reserve(stats_by_cell.size());

  std::unordered_set<int> suspicious_cells;
  suspicious_cells.reserve(stats_by_cell.size());
  const auto &layers = map.layers();
  const float upper_height_threshold = config_.geometry.upper_min_height_above_support;
  const float suspicious_vertical_span = config_.geometry.max_step_up * 0.75f;
  const float support_anchor_reobserve_tolerance =
      config_.geometry.support_anchor_reobserve_tolerance;
  const int min_neighbor_upper_support_cells =
      std::max(1, config_.geometry.min_neighbor_upper_support_cells);
  std::unordered_map<int, float> support_ref_by_cell;
  support_ref_by_cell.reserve(stats_by_cell.size());
  std::vector<float> raw_min_z_by_cell(static_cast<size_t>(map.size()),
                                       std::numeric_limits<float>::infinity());
  std::vector<float> support_anchor_candidate_by_cell(static_cast<size_t>(map.size()),
                                                      std::numeric_limits<float>::quiet_NaN());
  std::vector<uint8_t> local_history_anchor_valid(static_cast<size_t>(map.size()), 0U);
  std::vector<int> anchor_reobserve_count_by_cell(static_cast<size_t>(map.size()), 0);
  std::vector<int> below_anchor_count_by_cell(static_cast<size_t>(map.size()), 0);
  std::vector<int> upper_band_count_by_cell(static_cast<size_t>(map.size()), 0);
  std::vector<float> min_upper_band_z_by_cell(static_cast<size_t>(map.size()),
                                              std::numeric_limits<float>::infinity());
  // Frontend-local temporary explanation ref; never persisted as map support.
  std::vector<float> effective_support_ref_by_cell(static_cast<size_t>(map.size()),
                                                   std::numeric_limits<float>::quiet_NaN());
  std::vector<int> ascending_stair_support_count_by_cell(static_cast<size_t>(map.size()), 0);

  for (int cell = 0; cell < map.size(); ++cell) {
    const auto &sample_indices = sample_indices_by_cell[static_cast<size_t>(cell)];
    if (sample_indices.empty()) {
      continue;
    }
    float raw_min_z = std::numeric_limits<float>::infinity();
    for (const size_t sample_index : sample_indices) {
      raw_min_z = std::min(raw_min_z, frame.odom_samples[sample_index].point_in_odom.z);
    }
    raw_min_z_by_cell[static_cast<size_t>(cell)] = raw_min_z;

    const bool has_local_history_anchor = IsValidSupportAnchorCell(layers, cell, config_);
    local_history_anchor_valid[static_cast<size_t>(cell)] = has_local_history_anchor ? 1U : 0U;
    float support_anchor = std::numeric_limits<float>::quiet_NaN();
    bool has_support_anchor = false;
    if (has_local_history_anchor) {
      const float local_anchor = layers.support_height[static_cast<size_t>(cell)];
      if (local_anchor >= raw_min_z - config_.geometry.sub_support_leak_tolerance) {
        support_anchor = local_anchor;
        has_support_anchor = true;
      }
    }
    if (!has_support_anchor) {
      const float neighbor_anchor = ResolveNeighborSupportAnchor(cell, map, layers, config_);
      if (std::isfinite(neighbor_anchor) &&
          neighbor_anchor >= raw_min_z - config_.geometry.sub_support_leak_tolerance) {
        support_anchor = neighbor_anchor;
        has_support_anchor = true;
      }
    }
    if (has_support_anchor) {
      support_anchor_candidate_by_cell[static_cast<size_t>(cell)] = support_anchor;
      for (const size_t sample_index : sample_indices) {
        const auto &sample = frame.odom_samples[sample_index];
        if (sample.point_in_odom.z < support_anchor - config_.geometry.sub_support_leak_tolerance) {
          ++below_anchor_count_by_cell[static_cast<size_t>(cell)];
          continue;
        }
        if (sample.point_in_odom.z <= support_anchor + support_anchor_reobserve_tolerance) {
          ++anchor_reobserve_count_by_cell[static_cast<size_t>(cell)];
        }
        if (sample.point_in_odom.z >= support_anchor + upper_height_threshold) {
          ++upper_band_count_by_cell[static_cast<size_t>(cell)];
          min_upper_band_z_by_cell[static_cast<size_t>(cell)] = std::min(
              min_upper_band_z_by_cell[static_cast<size_t>(cell)], sample.point_in_odom.z);
        }
      }
    }
  }

  for (int cell = 0; cell < map.size(); ++cell) {
    const auto &sample_indices = sample_indices_by_cell[static_cast<size_t>(cell)];
    if (sample_indices.empty()) {
      continue;
    }

    const bool has_local_history_anchor =
        local_history_anchor_valid[static_cast<size_t>(cell)] != 0U;
    float support_anchor = support_anchor_candidate_by_cell[static_cast<size_t>(cell)];
    bool has_support_anchor = std::isfinite(support_anchor);
    int neighborhood_anchor_reobserve_count = 0;
    if (has_support_anchor) {
      const int row = cell / map.cols();
      const int col = cell % map.cols();
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          const int nr = row + dr;
          const int nc = col + dc;
          if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
            continue;
          }
          const int neighbor = nr * map.cols() + nc;
          neighborhood_anchor_reobserve_count +=
              anchor_reobserve_count_by_cell[static_cast<size_t>(neighbor)];
        }
      }
    }
    const bool reject_stale_anchor =
        has_support_anchor &&
        anchor_reobserve_count_by_cell[static_cast<size_t>(cell)] <= 1 &&
        neighborhood_anchor_reobserve_count <= 2 &&
        upper_band_count_by_cell[static_cast<size_t>(cell)] >= 1 &&
        min_upper_band_z_by_cell[static_cast<size_t>(cell)] >= support_anchor + upper_height_threshold;
    const bool reject_wall_only_anchor =
        has_support_anchor &&
        below_anchor_count_by_cell[static_cast<size_t>(cell)] >= 3 &&
        below_anchor_count_by_cell[static_cast<size_t>(cell)] >
            anchor_reobserve_count_by_cell[static_cast<size_t>(cell)] * 2 &&
        (upper_band_count_by_cell[static_cast<size_t>(cell)] == 0 ||
         min_upper_band_z_by_cell[static_cast<size_t>(cell)] <
             support_anchor + upper_height_threshold + support_anchor_reobserve_tolerance);
    if (reject_stale_anchor || reject_wall_only_anchor) {
      has_support_anchor = false;
      support_anchor = std::numeric_limits<float>::quiet_NaN();
    }
    if (has_support_anchor) {
      output.support_anchor_used[static_cast<size_t>(cell)] = support_anchor;
    }

    CellStats stats;
    uint16_t leak_count = 0U;
    for (const size_t sample_index : sample_indices) {
      const auto &sample = frame.odom_samples[sample_index];
      const bool is_leak =
          has_support_anchor &&
          sample.point_in_odom.z < support_anchor - config_.geometry.sub_support_leak_tolerance;
      if (is_leak) {
        if (leak_count < std::numeric_limits<uint16_t>::max()) {
          ++leak_count;
        }
        continue;
      }
      if (reject_stale_anchor &&
          sample.point_in_odom.z <= support_anchor + support_anchor_reobserve_tolerance) {
        continue;
      }
      stats.min_z = std::min(stats.min_z, sample.point_in_odom.z);
      stats.max_z = std::max(stats.max_z, sample.point_in_odom.z);
      ++stats.count;
    }
    output.sub_support_leak_count[static_cast<size_t>(cell)] = leak_count;
    if (stats.count == 0) {
      continue;
    }
    stats_by_cell.emplace(cell, stats);

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
    const float support_ref = has_local_history_anchor && has_support_anchor
                                  ? layers.support_height[static_cast<size_t>(cell)]
                                  : stats.min_z;
    support_ref_by_cell.emplace(cell, support_ref);
    effective_support_ref_by_cell[static_cast<size_t>(cell)] = support_ref;

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

  for (int cell = 0; cell < map.size(); ++cell) {
    if (output.upper_support_cell[static_cast<size_t>(cell)] != 0U) {
      continue;
    }
    const auto support_ref_it = support_ref_by_cell.find(cell);
    if (support_ref_it == support_ref_by_cell.end() || !std::isfinite(support_ref_it->second)) {
      continue;
    }
    const float support_anchor = output.support_anchor_used[static_cast<size_t>(cell)];
    const bool has_support_anchor = std::isfinite(support_anchor);
    for (const size_t sample_index : sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (has_support_anchor &&
          sample.point_in_odom.z < support_anchor - config_.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (sample.point_in_odom.z >= support_ref_it->second + upper_height_threshold) {
        output.upper_support_cell[static_cast<size_t>(cell)] = 1U;
        break;
      }
    }
  }

  for (int cell = 0; cell < map.size(); ++cell) {
    const auto support_ref_it = support_ref_by_cell.find(cell);
    if (support_ref_it == support_ref_by_cell.end()) {
      continue;
    }
    ascending_stair_support_count_by_cell[static_cast<size_t>(cell)] = CountAscendingNeighborSupportRefs(
        cell,
        map,
        support_ref_by_cell,
        support_ref_it->second,
        upper_height_threshold,
        config_.geometry.max_step_up);
  }

  for (int cell = 0; cell < map.size(); ++cell) {
    const auto stats_it = stats_by_cell.find(cell);
    const auto support_ref_it = support_ref_by_cell.find(cell);
    if (stats_it == stats_by_cell.end() || support_ref_it == support_ref_by_cell.end()) {
      continue;
    }
    if (output.upper_support_cell[static_cast<size_t>(cell)] == 0U) {
      continue;
    }

    const float support_anchor = output.support_anchor_used[static_cast<size_t>(cell)];
    const bool has_support_anchor = std::isfinite(support_anchor);
    if (!has_support_anchor) {
      continue;
    }
    const float support_ref = support_ref_it->second;
    const float upper_layer_consensus_candidate_z = ResolveNeighborUpperLayerConsensusHeight(
        cell,
        map,
        support_anchor_candidate_by_cell,
        upper_band_count_by_cell,
        min_upper_band_z_by_cell,
        output.sub_support_leak_count,
        upper_height_threshold,
        config_.geometry.max_step_up);
    const float anchored_support_candidate_z =
        ResolveNeighborSupportConsensusHeight(cell, map, layers, config_);
    const float effective_support_candidate_z = std::isfinite(upper_layer_consensus_candidate_z)
                                                    ? upper_layer_consensus_candidate_z
                                                    : anchored_support_candidate_z;
    if (!std::isfinite(effective_support_candidate_z)) {
      continue;
    }
    int candidate_reobserve_count = 0;
    for (const size_t sample_index : sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (sample.point_in_odom.z < support_anchor - config_.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (std::abs(sample.point_in_odom.z - effective_support_candidate_z) <=
          support_anchor_reobserve_tolerance) {
        ++candidate_reobserve_count;
      }
    }

    const float relative_support_anchor =
        support_anchor - frame.base_pose_in_odom.position.z();
    const float relative_support_ref = support_ref - frame.base_pose_in_odom.position.z();
    const float relative_effective_support_candidate_z =
        effective_support_candidate_z - frame.base_pose_in_odom.position.z();
    const float candidate_gap_above_support_ref = effective_support_candidate_z - support_ref;
    const bool stale_lower_anchor_mix =
        upper_band_count_by_cell[static_cast<size_t>(cell)] >=
            std::max(2, anchor_reobserve_count_by_cell[static_cast<size_t>(cell)]) &&
        min_upper_band_z_by_cell[static_cast<size_t>(cell)] >=
            support_anchor + upper_height_threshold;
    const bool using_upper_layer_consensus_candidate =
        std::isfinite(upper_layer_consensus_candidate_z);
    const int upper_layer_neighbor_match_count =
        using_upper_layer_consensus_candidate
            ? CountNeighborUpperLayersAlignedToHeight(cell,
                                                      map,
                                                      support_anchor_candidate_by_cell,
                                                      upper_band_count_by_cell,
                                                      min_upper_band_z_by_cell,
                                                      output.sub_support_leak_count,
                                                      effective_support_candidate_z,
                                                      support_anchor_reobserve_tolerance,
                                                      upper_height_threshold,
                                                      config_.geometry.max_step_up)
            : CountNeighborAnchorsAlignedToHeight(cell,
                                                  map,
                                                  layers,
                                                  config_,
                                                  effective_support_candidate_z,
                                                  support_anchor_reobserve_tolerance);
    const bool use_elevated_effective_support_ref = ShouldUseElevatedEffectiveSupportRef(
        has_support_anchor,
        relative_support_anchor,
        relative_support_ref,
        relative_effective_support_candidate_z,
        candidate_gap_above_support_ref,
        upper_band_count_by_cell[static_cast<size_t>(cell)],
        anchor_reobserve_count_by_cell[static_cast<size_t>(cell)],
        candidate_reobserve_count,
        upper_layer_neighbor_match_count,
        ascending_stair_support_count_by_cell[static_cast<size_t>(cell)],
        min_neighbor_upper_support_cells,
        support_anchor_reobserve_tolerance,
        config_.geometry.max_step_up,
        output.sub_support_leak_count[static_cast<size_t>(cell)],
        stale_lower_anchor_mix);
    if (!use_elevated_effective_support_ref) {
      continue;
    }

    effective_support_ref_by_cell[static_cast<size_t>(cell)] = effective_support_candidate_z;
    bool has_elevated_upper_support = false;
    for (const size_t sample_index : sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (sample.point_in_odom.z < support_anchor - config_.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (sample.point_in_odom.z >= effective_support_candidate_z + upper_height_threshold) {
        has_elevated_upper_support = true;
        break;
      }
    }
    output.upper_support_cell[static_cast<size_t>(cell)] = has_elevated_upper_support ? 1U : 0U;
  }

  for (const int cell : suspicious_cells) {
    const int row = cell / map.cols();
    const int col = cell % map.cols();
    int support_count = 0;
    int aligned_neighbor_support_count = 0;
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        const int nr = row + dr;
        const int nc = col + dc;
        if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
          continue;
        }
        const int neighbor = nr * map.cols() + nc;
        const bool is_upper_support = output.upper_support_cell[static_cast<size_t>(neighbor)] != 0U;
        support_count += is_upper_support ? 1 : 0;
        if (const auto stats_it = stats_by_cell.find(cell);
            stats_it != stats_by_cell.end() && IsValidSupportAnchorCell(layers, neighbor, config_) &&
            std::abs(layers.support_height[static_cast<size_t>(neighbor)] - stats_it->second.max_z) <=
                support_anchor_reobserve_tolerance) {
          ++aligned_neighbor_support_count;
        }
      }
    }
    output.neighbor_upper_support_count[static_cast<size_t>(cell)] =
        static_cast<int8_t>(std::clamp(support_count, 0, 9));
    if (support_count >= min_neighbor_upper_support_cells &&
        aligned_neighbor_support_count < min_neighbor_upper_support_cells) {
      const auto stats_it = stats_by_cell.find(cell);
      if (stats_it != stats_by_cell.end()) {
        const auto raw_support_ref_it = support_ref_by_cell.find(cell);
        if (raw_support_ref_it == support_ref_by_cell.end()) {
          continue;
        }
        const float vertical_span = stats_it->second.max_z - stats_it->second.min_z;
        const float support_ref = effective_support_ref_by_cell[static_cast<size_t>(cell)];
        const float raw_support_ref = raw_support_ref_it->second;
        const float support_anchor = output.support_anchor_used[static_cast<size_t>(cell)];
        const bool has_support_anchor = std::isfinite(support_anchor);
        const float relative_support_anchor =
            support_anchor - frame.base_pose_in_odom.position.z();
        const float relative_support_ref = support_ref - frame.base_pose_in_odom.position.z();
        const float relative_upper_z = stats_it->second.max_z - frame.base_pose_in_odom.position.z();
        const bool stale_lower_anchor_mix =
            has_support_anchor &&
            upper_band_count_by_cell[static_cast<size_t>(cell)] >=
                std::max(2, anchor_reobserve_count_by_cell[static_cast<size_t>(cell)]) &&
            min_upper_band_z_by_cell[static_cast<size_t>(cell)] >=
                support_anchor + upper_height_threshold;
        const int ascending_stair_support_count =
            ascending_stair_support_count_by_cell[static_cast<size_t>(cell)];
        const bool below_robot_stair_mix = ShouldRejectBelowRobotStairMix(
            raw_support_ref - frame.base_pose_in_odom.position.z(),
            relative_upper_z,
            support_count,
            config_.geometry.max_step_down,
            upper_height_threshold,
            output.sub_support_leak_count[static_cast<size_t>(cell)],
            stale_lower_anchor_mix);
        const bool below_robot_ground_layer_mix = ShouldRejectBelowRobotGroundLayerMix(
            raw_support_ref - frame.base_pose_in_odom.position.z(),
            relative_upper_z,
            vertical_span,
            aligned_neighbor_support_count,
            config_.geometry.max_step_down,
            upper_height_threshold,
            output.sub_support_leak_count[static_cast<size_t>(cell)],
            stale_lower_anchor_mix);
        const bool below_robot_upstair_ground_mix = ShouldRejectBelowRobotUpstairGroundMix(
            has_support_anchor,
            relative_support_anchor,
            raw_support_ref - frame.base_pose_in_odom.position.z(),
            relative_upper_z,
            vertical_span,
            ascending_stair_support_count,
            aligned_neighbor_support_count,
            min_neighbor_upper_support_cells,
            upper_height_threshold,
            config_.geometry.max_step_up,
            output.sub_support_leak_count[static_cast<size_t>(cell)]);
        if (below_robot_stair_mix || below_robot_ground_layer_mix ||
            below_robot_upstair_ground_mix) {
          output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(cell)] = 1U;
          continue;
        }
        const float wall_like_gain_scale =
            output.sub_support_leak_count[static_cast<size_t>(cell)] == 0U
                ? ComputeWallLikeObstacleGainScale(
                      stats_it->second, vertical_span, relative_upper_z, support_count)
                : 1.0f;
        output.obstacle_candidate_cell[static_cast<size_t>(cell)] = 1U;
        output.obstacle_candidates.push_back(
            ObstacleCandidate{cell,
                              stats_it->second.max_z,
                              std::clamp(vertical_span, 0.0f, 1.0f),
                              wall_like_gain_scale});
      }
    } else {
      output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(cell)] = 1U;
    }
  }
  return output;
}

} // namespace passable_area::core
