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

struct StableUpperLayerCandidate {
  float z = std::numeric_limits<float>::quiet_NaN();
  int sample_count = 0;
  int band_count = 0;
  float band_min_z = std::numeric_limits<float>::infinity();
  float band_max_z = -std::numeric_limits<float>::infinity();
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

StableUpperLayerCandidate ExtractStableUpperLayerCandidate(
    const ProcessedFrame &frame,
    const std::vector<size_t> &sample_indices,
    float support_ref,
    float upper_height_threshold,
    float support_anchor_reobserve_tolerance,
    float max_step_up) {
  StableUpperLayerCandidate candidate;
  if (!std::isfinite(support_ref)) {
    return candidate;
  }

  std::vector<float> eligible_z;
  eligible_z.reserve(sample_indices.size());
  const float max_stair_like_gap = max_step_up + support_anchor_reobserve_tolerance;
  for (const size_t sample_index : sample_indices) {
    const float z = frame.odom_samples[sample_index].point_in_odom.z;
    const float gap_above_support = z - support_ref;
    if (gap_above_support < upper_height_threshold || gap_above_support > max_stair_like_gap) {
      continue;
    }
    eligible_z.push_back(z);
  }
  if (eligible_z.size() < 2U) {
    return candidate;
  }

  std::sort(eligible_z.begin(), eligible_z.end());
  const float band_gap_tolerance = support_anchor_reobserve_tolerance;
  const float max_band_thickness = 2.0f * support_anchor_reobserve_tolerance;
  int best_begin = -1;
  int best_end = -1;
  int band_begin = 0;
  for (int i = 1; i <= static_cast<int>(eligible_z.size()); ++i) {
    const bool band_break =
        i == static_cast<int>(eligible_z.size()) ||
        eligible_z[static_cast<size_t>(i)] - eligible_z[static_cast<size_t>(i - 1)] >
            band_gap_tolerance;
    if (!band_break) {
      continue;
    }
    const int band_end = i;
    const int band_count = band_end - band_begin;
    const float band_min_z = eligible_z[static_cast<size_t>(band_begin)];
    const float band_max_z = eligible_z[static_cast<size_t>(band_end - 1)];
    if (band_count >= 2 && band_max_z - band_min_z <= max_band_thickness &&
        (best_begin < 0 || band_count > best_end - best_begin ||
         (band_count == best_end - best_begin &&
          band_min_z < eligible_z[static_cast<size_t>(best_begin)]))) {
      best_begin = band_begin;
      best_end = band_end;
    }
    band_begin = i;
  }
  if (best_begin < 0) {
    return candidate;
  }

  candidate.band_count = best_end - best_begin;
  candidate.sample_count = static_cast<int>(sample_indices.size());
  candidate.band_min_z = eligible_z[static_cast<size_t>(best_begin)];
  candidate.band_max_z = eligible_z[static_cast<size_t>(best_end - 1)];
  const int middle = best_begin + candidate.band_count / 2;
  candidate.z = eligible_z[static_cast<size_t>(middle)];
  return candidate;
}

struct DirectionalStripeStats {
  int aligned_count = 0;
  int forward_bin_count = 0;
};

struct LocalRunComponentStats {
  int aligned_count = 0;
  int forward_bin_count = 0;
  int lateral_bin_count = 0;
  float min_forward = std::numeric_limits<float>::infinity();
  float max_forward = -std::numeric_limits<float>::infinity();
};

template <typename HeightGetter>
DirectionalStripeStats CountAlignedHeightsInDirectionalStripe(
    int cell,
    const LocalTerrainMap &map,
    const ProcessedFrame &frame,
    float yaw,
    const HeightGetter &height_getter,
    float target_z,
    float tolerance,
    float forward_min,
    float forward_max,
    float lateral_abs_max) {
  DirectionalStripeStats stats;
  if (!std::isfinite(target_z)) {
    return stats;
  }

  const Eigen::Vector2f center = map.indexToOdom(cell);
  const int half_extent = 4;
  const int row = cell / map.cols();
  const int col = cell % map.cols();
  std::unordered_set<int> forward_bins;
  for (int dr = -half_extent; dr <= half_extent; ++dr) {
    for (int dc = -half_extent; dc <= half_extent; ++dc) {
      if (dr == 0 && dc == 0) {
        continue;
      }
      const int nr = row + dr;
      const int nc = col + dc;
      if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
        continue;
      }
      const int neighbor = nr * map.cols() + nc;
      const Eigen::Vector2f neighbor_center = map.indexToOdom(neighbor);
      const float odom_dx = neighbor_center.x() - center.x();
      const float odom_dy = neighbor_center.y() - center.y();
      const float local_forward = std::cos(yaw) * odom_dx + std::sin(yaw) * odom_dy;
      const float local_lateral = -std::sin(yaw) * odom_dx + std::cos(yaw) * odom_dy;
      if (local_forward < forward_min || local_forward > forward_max ||
          std::abs(local_lateral) > lateral_abs_max) {
        continue;
      }
      const float neighbor_height = height_getter(neighbor);
      if (!std::isfinite(neighbor_height) ||
          std::abs(neighbor_height - target_z) > tolerance) {
        continue;
      }
      ++stats.aligned_count;
      forward_bins.insert(static_cast<int>(std::floor(local_forward / map.resolution())));
    }
  }
  stats.forward_bin_count = static_cast<int>(forward_bins.size());
  return stats;
}

template <typename HeightGetter>
LocalRunComponentStats CountAlignedHeightComponentInLocalWindow(
    int cell,
    const LocalTerrainMap &map,
    float yaw,
    const HeightGetter &height_getter,
    float target_z,
    float tolerance,
    float forward_min,
    float forward_max,
    float lateral_abs_max) {
  LocalRunComponentStats stats;
  if (!std::isfinite(target_z)) {
    return stats;
  }

  const Eigen::Vector2f center = map.indexToOdom(cell);
  const int half_extent = 4;
  const int row = cell / map.cols();
  const int col = cell % map.cols();
  std::unordered_map<int, std::pair<float, float>> aligned_local_coords;
  aligned_local_coords.reserve(32);
  for (int dr = -half_extent; dr <= half_extent; ++dr) {
    for (int dc = -half_extent; dc <= half_extent; ++dc) {
      const int nr = row + dr;
      const int nc = col + dc;
      if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
        continue;
      }
      const int neighbor = nr * map.cols() + nc;
      const float neighbor_height = height_getter(neighbor);
      if (!std::isfinite(neighbor_height) ||
          std::abs(neighbor_height - target_z) > tolerance) {
        continue;
      }
      const Eigen::Vector2f neighbor_center = map.indexToOdom(neighbor);
      const float odom_dx = neighbor_center.x() - center.x();
      const float odom_dy = neighbor_center.y() - center.y();
      const float local_forward = std::cos(yaw) * odom_dx + std::sin(yaw) * odom_dy;
      const float local_lateral = -std::sin(yaw) * odom_dx + std::cos(yaw) * odom_dy;
      if (local_forward < forward_min || local_forward > forward_max ||
          std::abs(local_lateral) > lateral_abs_max) {
        continue;
      }
      aligned_local_coords.emplace(neighbor, std::make_pair(local_forward, local_lateral));
    }
  }
  if (aligned_local_coords.find(cell) == aligned_local_coords.end()) {
    return stats;
  }

  std::vector<int> queue;
  queue.reserve(aligned_local_coords.size());
  queue.push_back(cell);
  std::unordered_set<int> visited;
  visited.reserve(aligned_local_coords.size());
  visited.insert(cell);
  std::unordered_set<int> forward_bins;
  std::unordered_set<int> lateral_bins;
  while (!queue.empty()) {
    const int current = queue.back();
    queue.pop_back();
    const auto coords_it = aligned_local_coords.find(current);
    if (coords_it == aligned_local_coords.end()) {
      continue;
    }
    const float local_forward = coords_it->second.first;
    const float local_lateral = coords_it->second.second;
    ++stats.aligned_count;
    stats.min_forward = std::min(stats.min_forward, local_forward);
    stats.max_forward = std::max(stats.max_forward, local_forward);
    forward_bins.insert(static_cast<int>(std::floor(local_forward / map.resolution())));
    lateral_bins.insert(static_cast<int>(std::floor(local_lateral / map.resolution())));

    const int current_row = current / map.cols();
    const int current_col = current % map.cols();
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0) {
          continue;
        }
        const int nr = current_row + dr;
        const int nc = current_col + dc;
        if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
          continue;
        }
        const int neighbor = nr * map.cols() + nc;
        if (aligned_local_coords.find(neighbor) == aligned_local_coords.end() ||
            visited.find(neighbor) != visited.end()) {
          continue;
        }
        visited.insert(neighbor);
        queue.push_back(neighbor);
      }
    }
  }
  stats.forward_bin_count = static_cast<int>(forward_bins.size());
  stats.lateral_bin_count = static_cast<int>(lateral_bins.size());
  return stats;
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
  const float max_below_robot_upper_z = 0.25f * upper_height_threshold;
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

bool ShouldUseElevatedEffectiveSupportRefForUnanchoredLayeredStairRun(
    bool has_support_anchor,
    float relative_support_ref,
    float relative_upper_layer_candidate_z,
    float candidate_gap_above_support_ref,
    int upper_layer_band_count,
    float upper_layer_band_thickness,
    const LocalRunComponentStats &upper_run_component_stats,
    const DirectionalStripeStats &backward_lower_run_stats,
    float support_anchor_reobserve_tolerance,
    float max_step_up,
    uint16_t sub_support_leak_count,
    bool below_robot_stair_mix,
    bool below_robot_ground_layer_mix,
    bool below_robot_upstair_ground_mix) {
  const float min_below_robot_support_depth = -0.1f;
  const bool below_robot_layered_structure =
      !has_support_anchor && std::isfinite(relative_support_ref) &&
      relative_support_ref <= min_below_robot_support_depth &&
      std::isfinite(relative_upper_layer_candidate_z) &&
      relative_upper_layer_candidate_z <= 0.0f;
  const bool stair_like_gap = candidate_gap_above_support_ref > support_anchor_reobserve_tolerance &&
                              candidate_gap_above_support_ref <=
                                  max_step_up + support_anchor_reobserve_tolerance;
  const bool has_stable_upper_layer =
      upper_layer_band_count >= 2 &&
      upper_layer_band_thickness <= 2.0f * support_anchor_reobserve_tolerance;
  // Require a local connected stair-run patch, not just a few upper neighbors.
  const bool has_local_stair_run_consistency =
      upper_run_component_stats.aligned_count >= 3 &&
      upper_run_component_stats.forward_bin_count >= 2 &&
      upper_run_component_stats.lateral_bin_count >= 2 &&
      upper_run_component_stats.max_forward - upper_run_component_stats.min_forward >= 0.1f;
  const bool lower_run_exists_behind_current_cell =
      backward_lower_run_stats.aligned_count >= 1 && backward_lower_run_stats.forward_bin_count >= 1;
  return below_robot_layered_structure && stair_like_gap && has_stable_upper_layer &&
         has_local_stair_run_consistency && lower_run_exists_behind_current_cell &&
         sub_support_leak_count == 0U && !below_robot_stair_mix &&
         !below_robot_ground_layer_mix && !below_robot_upstair_ground_mix;
}

bool IsValidUnanchoredLayeredStairRunInterior(const LocalRunComponentStats &upper_run_component_stats) {
  return upper_run_component_stats.aligned_count >= 5 &&
         upper_run_component_stats.forward_bin_count >= 2 &&
         upper_run_component_stats.lateral_bin_count >= 3 &&
         upper_run_component_stats.max_forward - upper_run_component_stats.min_forward >= 0.15f;
}

int CountAdjacentStrictUnanchoredLayeredStairRunInteriorCells(
    int cell,
    const LocalTerrainMap &map,
    const std::vector<uint8_t> &strict_unanchored_stair_run_interior_by_cell,
    const std::vector<float> &unanchored_effective_support_candidate_z_by_cell,
    float target_z,
    float tolerance) {
  if (!std::isfinite(target_z)) {
    return 0;
  }
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
      if (strict_unanchored_stair_run_interior_by_cell[static_cast<size_t>(neighbor)] == 0U) {
        continue;
      }
      const float neighbor_z =
          unanchored_effective_support_candidate_z_by_cell[static_cast<size_t>(neighbor)];
      if (!std::isfinite(neighbor_z) || std::abs(neighbor_z - target_z) > tolerance) {
        continue;
      }
      ++count;
    }
  }
  return count;
}

bool ShouldUseElevatedEffectiveSupportRefForUnanchoredLayeredStairRunEdge(
    bool has_support_anchor,
    float relative_support_ref,
    float relative_upper_layer_candidate_z,
    float candidate_gap_above_support_ref,
    int upper_layer_band_count,
    float upper_layer_band_thickness,
    const LocalRunComponentStats &upper_run_component_stats,
    const DirectionalStripeStats &backward_lower_run_stats,
    int adjacent_valid_patch_interior_count,
    float support_anchor_reobserve_tolerance,
    float max_step_up,
    uint16_t sub_support_leak_count,
    bool below_robot_stair_mix,
    bool below_robot_ground_layer_mix,
    bool below_robot_upstair_ground_mix) {
  const float min_below_robot_support_depth = -0.1f;
  const bool below_robot_layered_structure =
      !has_support_anchor && std::isfinite(relative_support_ref) &&
      relative_support_ref <= min_below_robot_support_depth &&
      std::isfinite(relative_upper_layer_candidate_z) &&
      relative_upper_layer_candidate_z <= 0.0f;
  const bool stair_like_gap =
      candidate_gap_above_support_ref > support_anchor_reobserve_tolerance &&
      candidate_gap_above_support_ref <=
          max_step_up + 0.5f * support_anchor_reobserve_tolerance;
  const bool has_stable_upper_layer =
      upper_layer_band_count >= 2 &&
      upper_layer_band_thickness <= 2.0f * support_anchor_reobserve_tolerance;
  const bool lower_run_exists_behind_current_cell =
      backward_lower_run_stats.aligned_count >= 1 && backward_lower_run_stats.forward_bin_count >= 1;
  const bool edge_cell_has_valid_local_shape =
      upper_run_component_stats.aligned_count >= 3 &&
      upper_run_component_stats.lateral_bin_count >= 2 &&
      upper_run_component_stats.forward_bin_count >= 1;
  const bool edge_shortfall_only =
      upper_run_component_stats.forward_bin_count < 2 ||
      upper_run_component_stats.max_forward - upper_run_component_stats.min_forward < 0.1f;
  return below_robot_layered_structure && stair_like_gap && has_stable_upper_layer &&
         edge_cell_has_valid_local_shape && edge_shortfall_only &&
         lower_run_exists_behind_current_cell && adjacent_valid_patch_interior_count >= 1 &&
         sub_support_leak_count == 0U && !below_robot_stair_mix &&
         !below_robot_ground_layer_mix && !below_robot_upstair_ground_mix;
}

int CountAdjacentValidatedUnanchoredStairRunCells(
    int cell,
    const LocalTerrainMap &map,
    const std::vector<uint8_t> &validated_unanchored_stair_run_by_cell,
    const std::vector<float> &unanchored_effective_support_candidate_z_by_cell,
    float target_z,
    float tolerance) {
  if (!std::isfinite(target_z)) {
    return 0;
  }
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
      if (validated_unanchored_stair_run_by_cell[static_cast<size_t>(neighbor)] == 0U) {
        continue;
      }
      const float neighbor_z =
          unanchored_effective_support_candidate_z_by_cell[static_cast<size_t>(neighbor)];
      if (!std::isfinite(neighbor_z) || std::abs(neighbor_z - target_z) > tolerance) {
        continue;
      }
      ++count;
    }
  }
  return count;
}

int CountAdjacentExplainedEffectiveSupportPatchCells(
    int cell,
    const LocalTerrainMap &map,
    const std::vector<uint8_t> &elevated_effective_support_ref_by_cell,
    const std::vector<uint8_t> &obstacle_rejected_by_neighbor_support,
    const std::vector<float> &effective_support_ref_by_cell,
    float target_z,
    float tolerance) {
  if (!std::isfinite(target_z)) {
    return 0;
  }
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
      const bool explained_patch_neighbor =
          elevated_effective_support_ref_by_cell[static_cast<size_t>(neighbor)] != 0U ||
          obstacle_rejected_by_neighbor_support[static_cast<size_t>(neighbor)] != 0U;
      if (!explained_patch_neighbor) {
        continue;
      }
      const float neighbor_z = effective_support_ref_by_cell[static_cast<size_t>(neighbor)];
      if (!std::isfinite(neighbor_z) || std::abs(neighbor_z - target_z) > tolerance) {
        continue;
      }
      ++count;
    }
  }
  return count;
}

int CountAdjacentUnanchoredStairRunEdgeSupportCells(
    int cell,
    const LocalTerrainMap &map,
    const std::vector<StableUpperLayerCandidate> &stable_upper_layer_candidate_by_cell,
    const std::vector<float> &unanchored_effective_support_candidate_z_by_cell,
    const std::vector<LocalRunComponentStats> &unanchored_upper_run_component_by_cell,
    float target_z,
    float tolerance) {
  if (!std::isfinite(target_z)) {
    return 0;
  }
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
      const float neighbor_z =
          unanchored_effective_support_candidate_z_by_cell[static_cast<size_t>(neighbor)];
      const auto &neighbor_candidate =
          stable_upper_layer_candidate_by_cell[static_cast<size_t>(neighbor)];
      const auto &neighbor_run =
          unanchored_upper_run_component_by_cell[static_cast<size_t>(neighbor)];
      const bool has_edge_support_shape =
          neighbor_candidate.band_count >= 2 &&
          neighbor_run.aligned_count >= 2 &&
          neighbor_run.forward_bin_count >= 1 &&
          neighbor_run.lateral_bin_count >= 2;
      if (!has_edge_support_shape || !std::isfinite(neighbor_z) ||
          std::abs(neighbor_z - target_z) > tolerance) {
        continue;
      }
      ++count;
    }
  }
  return count;
}

bool HasAdjacentUnanchoredStairRunEdgeSupportChainToExplainedPatch(
    int cell,
    const LocalTerrainMap &map,
    const std::vector<StableUpperLayerCandidate> &stable_upper_layer_candidate_by_cell,
    const std::vector<float> &unanchored_effective_support_candidate_z_by_cell,
    const std::vector<LocalRunComponentStats> &unanchored_upper_run_component_by_cell,
    const std::vector<uint8_t> &elevated_effective_support_ref_by_cell,
    const std::vector<uint8_t> &obstacle_rejected_by_neighbor_support,
    const std::vector<float> &effective_support_ref_by_cell,
    float target_z,
    float tolerance) {
  if (!std::isfinite(target_z)) {
    return false;
  }
  const int row = cell / map.cols();
  const int col = cell % map.cols();
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
      const float neighbor_z =
          unanchored_effective_support_candidate_z_by_cell[static_cast<size_t>(neighbor)];
      const auto &neighbor_candidate =
          stable_upper_layer_candidate_by_cell[static_cast<size_t>(neighbor)];
      const auto &neighbor_run =
          unanchored_upper_run_component_by_cell[static_cast<size_t>(neighbor)];
      const bool has_edge_support_shape =
          neighbor_candidate.band_count >= 2 &&
          neighbor_run.aligned_count >= 2 &&
          neighbor_run.forward_bin_count >= 1 &&
          neighbor_run.lateral_bin_count >= 2;
      if (!has_edge_support_shape || !std::isfinite(neighbor_z) ||
          std::abs(neighbor_z - target_z) > tolerance) {
        continue;
      }
      const int neighbor_row = neighbor / map.cols();
      const int neighbor_col = neighbor % map.cols();
      for (int ndr = -1; ndr <= 1; ++ndr) {
        for (int ndc = -1; ndc <= 1; ++ndc) {
          if (ndr == 0 && ndc == 0) {
            continue;
          }
          const int nnr = neighbor_row + ndr;
          const int nnc = neighbor_col + ndc;
          if (nnr < 0 || nnr >= map.rows() || nnc < 0 || nnc >= map.cols()) {
            continue;
          }
          const int second_neighbor = nnr * map.cols() + nnc;
          const bool explained_patch_neighbor =
              elevated_effective_support_ref_by_cell[static_cast<size_t>(second_neighbor)] != 0U ||
              obstacle_rejected_by_neighbor_support[static_cast<size_t>(second_neighbor)] != 0U;
          const float second_neighbor_z = effective_support_ref_by_cell[static_cast<size_t>(second_neighbor)];
          if (!explained_patch_neighbor || !std::isfinite(second_neighbor_z) ||
              std::abs(second_neighbor_z - target_z) > tolerance) {
            continue;
          }
          return true;
        }
      }
    }
  }
  return false;
}

bool ShouldUseElevatedEffectiveSupportRefForUnanchoredLayeredStairRunFrontEdge(
    bool has_support_anchor,
    float relative_support_ref,
    float relative_upper_layer_candidate_z,
    float candidate_gap_above_support_ref,
    int upper_layer_band_count,
    float upper_layer_band_thickness,
    const LocalRunComponentStats &upper_run_component_stats,
    const DirectionalStripeStats &backward_lower_run_stats,
    int adjacent_validated_patch_count,
    float support_anchor_reobserve_tolerance,
    float upper_height_threshold,
    float max_step_up,
    uint16_t sub_support_leak_count,
    bool below_robot_stair_mix,
    bool below_robot_ground_layer_mix,
    bool below_robot_upstair_ground_mix) {
  const float min_below_robot_support_depth = -0.1f;
  const float max_front_edge_positive_upper_z =
      std::min(0.5f * upper_height_threshold, support_anchor_reobserve_tolerance);
  const bool near_below_robot_layered_structure =
      !has_support_anchor && std::isfinite(relative_support_ref) &&
      relative_support_ref <= min_below_robot_support_depth &&
      std::isfinite(relative_upper_layer_candidate_z) &&
      relative_upper_layer_candidate_z <= max_front_edge_positive_upper_z;
  const bool stair_like_gap = candidate_gap_above_support_ref > support_anchor_reobserve_tolerance &&
                              candidate_gap_above_support_ref <=
                                  max_step_up + support_anchor_reobserve_tolerance;
  const bool has_stable_upper_layer =
      upper_layer_band_count >= 2 &&
      upper_layer_band_thickness <= 2.0f * support_anchor_reobserve_tolerance;
  const bool front_edge_local_shape =
      upper_run_component_stats.aligned_count >= 2 &&
      upper_run_component_stats.lateral_bin_count >= 2 &&
      upper_run_component_stats.forward_bin_count >= 1;
  const bool front_edge_shortfall =
      upper_run_component_stats.forward_bin_count < 2 ||
      upper_run_component_stats.max_forward - upper_run_component_stats.min_forward < 0.1f;
  const bool lower_run_exists_behind_current_cell =
      backward_lower_run_stats.aligned_count >= 1 && backward_lower_run_stats.forward_bin_count >= 1;
  return near_below_robot_layered_structure && stair_like_gap && has_stable_upper_layer &&
         front_edge_local_shape && front_edge_shortfall &&
         lower_run_exists_behind_current_cell && adjacent_validated_patch_count >= 1 &&
         sub_support_leak_count == 0U && !below_robot_stair_mix &&
         !below_robot_ground_layer_mix && !below_robot_upstair_ground_mix;
}

bool ShouldUseElevatedEffectiveSupportRefForUnanchoredLayeredStairRunEdgeOfEdge(
    bool has_support_anchor,
    float relative_support_ref,
    float relative_upper_layer_candidate_z,
    float candidate_gap_above_support_ref,
    int upper_layer_band_count,
    float upper_layer_band_thickness,
    const LocalRunComponentStats &upper_run_component_stats,
    const DirectionalStripeStats &backward_lower_run_stats,
    int neighbor_upper_support_count,
    int adjacent_elevated_patch_count,
    bool has_two_hop_edge_support_chain,
    float support_anchor_reobserve_tolerance,
    float upper_height_threshold,
    float max_step_up,
    uint16_t sub_support_leak_count,
    bool below_robot_stair_mix,
    bool below_robot_ground_layer_mix,
    bool below_robot_upstair_ground_mix) {
  const float min_below_robot_support_depth = -0.1f;
  const float max_front_edge_positive_upper_z =
      std::min(0.5f * upper_height_threshold, support_anchor_reobserve_tolerance);
  const bool near_below_robot_layered_structure =
      !has_support_anchor && std::isfinite(relative_support_ref) &&
      relative_support_ref <= min_below_robot_support_depth &&
      std::isfinite(relative_upper_layer_candidate_z) &&
      relative_upper_layer_candidate_z <= max_front_edge_positive_upper_z;
  const bool stair_like_gap = candidate_gap_above_support_ref > support_anchor_reobserve_tolerance &&
                              candidate_gap_above_support_ref <=
                                  max_step_up + support_anchor_reobserve_tolerance;
  const bool has_stable_upper_layer =
      upper_layer_band_count >= 2 &&
      upper_layer_band_thickness <= 2.0f * support_anchor_reobserve_tolerance;
  const bool current_cell_keeps_stair_run_lateral_shape =
      upper_run_component_stats.aligned_count >= 1 &&
      upper_run_component_stats.lateral_bin_count >= 2;
  const bool lower_run_exists_behind_current_cell =
      backward_lower_run_stats.aligned_count >= 1 && backward_lower_run_stats.forward_bin_count >= 1;
  return near_below_robot_layered_structure && stair_like_gap && has_stable_upper_layer &&
         current_cell_keeps_stair_run_lateral_shape && lower_run_exists_behind_current_cell &&
         (neighbor_upper_support_count >= 2 || adjacent_elevated_patch_count >= 1) &&
         (adjacent_elevated_patch_count >= 1 || has_two_hop_edge_support_chain) &&
         sub_support_leak_count == 0U &&
         !below_robot_stair_mix && !below_robot_ground_layer_mix &&
         !below_robot_upstair_ground_mix;
}

bool ShouldUseElevatedEffectiveSupportRefForAnchoredLayeredStairRunFrontEdge(
    bool has_support_anchor,
    float relative_support_anchor,
    float relative_support_ref,
    float relative_upper_layer_candidate_z,
    float candidate_gap_above_support_ref,
    int upper_band_count,
    int candidate_reobserve_count,
    int adjacent_validated_patch_count,
    const DirectionalStripeStats &backward_lower_run_stats,
    float support_anchor_reobserve_tolerance,
    float upper_height_threshold,
    float max_step_up,
    uint16_t sub_support_leak_count,
    bool below_robot_stair_mix,
    bool below_robot_ground_layer_mix,
    bool below_robot_upstair_ground_mix) {
  const float min_below_robot_support_depth = -0.1f;
  const float max_front_edge_positive_upper_z =
      std::min(0.5f * upper_height_threshold, support_anchor_reobserve_tolerance);
  const bool anchored_front_edge_layered_structure =
      has_support_anchor && std::isfinite(relative_support_anchor) &&
      relative_support_anchor <= min_below_robot_support_depth &&
      std::isfinite(relative_support_ref) &&
      relative_support_ref <= min_below_robot_support_depth &&
      std::isfinite(relative_upper_layer_candidate_z) &&
      relative_upper_layer_candidate_z <= max_front_edge_positive_upper_z;
  const bool stair_like_gap =
      candidate_gap_above_support_ref > support_anchor_reobserve_tolerance &&
      candidate_gap_above_support_ref <=
          max_step_up + support_anchor_reobserve_tolerance;
  const bool has_stable_upper_layer =
      upper_band_count >= 2 && candidate_reobserve_count >= 2;
  const bool lower_run_exists_behind_current_cell =
      backward_lower_run_stats.aligned_count >= 1 && backward_lower_run_stats.forward_bin_count >= 1;
  return anchored_front_edge_layered_structure && stair_like_gap &&
         has_stable_upper_layer && lower_run_exists_behind_current_cell &&
         adjacent_validated_patch_count >= 1 && sub_support_leak_count == 0U &&
         !below_robot_stair_mix && !below_robot_ground_layer_mix &&
         !below_robot_upstair_ground_mix;
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
  output.effective_support_ref_elevated.assign(static_cast<size_t>(map.size()), 0U);
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
  std::vector<uint8_t> reject_stale_anchor_by_cell(static_cast<size_t>(map.size()), 0U);
  std::vector<uint8_t> reject_wall_only_anchor_by_cell(static_cast<size_t>(map.size()), 0U);
  std::vector<int> anchor_reobserve_count_by_cell(static_cast<size_t>(map.size()), 0);
  std::vector<int> below_anchor_count_by_cell(static_cast<size_t>(map.size()), 0);
  std::vector<int> upper_band_count_by_cell(static_cast<size_t>(map.size()), 0);
  std::vector<float> min_upper_band_z_by_cell(static_cast<size_t>(map.size()),
                                              std::numeric_limits<float>::infinity());
  std::vector<StableUpperLayerCandidate> stable_upper_layer_candidate_by_cell(
      static_cast<size_t>(map.size()));
  // Frontend-local temporary explanation ref; never persisted as map support.
  std::vector<float> effective_support_ref_by_cell(static_cast<size_t>(map.size()),
                                                   std::numeric_limits<float>::quiet_NaN());
  std::vector<uint8_t> elevated_effective_support_ref_by_cell(static_cast<size_t>(map.size()), 0U);
  std::vector<int> ascending_stair_support_count_by_cell(static_cast<size_t>(map.size()), 0);
  std::vector<uint8_t> strict_unanchored_stair_run_interior_by_cell(static_cast<size_t>(map.size()), 0U);
  std::vector<uint8_t> validated_unanchored_stair_run_by_cell(static_cast<size_t>(map.size()), 0U);
  std::vector<uint8_t> anchored_front_edge_reinterpreted_by_cell(static_cast<size_t>(map.size()), 0U);
  std::vector<float> unanchored_effective_support_candidate_z_by_cell(
      static_cast<size_t>(map.size()), std::numeric_limits<float>::quiet_NaN());
  std::vector<LocalRunComponentStats> unanchored_upper_run_component_by_cell(
      static_cast<size_t>(map.size()));
  std::vector<DirectionalStripeStats> unanchored_backward_lower_run_by_cell(
      static_cast<size_t>(map.size()));

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
    reject_stale_anchor_by_cell[static_cast<size_t>(cell)] = reject_stale_anchor ? 1U : 0U;
    reject_wall_only_anchor_by_cell[static_cast<size_t>(cell)] = reject_wall_only_anchor ? 1U : 0U;
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
    if (!has_support_anchor) {
      stable_upper_layer_candidate_by_cell[static_cast<size_t>(cell)] =
          ExtractStableUpperLayerCandidate(frame,
                                          sample_indices,
                                          support_ref,
                                          upper_height_threshold,
                                          support_anchor_reobserve_tolerance,
                                          config_.geometry.max_step_up);
    }

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
    const float support_ref = support_ref_it->second;
    const int row = cell / map.cols();
    const int col = cell % map.cols();
    int current_neighbor_upper_support_count = 0;
    int aligned_neighbor_support_count = 0;
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        const int nr = row + dr;
        const int nc = col + dc;
        if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
          continue;
        }
        const int neighbor = nr * map.cols() + nc;
        if (output.upper_support_cell[static_cast<size_t>(neighbor)] != 0U) {
          ++current_neighbor_upper_support_count;
        }
        if (IsValidSupportAnchorCell(layers, neighbor, config_) &&
            std::abs(layers.support_height[static_cast<size_t>(neighbor)] - stats_it->second.max_z) <=
                support_anchor_reobserve_tolerance) {
          ++aligned_neighbor_support_count;
        }
      }
    }
    const auto &stable_upper_layer_candidate =
        stable_upper_layer_candidate_by_cell[static_cast<size_t>(cell)];
    const float current_upper_layer_candidate_z =
        has_support_anchor ? (upper_band_count_by_cell[static_cast<size_t>(cell)] >= 2
                                  ? min_upper_band_z_by_cell[static_cast<size_t>(cell)]
                                  : std::numeric_limits<float>::quiet_NaN())
                           : stable_upper_layer_candidate.z;
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
    const float relative_support_anchor = has_support_anchor
                                              ? support_anchor - frame.base_pose_in_odom.position.z()
                                              : std::numeric_limits<float>::quiet_NaN();
    const float relative_support_ref = support_ref - frame.base_pose_in_odom.position.z();
    const float relative_upper_layer_candidate_z =
        std::isfinite(current_upper_layer_candidate_z)
            ? current_upper_layer_candidate_z - frame.base_pose_in_odom.position.z()
            : std::numeric_limits<float>::quiet_NaN();
    const bool stale_lower_anchor_mix =
        has_support_anchor &&
        upper_band_count_by_cell[static_cast<size_t>(cell)] >=
            std::max(2, anchor_reobserve_count_by_cell[static_cast<size_t>(cell)]) &&
        min_upper_band_z_by_cell[static_cast<size_t>(cell)] >=
            support_anchor + upper_height_threshold;
    const float relative_upper_z = stats_it->second.max_z - frame.base_pose_in_odom.position.z();
    const float vertical_span = stats_it->second.max_z - stats_it->second.min_z;
    const int ascending_stair_support_count =
        ascending_stair_support_count_by_cell[static_cast<size_t>(cell)];
    const bool below_robot_stair_mix = ShouldRejectBelowRobotStairMix(
        relative_support_ref,
        relative_upper_z,
        current_neighbor_upper_support_count,
        config_.geometry.max_step_down,
        upper_height_threshold,
        output.sub_support_leak_count[static_cast<size_t>(cell)],
        stale_lower_anchor_mix);
    const bool below_robot_ground_layer_mix = ShouldRejectBelowRobotGroundLayerMix(
        relative_support_ref,
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
        relative_support_ref,
        relative_upper_z,
        vertical_span,
        ascending_stair_support_count,
        aligned_neighbor_support_count,
        min_neighbor_upper_support_cells,
        upper_height_threshold,
        config_.geometry.max_step_up,
        output.sub_support_leak_count[static_cast<size_t>(cell)]);

    const bool using_upper_layer_consensus_candidate =
        std::isfinite(upper_layer_consensus_candidate_z);
    const float effective_support_candidate_z = has_support_anchor
                                                    ? (using_upper_layer_consensus_candidate
                                                           ? upper_layer_consensus_candidate_z
                                                           : anchored_support_candidate_z)
                                                    : current_upper_layer_candidate_z;
    if (!std::isfinite(effective_support_candidate_z)) {
      continue;
    }
    int candidate_reobserve_count = 0;
    for (const size_t sample_index : sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (has_support_anchor &&
          sample.point_in_odom.z < support_anchor - config_.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (std::abs(sample.point_in_odom.z - effective_support_candidate_z) <=
          support_anchor_reobserve_tolerance) {
        ++candidate_reobserve_count;
      }
    }

    const float relative_effective_support_candidate_z =
        effective_support_candidate_z - frame.base_pose_in_odom.position.z();
    const float candidate_gap_above_support_ref = effective_support_candidate_z - support_ref;
    const int effective_support_candidate_neighbor_match_count =
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
    const LocalRunComponentStats upper_run_component_stats =
        CountAlignedHeightComponentInLocalWindow(
            cell,
            map,
            yaw,
            [&stable_upper_layer_candidate_by_cell](int neighbor) {
              return stable_upper_layer_candidate_by_cell[static_cast<size_t>(neighbor)].z;
            },
            effective_support_candidate_z,
            support_anchor_reobserve_tolerance,
            -0.05f,
            0.65f,
            0.45f);
    const DirectionalStripeStats forward_lower_run_stats =
        CountAlignedHeightsInDirectionalStripe(
            cell,
            map,
            frame,
            yaw,
            [&support_ref_by_cell](int neighbor) {
              const auto it = support_ref_by_cell.find(neighbor);
              return it == support_ref_by_cell.end()
                         ? std::numeric_limits<float>::quiet_NaN()
                         : it->second;
            },
            support_ref,
            support_anchor_reobserve_tolerance,
            0.05f,
            0.65f,
            0.25f);
    const DirectionalStripeStats backward_lower_run_stats =
        CountAlignedHeightsInDirectionalStripe(
            cell,
            map,
            frame,
            yaw,
            [&support_ref_by_cell](int neighbor) {
              const auto it = support_ref_by_cell.find(neighbor);
              return it == support_ref_by_cell.end()
                         ? std::numeric_limits<float>::quiet_NaN()
                         : it->second;
            },
            support_ref,
            support_anchor_reobserve_tolerance,
            -0.65f,
            -0.05f,
            0.25f);
    const bool use_elevated_effective_support_ref = has_support_anchor
                                                        ? ShouldUseElevatedEffectiveSupportRef(
                                                              has_support_anchor,
                                                              relative_support_anchor,
                                                              relative_support_ref,
                                                              relative_effective_support_candidate_z,
                                                              candidate_gap_above_support_ref,
                                                              upper_band_count_by_cell[static_cast<size_t>(cell)],
                                                              anchor_reobserve_count_by_cell[static_cast<size_t>(cell)],
                                                              candidate_reobserve_count,
                                                              effective_support_candidate_neighbor_match_count,
                                                              ascending_stair_support_count,
                                                              min_neighbor_upper_support_cells,
                                                              support_anchor_reobserve_tolerance,
                                                              config_.geometry.max_step_up,
                                                              output.sub_support_leak_count[static_cast<size_t>(cell)],
                                                              stale_lower_anchor_mix)
                                                        : ShouldUseElevatedEffectiveSupportRefForUnanchoredLayeredStairRun(
                                                              has_support_anchor,
                                                              relative_support_ref,
                                                              relative_upper_layer_candidate_z,
                                                              candidate_gap_above_support_ref,
                                                              stable_upper_layer_candidate.band_count,
                                                              stable_upper_layer_candidate.band_max_z -
                                                                  stable_upper_layer_candidate.band_min_z,
                                                              upper_run_component_stats,
                                                              backward_lower_run_stats,
                                                              support_anchor_reobserve_tolerance,
                                                              config_.geometry.max_step_up,
                                                              output.sub_support_leak_count[static_cast<size_t>(cell)],
                                                              below_robot_stair_mix,
                                                              below_robot_ground_layer_mix,
                                                              below_robot_upstair_ground_mix);
    if (!has_support_anchor) {
      unanchored_effective_support_candidate_z_by_cell[static_cast<size_t>(cell)] =
          effective_support_candidate_z;
      unanchored_upper_run_component_by_cell[static_cast<size_t>(cell)] =
          upper_run_component_stats;
      unanchored_backward_lower_run_by_cell[static_cast<size_t>(cell)] =
          backward_lower_run_stats;
      if (use_elevated_effective_support_ref) {
        validated_unanchored_stair_run_by_cell[static_cast<size_t>(cell)] = 1U;
        if (IsValidUnanchoredLayeredStairRunInterior(upper_run_component_stats)) {
          strict_unanchored_stair_run_interior_by_cell[static_cast<size_t>(cell)] = 1U;
        }
      }
    }
    if (!use_elevated_effective_support_ref) {
      continue;
    }

    effective_support_ref_by_cell[static_cast<size_t>(cell)] = effective_support_candidate_z;
    elevated_effective_support_ref_by_cell[static_cast<size_t>(cell)] = 1U;
    output.effective_support_ref_elevated[static_cast<size_t>(cell)] = 1U;
    bool has_elevated_upper_support = false;
    for (const size_t sample_index : sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (has_support_anchor &&
          sample.point_in_odom.z < support_anchor - config_.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (sample.point_in_odom.z >= effective_support_candidate_z + upper_height_threshold) {
        has_elevated_upper_support = true;
        break;
      }
    }
    output.upper_support_cell[static_cast<size_t>(cell)] = has_elevated_upper_support ? 1U : 0U;
  }

  for (int cell = 0; cell < map.size(); ++cell) {
    if (elevated_effective_support_ref_by_cell[static_cast<size_t>(cell)] != 0U) {
      continue;
    }
    if (std::isfinite(output.support_anchor_used[static_cast<size_t>(cell)])) {
      continue;
    }
    const auto stats_it = stats_by_cell.find(cell);
    const auto support_ref_it = support_ref_by_cell.find(cell);
    if (stats_it == stats_by_cell.end() || support_ref_it == support_ref_by_cell.end()) {
      continue;
    }
    const float effective_support_candidate_z =
        unanchored_effective_support_candidate_z_by_cell[static_cast<size_t>(cell)];
    if (!std::isfinite(effective_support_candidate_z)) {
      continue;
    }
    const auto &upper_run_component_stats =
        unanchored_upper_run_component_by_cell[static_cast<size_t>(cell)];
    const auto &backward_lower_run_stats =
        unanchored_backward_lower_run_by_cell[static_cast<size_t>(cell)];
    const auto &stable_upper_layer_candidate =
        stable_upper_layer_candidate_by_cell[static_cast<size_t>(cell)];
    const float support_ref = support_ref_it->second;
    const float relative_support_ref = support_ref - frame.base_pose_in_odom.position.z();
    const float relative_upper_layer_candidate_z =
        effective_support_candidate_z - frame.base_pose_in_odom.position.z();
    const float candidate_gap_above_support_ref = effective_support_candidate_z - support_ref;
    const float relative_upper_z = stats_it->second.max_z - frame.base_pose_in_odom.position.z();
    const float vertical_span = stats_it->second.max_z - stats_it->second.min_z;
    int support_count = 0;
    int aligned_neighbor_support_count = 0;
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
        if (output.upper_support_cell[static_cast<size_t>(neighbor)] != 0U) {
          ++support_count;
        }
        if (IsValidSupportAnchorCell(layers, neighbor, config_) &&
            std::abs(layers.support_height[static_cast<size_t>(neighbor)] - stats_it->second.max_z) <=
                support_anchor_reobserve_tolerance) {
          ++aligned_neighbor_support_count;
        }
      }
    }
    const int ascending_stair_support_count =
        ascending_stair_support_count_by_cell[static_cast<size_t>(cell)];
    const bool below_robot_stair_mix = ShouldRejectBelowRobotStairMix(
        relative_support_ref,
        relative_upper_z,
        support_count,
        config_.geometry.max_step_down,
        upper_height_threshold,
        output.sub_support_leak_count[static_cast<size_t>(cell)],
        false);
    const bool below_robot_ground_layer_mix = ShouldRejectBelowRobotGroundLayerMix(
        relative_support_ref,
        relative_upper_z,
        vertical_span,
        aligned_neighbor_support_count,
        config_.geometry.max_step_down,
        upper_height_threshold,
        output.sub_support_leak_count[static_cast<size_t>(cell)],
        false);
    const bool below_robot_upstair_ground_mix = ShouldRejectBelowRobotUpstairGroundMix(
        false,
        std::numeric_limits<float>::quiet_NaN(),
        relative_support_ref,
        relative_upper_z,
        vertical_span,
        ascending_stair_support_count,
        aligned_neighbor_support_count,
        min_neighbor_upper_support_cells,
        upper_height_threshold,
        config_.geometry.max_step_up,
        output.sub_support_leak_count[static_cast<size_t>(cell)]);
    const int adjacent_valid_patch_interior_count =
        CountAdjacentStrictUnanchoredLayeredStairRunInteriorCells(
            cell,
            map,
            strict_unanchored_stair_run_interior_by_cell,
            unanchored_effective_support_candidate_z_by_cell,
            effective_support_candidate_z,
            support_anchor_reobserve_tolerance);
    const bool use_edge_elevated_effective_support_ref =
        ShouldUseElevatedEffectiveSupportRefForUnanchoredLayeredStairRunEdge(
            false,
            relative_support_ref,
            relative_upper_layer_candidate_z,
            candidate_gap_above_support_ref,
            stable_upper_layer_candidate.band_count,
            stable_upper_layer_candidate.band_max_z - stable_upper_layer_candidate.band_min_z,
            upper_run_component_stats,
            backward_lower_run_stats,
            adjacent_valid_patch_interior_count,
            support_anchor_reobserve_tolerance,
            config_.geometry.max_step_up,
            output.sub_support_leak_count[static_cast<size_t>(cell)],
            below_robot_stair_mix,
            below_robot_ground_layer_mix,
            below_robot_upstair_ground_mix);
    if (!use_edge_elevated_effective_support_ref) {
      continue;
    }

    effective_support_ref_by_cell[static_cast<size_t>(cell)] = effective_support_candidate_z;
    elevated_effective_support_ref_by_cell[static_cast<size_t>(cell)] = 1U;
    output.effective_support_ref_elevated[static_cast<size_t>(cell)] = 1U;
    validated_unanchored_stair_run_by_cell[static_cast<size_t>(cell)] = 1U;
    bool has_elevated_upper_support = false;
    for (const size_t sample_index : sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (sample.point_in_odom.z >= effective_support_candidate_z + upper_height_threshold) {
        has_elevated_upper_support = true;
        break;
      }
    }
    output.upper_support_cell[static_cast<size_t>(cell)] = has_elevated_upper_support ? 1U : 0U;
  }

  for (int cell = 0; cell < map.size(); ++cell) {
    if (elevated_effective_support_ref_by_cell[static_cast<size_t>(cell)] != 0U) {
      continue;
    }
    if (std::isfinite(output.support_anchor_used[static_cast<size_t>(cell)])) {
      continue;
    }
    const auto stats_it = stats_by_cell.find(cell);
    const auto support_ref_it = support_ref_by_cell.find(cell);
    if (stats_it == stats_by_cell.end() || support_ref_it == support_ref_by_cell.end()) {
      continue;
    }
    const float effective_support_candidate_z =
        unanchored_effective_support_candidate_z_by_cell[static_cast<size_t>(cell)];
    if (!std::isfinite(effective_support_candidate_z)) {
      continue;
    }
    const auto &upper_run_component_stats =
        unanchored_upper_run_component_by_cell[static_cast<size_t>(cell)];
    const auto &backward_lower_run_stats =
        unanchored_backward_lower_run_by_cell[static_cast<size_t>(cell)];
    const auto &stable_upper_layer_candidate =
        stable_upper_layer_candidate_by_cell[static_cast<size_t>(cell)];
    const float support_ref = support_ref_it->second;
    const float relative_support_ref = support_ref - frame.base_pose_in_odom.position.z();
    const float relative_upper_layer_candidate_z =
        effective_support_candidate_z - frame.base_pose_in_odom.position.z();
    const float candidate_gap_above_support_ref = effective_support_candidate_z - support_ref;
    const float relative_upper_z = stats_it->second.max_z - frame.base_pose_in_odom.position.z();
    const float vertical_span = stats_it->second.max_z - stats_it->second.min_z;
    int support_count = 0;
    int aligned_neighbor_support_count = 0;
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
        if (output.upper_support_cell[static_cast<size_t>(neighbor)] != 0U) {
          ++support_count;
        }
        if (IsValidSupportAnchorCell(layers, neighbor, config_) &&
            std::abs(layers.support_height[static_cast<size_t>(neighbor)] - stats_it->second.max_z) <=
                support_anchor_reobserve_tolerance) {
          ++aligned_neighbor_support_count;
        }
      }
    }
    const int ascending_stair_support_count =
        ascending_stair_support_count_by_cell[static_cast<size_t>(cell)];
    const bool below_robot_stair_mix = ShouldRejectBelowRobotStairMix(
        relative_support_ref,
        relative_upper_z,
        support_count,
        config_.geometry.max_step_down,
        upper_height_threshold,
        output.sub_support_leak_count[static_cast<size_t>(cell)],
        false);
    const bool below_robot_ground_layer_mix = ShouldRejectBelowRobotGroundLayerMix(
        relative_support_ref,
        relative_upper_z,
        vertical_span,
        aligned_neighbor_support_count,
        config_.geometry.max_step_down,
        upper_height_threshold,
        output.sub_support_leak_count[static_cast<size_t>(cell)],
        false);
    const bool below_robot_upstair_ground_mix = ShouldRejectBelowRobotUpstairGroundMix(
        false,
        std::numeric_limits<float>::quiet_NaN(),
        relative_support_ref,
        relative_upper_z,
        vertical_span,
        ascending_stair_support_count,
        aligned_neighbor_support_count,
        min_neighbor_upper_support_cells,
        upper_height_threshold,
        config_.geometry.max_step_up,
        output.sub_support_leak_count[static_cast<size_t>(cell)]);
    const int adjacent_validated_patch_count =
        CountAdjacentExplainedEffectiveSupportPatchCells(
            cell,
            map,
            elevated_effective_support_ref_by_cell,
            output.obstacle_rejected_by_neighbor_support,
            effective_support_ref_by_cell,
            effective_support_candidate_z,
            support_anchor_reobserve_tolerance);
    const bool use_front_edge_elevated_effective_support_ref =
        ShouldUseElevatedEffectiveSupportRefForUnanchoredLayeredStairRunFrontEdge(
            false,
            relative_support_ref,
            relative_upper_layer_candidate_z,
            candidate_gap_above_support_ref,
            stable_upper_layer_candidate.band_count,
            stable_upper_layer_candidate.band_max_z - stable_upper_layer_candidate.band_min_z,
            upper_run_component_stats,
            backward_lower_run_stats,
            adjacent_validated_patch_count,
            support_anchor_reobserve_tolerance,
            upper_height_threshold,
            config_.geometry.max_step_up,
            output.sub_support_leak_count[static_cast<size_t>(cell)],
            below_robot_stair_mix,
            below_robot_ground_layer_mix,
            below_robot_upstair_ground_mix);
    if (!use_front_edge_elevated_effective_support_ref) {
      continue;
    }

    effective_support_ref_by_cell[static_cast<size_t>(cell)] = effective_support_candidate_z;
    elevated_effective_support_ref_by_cell[static_cast<size_t>(cell)] = 1U;
    output.effective_support_ref_elevated[static_cast<size_t>(cell)] = 1U;
    validated_unanchored_stair_run_by_cell[static_cast<size_t>(cell)] = 1U;
    bool has_elevated_upper_support = false;
    for (const size_t sample_index : sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (sample.point_in_odom.z >= effective_support_candidate_z + upper_height_threshold) {
        has_elevated_upper_support = true;
        break;
      }
    }
    output.upper_support_cell[static_cast<size_t>(cell)] = has_elevated_upper_support ? 1U : 0U;
  }

  for (int cell = 0; cell < map.size(); ++cell) {
    if (elevated_effective_support_ref_by_cell[static_cast<size_t>(cell)] != 0U) {
      continue;
    }
    if (std::isfinite(output.support_anchor_used[static_cast<size_t>(cell)])) {
      continue;
    }
    const auto stats_it = stats_by_cell.find(cell);
    const auto support_ref_it = support_ref_by_cell.find(cell);
    if (stats_it == stats_by_cell.end() || support_ref_it == support_ref_by_cell.end()) {
      continue;
    }
    const float effective_support_candidate_z =
        unanchored_effective_support_candidate_z_by_cell[static_cast<size_t>(cell)];
    if (!std::isfinite(effective_support_candidate_z)) {
      continue;
    }
    const auto &upper_run_component_stats =
        unanchored_upper_run_component_by_cell[static_cast<size_t>(cell)];
    const auto &backward_lower_run_stats =
        unanchored_backward_lower_run_by_cell[static_cast<size_t>(cell)];
    const auto &stable_upper_layer_candidate =
        stable_upper_layer_candidate_by_cell[static_cast<size_t>(cell)];
    const float support_ref = support_ref_it->second;
    const float relative_support_ref = support_ref - frame.base_pose_in_odom.position.z();
    const float relative_upper_layer_candidate_z =
        effective_support_candidate_z - frame.base_pose_in_odom.position.z();
    const float candidate_gap_above_support_ref = effective_support_candidate_z - support_ref;
    const float relative_upper_z = stats_it->second.max_z - frame.base_pose_in_odom.position.z();
    const float vertical_span = stats_it->second.max_z - stats_it->second.min_z;
    int support_count = 0;
    int aligned_neighbor_support_count = 0;
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
        if (output.upper_support_cell[static_cast<size_t>(neighbor)] != 0U) {
          ++support_count;
        }
        if (IsValidSupportAnchorCell(layers, neighbor, config_) &&
            std::abs(layers.support_height[static_cast<size_t>(neighbor)] - stats_it->second.max_z) <=
                support_anchor_reobserve_tolerance) {
          ++aligned_neighbor_support_count;
        }
      }
    }
    const int ascending_stair_support_count =
        ascending_stair_support_count_by_cell[static_cast<size_t>(cell)];
    const bool below_robot_stair_mix = ShouldRejectBelowRobotStairMix(
        relative_support_ref,
        relative_upper_z,
        support_count,
        config_.geometry.max_step_down,
        upper_height_threshold,
        output.sub_support_leak_count[static_cast<size_t>(cell)],
        false);
    const bool below_robot_ground_layer_mix = ShouldRejectBelowRobotGroundLayerMix(
        relative_support_ref,
        relative_upper_z,
        vertical_span,
        aligned_neighbor_support_count,
        config_.geometry.max_step_down,
        upper_height_threshold,
        output.sub_support_leak_count[static_cast<size_t>(cell)],
        false);
    const bool below_robot_upstair_ground_mix = ShouldRejectBelowRobotUpstairGroundMix(
        false,
        std::numeric_limits<float>::quiet_NaN(),
        relative_support_ref,
        relative_upper_z,
        vertical_span,
        ascending_stair_support_count,
        aligned_neighbor_support_count,
        min_neighbor_upper_support_cells,
        upper_height_threshold,
        config_.geometry.max_step_up,
        output.sub_support_leak_count[static_cast<size_t>(cell)]);
    const int adjacent_validated_patch_count =
        CountAdjacentExplainedEffectiveSupportPatchCells(
            cell,
            map,
            elevated_effective_support_ref_by_cell,
            output.obstacle_rejected_by_neighbor_support,
            effective_support_ref_by_cell,
            effective_support_candidate_z,
            support_anchor_reobserve_tolerance);
    const bool has_two_hop_edge_support_chain =
        HasAdjacentUnanchoredStairRunEdgeSupportChainToExplainedPatch(
            cell,
            map,
            stable_upper_layer_candidate_by_cell,
            unanchored_effective_support_candidate_z_by_cell,
            unanchored_upper_run_component_by_cell,
            elevated_effective_support_ref_by_cell,
            output.obstacle_rejected_by_neighbor_support,
            effective_support_ref_by_cell,
            effective_support_candidate_z,
            support_anchor_reobserve_tolerance);
    const bool use_edge_of_edge_elevated_effective_support_ref =
        ShouldUseElevatedEffectiveSupportRefForUnanchoredLayeredStairRunEdgeOfEdge(
            false,
            relative_support_ref,
            relative_upper_layer_candidate_z,
            candidate_gap_above_support_ref,
            stable_upper_layer_candidate.band_count,
            stable_upper_layer_candidate.band_max_z - stable_upper_layer_candidate.band_min_z,
            upper_run_component_stats,
            backward_lower_run_stats,
            support_count,
            adjacent_validated_patch_count,
            has_two_hop_edge_support_chain,
            support_anchor_reobserve_tolerance,
            upper_height_threshold,
            config_.geometry.max_step_up,
            output.sub_support_leak_count[static_cast<size_t>(cell)],
            below_robot_stair_mix,
            below_robot_ground_layer_mix,
            below_robot_upstair_ground_mix);
    if (!use_edge_of_edge_elevated_effective_support_ref) {
      continue;
    }

    effective_support_ref_by_cell[static_cast<size_t>(cell)] = effective_support_candidate_z;
    elevated_effective_support_ref_by_cell[static_cast<size_t>(cell)] = 1U;
    output.effective_support_ref_elevated[static_cast<size_t>(cell)] = 1U;
    validated_unanchored_stair_run_by_cell[static_cast<size_t>(cell)] = 1U;
    bool has_elevated_upper_support = false;
    for (const size_t sample_index : sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (sample.point_in_odom.z >= effective_support_candidate_z + upper_height_threshold) {
        has_elevated_upper_support = true;
        break;
      }
    }
    output.upper_support_cell[static_cast<size_t>(cell)] = has_elevated_upper_support ? 1U : 0U;
  }

  for (int cell = 0; cell < map.size(); ++cell) {
    if (elevated_effective_support_ref_by_cell[static_cast<size_t>(cell)] != 0U) {
      continue;
    }
    const float support_anchor = output.support_anchor_used[static_cast<size_t>(cell)];
    if (!std::isfinite(support_anchor)) {
      continue;
    }
    const auto stats_it = stats_by_cell.find(cell);
    const auto support_ref_it = support_ref_by_cell.find(cell);
    if (stats_it == stats_by_cell.end() || support_ref_it == support_ref_by_cell.end()) {
      continue;
    }
    if (output.upper_support_cell[static_cast<size_t>(cell)] == 0U) {
      continue;
    }

    const float support_ref = support_ref_it->second;
    const float current_upper_layer_candidate_z =
        upper_band_count_by_cell[static_cast<size_t>(cell)] >= 2
            ? min_upper_band_z_by_cell[static_cast<size_t>(cell)]
            : std::numeric_limits<float>::quiet_NaN();
    if (!std::isfinite(current_upper_layer_candidate_z)) {
      continue;
    }

    int candidate_reobserve_count = 0;
    for (const size_t sample_index : sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (sample.point_in_odom.z < support_anchor - config_.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (std::abs(sample.point_in_odom.z - current_upper_layer_candidate_z) <=
          support_anchor_reobserve_tolerance) {
        ++candidate_reobserve_count;
      }
    }

    const float relative_support_anchor =
        support_anchor - frame.base_pose_in_odom.position.z();
    const float relative_support_ref = support_ref - frame.base_pose_in_odom.position.z();
    const float relative_upper_layer_candidate_z =
        current_upper_layer_candidate_z - frame.base_pose_in_odom.position.z();
    const float candidate_gap_above_support_ref =
        current_upper_layer_candidate_z - support_ref;
    const float relative_upper_z = stats_it->second.max_z - frame.base_pose_in_odom.position.z();
    const float vertical_span = stats_it->second.max_z - stats_it->second.min_z;
    int support_count = 0;
    int aligned_neighbor_support_count = 0;
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
        if (output.upper_support_cell[static_cast<size_t>(neighbor)] != 0U) {
          ++support_count;
        }
        if (IsValidSupportAnchorCell(layers, neighbor, config_) &&
            std::abs(layers.support_height[static_cast<size_t>(neighbor)] - stats_it->second.max_z) <=
                support_anchor_reobserve_tolerance) {
          ++aligned_neighbor_support_count;
        }
      }
    }
    const int ascending_stair_support_count =
        ascending_stair_support_count_by_cell[static_cast<size_t>(cell)];
    const bool stale_lower_anchor_mix =
        upper_band_count_by_cell[static_cast<size_t>(cell)] >=
            std::max(2, anchor_reobserve_count_by_cell[static_cast<size_t>(cell)]) &&
        min_upper_band_z_by_cell[static_cast<size_t>(cell)] >=
            support_anchor + upper_height_threshold;
    const bool below_robot_stair_mix = ShouldRejectBelowRobotStairMix(
        relative_support_ref,
        relative_upper_z,
        support_count,
        config_.geometry.max_step_down,
        upper_height_threshold,
        output.sub_support_leak_count[static_cast<size_t>(cell)],
        stale_lower_anchor_mix);
    const bool below_robot_ground_layer_mix = ShouldRejectBelowRobotGroundLayerMix(
        relative_support_ref,
        relative_upper_z,
        vertical_span,
        aligned_neighbor_support_count,
        config_.geometry.max_step_down,
        upper_height_threshold,
        output.sub_support_leak_count[static_cast<size_t>(cell)],
        stale_lower_anchor_mix);
    const bool below_robot_upstair_ground_mix = ShouldRejectBelowRobotUpstairGroundMix(
        true,
        relative_support_anchor,
        relative_support_ref,
        relative_upper_z,
        vertical_span,
        ascending_stair_support_count,
        aligned_neighbor_support_count,
        min_neighbor_upper_support_cells,
        upper_height_threshold,
        config_.geometry.max_step_up,
        output.sub_support_leak_count[static_cast<size_t>(cell)]);
    const DirectionalStripeStats backward_lower_run_stats =
        CountAlignedHeightsInDirectionalStripe(
            cell,
            map,
            frame,
            yaw,
            [&support_ref_by_cell](int neighbor) {
              const auto it = support_ref_by_cell.find(neighbor);
              return it == support_ref_by_cell.end()
                         ? std::numeric_limits<float>::quiet_NaN()
                         : it->second;
            },
            support_ref,
            support_anchor_reobserve_tolerance,
            -0.65f,
            -0.05f,
            0.25f);
    const int adjacent_validated_patch_count =
        CountAdjacentExplainedEffectiveSupportPatchCells(
            cell,
            map,
            elevated_effective_support_ref_by_cell,
            output.obstacle_rejected_by_neighbor_support,
            effective_support_ref_by_cell,
            current_upper_layer_candidate_z,
            support_anchor_reobserve_tolerance);
    const bool use_anchored_front_edge_elevated_effective_support_ref =
        ShouldUseElevatedEffectiveSupportRefForAnchoredLayeredStairRunFrontEdge(
            true,
            relative_support_anchor,
            relative_support_ref,
            relative_upper_layer_candidate_z,
            candidate_gap_above_support_ref,
            upper_band_count_by_cell[static_cast<size_t>(cell)],
            candidate_reobserve_count,
            adjacent_validated_patch_count,
            backward_lower_run_stats,
            support_anchor_reobserve_tolerance,
            upper_height_threshold,
            config_.geometry.max_step_up,
            output.sub_support_leak_count[static_cast<size_t>(cell)],
            below_robot_stair_mix,
            below_robot_ground_layer_mix,
            below_robot_upstair_ground_mix);
    if (!use_anchored_front_edge_elevated_effective_support_ref) {
      continue;
    }

    effective_support_ref_by_cell[static_cast<size_t>(cell)] = current_upper_layer_candidate_z;
    elevated_effective_support_ref_by_cell[static_cast<size_t>(cell)] = 1U;
    anchored_front_edge_reinterpreted_by_cell[static_cast<size_t>(cell)] = 1U;
    output.effective_support_ref_elevated[static_cast<size_t>(cell)] = 1U;
    bool has_elevated_upper_support = false;
    for (const size_t sample_index : sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (sample.point_in_odom.z < support_anchor - config_.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (sample.point_in_odom.z >= current_upper_layer_candidate_z + upper_height_threshold) {
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
    if (elevated_effective_support_ref_by_cell[static_cast<size_t>(cell)] != 0U &&
        !std::isfinite(output.support_anchor_used[static_cast<size_t>(cell)]) &&
        output.upper_support_cell[static_cast<size_t>(cell)] == 0U) {
      output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(cell)] = 1U;
      continue;
    }
    if (anchored_front_edge_reinterpreted_by_cell[static_cast<size_t>(cell)] != 0U &&
        output.upper_support_cell[static_cast<size_t>(cell)] == 0U) {
      output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(cell)] = 1U;
      continue;
    }
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
