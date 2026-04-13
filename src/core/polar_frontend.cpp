#include "passable_area/core/polar_frontend.hpp"
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

enum class AnchorValidityDecision : uint8_t {
  kInvalidInsufficientSupport = 0U,
  kValid = 1U,
  kInvalidStale = 2U,
  kInvalidWallOnly = 3U,
};

enum class ExplanationDecision : uint8_t {
  kNone = 0U,
  kBelowRobotStairMix = 1U,
  kBelowRobotGroundLayerMix = 2U,
  kBelowRobotUpstairGroundMix = 3U,
};

struct CellWorkspace {
  float raw_min_z = std::numeric_limits<float>::infinity();
  float support_anchor_candidate = std::numeric_limits<float>::quiet_NaN();
  float support_ref = std::numeric_limits<float>::quiet_NaN();
  float min_upper_band_z = std::numeric_limits<float>::infinity();
  CellStats stats;
  int anchor_reobserve_count = 0;
  int below_anchor_count = 0;
  int upper_band_count = 0;
  int ascending_stair_support_count = 0;
  uint16_t sub_support_leak_count = 0U;
  uint8_t local_history_anchor_valid = 0U;
  AnchorValidityDecision anchor_validity =
      AnchorValidityDecision::kInvalidInsufficientSupport;
  bool has_stats = false;
  bool stale_lower_anchor_mix = false;
};

float ComputeWallLikeObstacleGainScale(const CellStats &stats,
                                       float vertical_span,
                                       float relative_upper_z,
                                       int neighbor_upper_support_count) {
  if (stats.count < 10 || vertical_span < 0.5f || relative_upper_z < 0.0f) {
    return 1.0f;
  }

  const float count_bonus = std::clamp(
      (static_cast<float>(stats.count) - 10.0f) * 0.02f, 0.0f, 0.55f);
  const float span_bonus =
      std::clamp((vertical_span - 0.5f) * 0.5f, 0.0f, 0.2f);
  const float neighbor_bonus = std::clamp(
      (static_cast<float>(neighbor_upper_support_count) - 2.0f) * 0.1f, 0.0f,
      0.2f);
  return 1.0f + count_bonus + span_bonus + neighbor_bonus;
}

float NormalizeAngle(float angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool IsValidSupportAnchorCell(const TerrainLayers &layers, int cell,
                              const Config &config) {
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
  if (static_cast<SupportState>(
          layers.support_state[static_cast<size_t>(cell)]) ==
      SupportState::kNone) {
    return false;
  }
  return layers.last_reliable_age[static_cast<size_t>(cell)] <=
         static_cast<uint16_t>(
             std::max(1, config.persistence.support_persistence_frames));
}

float ResolveNeighborSupportAnchor(int cell, const LocalTerrainMap &map,
                                   const TerrainLayers &layers,
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
      neighbor_heights.push_back(
          layers.support_height[static_cast<size_t>(neighbor)]);
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
      neighbor_heights.push_back(
          layers.support_height[static_cast<size_t>(neighbor)]);
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

int CountNeighborAnchorsAlignedToHeight(int cell, const LocalTerrainMap &map,
                                        const TerrainLayers &layers,
                                        const Config &config, float target_z,
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
      if (std::abs(layers.support_height[static_cast<size_t>(neighbor)] -
                   target_z) <= tolerance) {
        ++match_count;
      }
    }
  }
  return match_count;
}

int CountValidNeighborAnchors(int cell, const LocalTerrainMap &map,
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

float ResolveNeighborUpperLayerConsensusHeight(
    int cell, const LocalTerrainMap &map,
    const std::vector<CellWorkspace> &cell_workspaces,
    float upper_height_threshold, float max_step_up) {
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
      const auto &neighbor_workspace =
          cell_workspaces[static_cast<size_t>(neighbor)];
      const float neighbor_support_anchor =
          neighbor_workspace.support_anchor_candidate;
      const float neighbor_min_upper_band_z =
          neighbor_workspace.min_upper_band_z;
      if (!std::isfinite(neighbor_support_anchor) ||
          !std::isfinite(neighbor_min_upper_band_z) ||
          neighbor_workspace.upper_band_count == 0 ||
          neighbor_workspace.sub_support_leak_count != 0U) {
        continue;
      }
      const float neighbor_upper_gap =
          neighbor_min_upper_band_z - neighbor_support_anchor;
      if (neighbor_upper_gap < upper_height_threshold ||
          neighbor_upper_gap > max_step_up) {
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

int CountNeighborUpperLayersAlignedToHeight(
    int cell, const LocalTerrainMap &map,
    const std::vector<CellWorkspace> &cell_workspaces, float target_z,
    float tolerance, float upper_height_threshold, float max_step_up) {
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
      const auto &neighbor_workspace =
          cell_workspaces[static_cast<size_t>(neighbor)];
      const float neighbor_support_anchor =
          neighbor_workspace.support_anchor_candidate;
      const float neighbor_min_upper_band_z =
          neighbor_workspace.min_upper_band_z;
      if (!std::isfinite(neighbor_support_anchor) ||
          !std::isfinite(neighbor_min_upper_band_z) ||
          neighbor_workspace.upper_band_count == 0 ||
          neighbor_workspace.sub_support_leak_count != 0U) {
        continue;
      }
      const float neighbor_upper_gap =
          neighbor_min_upper_band_z - neighbor_support_anchor;
      if (neighbor_upper_gap < upper_height_threshold ||
          neighbor_upper_gap > max_step_up) {
        continue;
      }
      if (std::abs(neighbor_min_upper_band_z - target_z) <= tolerance) {
        ++match_count;
      }
    }
  }
  return match_count;
}

int CountAscendingNeighborSupportRefs(
    int cell, const LocalTerrainMap &map,
    const std::vector<CellWorkspace> &cell_workspaces, float support_ref,
    float upper_height_threshold, float max_step_up) {
  if (!std::isfinite(support_ref)) {
    return 0;
  }

  const int row = cell / map.cols();
  const int col = cell % map.cols();
  // Neighbor support must rise clearly above the current support before we call
  // it an ascending stair trend.
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
      const auto &neighbor_workspace =
          cell_workspaces[static_cast<size_t>(neighbor)];
      if (!std::isfinite(neighbor_workspace.support_ref)) {
        continue;
      }
      const float neighbor_support = neighbor_workspace.support_ref;
      if (neighbor_support >= support_ref + min_ascending_support_delta &&
          neighbor_support <= support_ref + max_step_up) {
        ++ascending_stair_support_count;
      }
    }
  }
  return ascending_stair_support_count;
}

ExplanationDecision ClassifyExplanationDecision(
    bool has_support_anchor, float relative_support_anchor,
    float relative_support_ref, float relative_upper_z, float vertical_span,
    int support_count, int ascending_stair_support_count,
    int aligned_neighbor_support_count, int min_neighbor_upper_support_cells,
    float max_step_down, float upper_height_threshold, float max_step_up,
    uint16_t sub_support_leak_count, bool stale_lower_anchor_mix) {
  if (std::isfinite(relative_support_ref) &&
      relative_support_ref <= -max_step_down &&
      relative_upper_z <= -upper_height_threshold && support_count >= 3 &&
      (sub_support_leak_count > 0U || stale_lower_anchor_mix)) {
    return ExplanationDecision::kBelowRobotStairMix;
  }

  // Downstairs ground-mix only trusts local anchored upper-support structure;
  // dense upper bands alone do not override a below-robot ground-layer
  // interpretation.
  const bool has_reinforcing_support_structure =
      aligned_neighbor_support_count > 0;
  if (std::isfinite(relative_support_ref) &&
      relative_support_ref <= -max_step_down &&
      relative_upper_z <= -upper_height_threshold &&
      vertical_span <= max_step_down && !has_reinforcing_support_structure &&
      sub_support_leak_count == 0U && !stale_lower_anchor_mix) {
    return ExplanationDecision::kBelowRobotGroundLayerMix;
  }

  const float min_below_robot_support_depth = -0.5f * upper_height_threshold;
  const float max_ground_mix_span = max_step_up + upper_height_threshold;
  const float max_below_robot_upper_z = 0.0f;
  const bool anchor_is_compatible_with_below_robot_mix =
      !has_support_anchor ||
      (std::isfinite(relative_support_anchor) &&
       relative_support_anchor <= min_below_robot_support_depth);
  // Upstairs ground-mix is explained by an ascending support trend, with
  // below-robot anchors still allowed.
  if (anchor_is_compatible_with_below_robot_mix &&
      std::isfinite(relative_support_ref) &&
      relative_support_ref <= min_below_robot_support_depth &&
      relative_upper_z <= max_below_robot_upper_z &&
      vertical_span <= max_ground_mix_span &&
      ascending_stair_support_count >=
          std::max(1, min_neighbor_upper_support_cells) &&
      aligned_neighbor_support_count == 0 && sub_support_leak_count == 0U) {
    return ExplanationDecision::kBelowRobotUpstairGroundMix;
  }

  return ExplanationDecision::kNone;
}

bool ShouldUseElevatedEffectiveSupportRef(
    bool has_support_anchor, float relative_support_anchor,
    float relative_support_ref, float relative_effective_support_candidate_z,
    float candidate_gap_above_support_ref, int upper_band_count,
    int anchor_reobserve_count, int candidate_reobserve_count,
    int upper_layer_neighbor_match_count, int ascending_stair_support_count,
    int min_neighbor_upper_support_cells,
    float support_anchor_reobserve_tolerance, float max_step_up,
    uint16_t sub_support_leak_count, bool stale_lower_anchor_mix) {
  const float min_below_robot_support_depth = -0.1f;
  const bool below_robot_layered_structure =
      has_support_anchor && std::isfinite(relative_support_anchor) &&
      relative_support_anchor <= min_below_robot_support_depth &&
      std::isfinite(relative_support_ref) &&
      relative_support_ref <= min_below_robot_support_depth &&
      std::isfinite(relative_effective_support_candidate_z) &&
      relative_effective_support_candidate_z <= 0.0f;
  const bool non_stair_trend = ascending_stair_support_count <
                               std::max(1, min_neighbor_upper_support_cells);
  // This is a frontend-local temporary explanation ref, not a new persisted
  // support estimate.
  return below_robot_layered_structure &&
         candidate_gap_above_support_ref >
             0.5f * support_anchor_reobserve_tolerance &&
         candidate_gap_above_support_ref <= max_step_up &&
         upper_band_count > 0 && anchor_reobserve_count >= 2 &&
         candidate_reobserve_count >= 2 &&
         upper_layer_neighbor_match_count >= 2 && non_stair_trend &&
         sub_support_leak_count == 0U && !stale_lower_anchor_mix;
}

std::vector<std::vector<size_t>>
GroupSampleIndicesByCell(const ProcessedFrame &frame,
                         const LocalTerrainMap &map) {
  std::vector<std::vector<size_t>> sample_indices_by_cell(
      static_cast<size_t>(map.size()));
  for (size_t sample_index = 0; sample_index < frame.odom_samples.size();
       ++sample_index) {
    const auto &sample = frame.odom_samples[sample_index];
    int cell = -1;
    if (!map.odomToIndex(sample.point_in_odom.x, sample.point_in_odom.y,
                         cell)) {
      continue;
    }
    sample_indices_by_cell[static_cast<size_t>(cell)].push_back(sample_index);
  }
  return sample_indices_by_cell;
}

int CountActiveCells(
    const std::vector<std::vector<size_t>> &sample_indices_by_cell) {
  return static_cast<int>(std::count_if(
      sample_indices_by_cell.begin(), sample_indices_by_cell.end(),
      [](const auto &sample_indices) { return !sample_indices.empty(); }));
}

void ResolveAnchors(
    const ProcessedFrame &frame, const LocalTerrainMap &map,
    const TerrainLayers &layers, const Config &config,
    const std::vector<std::vector<size_t>> &sample_indices_by_cell,
    std::vector<CellWorkspace> &cell_workspaces) {
  for (int cell = 0; cell < map.size(); ++cell) {
    const auto &sample_indices =
        sample_indices_by_cell[static_cast<size_t>(cell)];
    if (sample_indices.empty()) {
      continue;
    }

    auto &workspace = cell_workspaces[static_cast<size_t>(cell)];
    for (const size_t sample_index : sample_indices) {
      workspace.raw_min_z =
          std::min(workspace.raw_min_z,
                   frame.odom_samples[sample_index].point_in_odom.z);
    }

    const bool has_local_history_anchor =
        IsValidSupportAnchorCell(layers, cell, config);
    workspace.local_history_anchor_valid = has_local_history_anchor ? 1U : 0U;

    float support_anchor = std::numeric_limits<float>::quiet_NaN();
    bool has_support_anchor = false;
    if (has_local_history_anchor) {
      const float local_anchor =
          layers.support_height[static_cast<size_t>(cell)];
      if (local_anchor >=
          workspace.raw_min_z - config.geometry.sub_support_leak_tolerance) {
        support_anchor = local_anchor;
        has_support_anchor = true;
      }
    }
    if (!has_support_anchor) {
      const float neighbor_anchor =
          ResolveNeighborSupportAnchor(cell, map, layers, config);
      if (std::isfinite(neighbor_anchor) &&
          neighbor_anchor >= workspace.raw_min_z -
                                 config.geometry.sub_support_leak_tolerance) {
        support_anchor = neighbor_anchor;
        has_support_anchor = true;
      }
    }
    if (!has_support_anchor) {
      workspace.anchor_validity =
          AnchorValidityDecision::kInvalidInsufficientSupport;
      continue;
    }

    workspace.support_anchor_candidate = support_anchor;
    workspace.anchor_validity = AnchorValidityDecision::kValid;
    for (const size_t sample_index : sample_indices) {
      const auto &sample = frame.odom_samples[sample_index];
      if (sample.point_in_odom.z <
          support_anchor - config.geometry.sub_support_leak_tolerance) {
        ++workspace.below_anchor_count;
        continue;
      }
      if (sample.point_in_odom.z <=
          support_anchor + config.geometry.support_anchor_reobserve_tolerance) {
        ++workspace.anchor_reobserve_count;
      }
      if (sample.point_in_odom.z >=
          support_anchor + config.geometry.upper_min_height_above_support) {
        ++workspace.upper_band_count;
        workspace.min_upper_band_z =
            std::min(workspace.min_upper_band_z, sample.point_in_odom.z);
      }
    }
  }
}

int CountNeighborhoodAnchorReobserveCount(
    int cell, const LocalTerrainMap &map,
    const std::vector<CellWorkspace> &cell_workspaces) {
  int neighborhood_anchor_reobserve_count = 0;
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
          cell_workspaces[static_cast<size_t>(neighbor)].anchor_reobserve_count;
    }
  }
  return neighborhood_anchor_reobserve_count;
}

AnchorValidityDecision
ClassifyAnchorValidity(const CellWorkspace &workspace,
                       int neighborhood_anchor_reobserve_count,
                       float support_anchor, float upper_height_threshold,
                       float support_anchor_reobserve_tolerance) {
  if (!std::isfinite(support_anchor)) {
    return AnchorValidityDecision::kInvalidInsufficientSupport;
  }

  const bool reject_stale_anchor =
      workspace.anchor_reobserve_count <= 1 &&
      neighborhood_anchor_reobserve_count <= 2 &&
      workspace.below_anchor_count == 0 &&
      workspace.upper_band_count >=
          std::max(2, workspace.anchor_reobserve_count + 1) &&
      workspace.min_upper_band_z >= support_anchor + upper_height_threshold;
  if (reject_stale_anchor) {
    return AnchorValidityDecision::kInvalidStale;
  }

  const bool reject_wall_only_anchor =
      workspace.below_anchor_count >= 3 &&
      workspace.below_anchor_count > workspace.anchor_reobserve_count * 2 &&
      (workspace.upper_band_count == 0 ||
       workspace.min_upper_band_z < support_anchor + upper_height_threshold +
                                        support_anchor_reobserve_tolerance);
  if (reject_wall_only_anchor) {
    return AnchorValidityDecision::kInvalidWallOnly;
  }

  return AnchorValidityDecision::kValid;
}

std::vector<int> BuildLocalProfilesAndSeedCandidates(
    const ProcessedFrame &frame, const FrameObservability &observability,
    const LocalTerrainMap &map, const TerrainLayers &layers,
    const Config &config,
    const std::vector<std::vector<size_t>> &sample_indices_by_cell,
    std::vector<CellWorkspace> &cell_workspaces, FrontendOutput &output) {
  std::vector<int> suspicious_cells;
  suspicious_cells.reserve(cell_workspaces.size() / 8U + 1U);
  const float sector_size = 2.0f * static_cast<float>(M_PI) /
                            static_cast<float>(observability.sectors.size());
  const float yaw = YawFromQuaternion(frame.base_pose_in_odom.orientation);
  const float suspicious_vertical_span = config.geometry.max_step_up * 0.75f;

  for (int cell = 0; cell < map.size(); ++cell) {
    const auto &sample_indices =
        sample_indices_by_cell[static_cast<size_t>(cell)];
    if (sample_indices.empty()) {
      continue;
    }

    auto &workspace = cell_workspaces[static_cast<size_t>(cell)];
    const bool has_local_history_anchor =
        workspace.local_history_anchor_valid != 0U;
    float support_anchor = workspace.support_anchor_candidate;
    workspace.anchor_validity =
        std::isfinite(support_anchor)
            ? AnchorValidityDecision::kValid
            : AnchorValidityDecision::kInvalidInsufficientSupport;
    if (workspace.anchor_validity == AnchorValidityDecision::kValid) {
      const int neighborhood_anchor_reobserve_count =
          CountNeighborhoodAnchorReobserveCount(cell, map, cell_workspaces);
      workspace.anchor_validity = ClassifyAnchorValidity(
          workspace, neighborhood_anchor_reobserve_count, support_anchor,
          config.geometry.upper_min_height_above_support,
          config.geometry.support_anchor_reobserve_tolerance);
      if (workspace.anchor_validity != AnchorValidityDecision::kValid) {
        output.support_anchor_used[static_cast<size_t>(cell)] =
            std::numeric_limits<float>::quiet_NaN();
      }
    }
    const bool has_support_anchor =
        workspace.anchor_validity == AnchorValidityDecision::kValid;

    if (has_support_anchor) {
      output.support_anchor_used[static_cast<size_t>(cell)] = support_anchor;
    }

    for (const size_t sample_index : sample_indices) {
      const auto &sample = frame.odom_samples[sample_index];
      const bool is_leak =
          has_support_anchor &&
          sample.point_in_odom.z <
              support_anchor - config.geometry.sub_support_leak_tolerance;
      if (is_leak) {
        if (workspace.sub_support_leak_count <
            std::numeric_limits<uint16_t>::max()) {
          ++workspace.sub_support_leak_count;
        }
        continue;
      }
      if (workspace.anchor_validity == AnchorValidityDecision::kInvalidStale &&
          sample.point_in_odom.z <=
              support_anchor +
                  config.geometry.support_anchor_reobserve_tolerance) {
        continue;
      }
      workspace.stats.min_z =
          std::min(workspace.stats.min_z, sample.point_in_odom.z);
      workspace.stats.max_z =
          std::max(workspace.stats.max_z, sample.point_in_odom.z);
      ++workspace.stats.count;
    }
    output.sub_support_leak_count[static_cast<size_t>(cell)] =
        workspace.sub_support_leak_count;
    if (workspace.stats.count == 0) {
      continue;
    }
    workspace.has_stats = true;

    const Eigen::Vector2f center = map.indexToOdom(cell);
    const float representative_base_angle = NormalizeAngle(
        std::atan2(center.y() - frame.base_pose_in_odom.position.y(),
                   center.x() - frame.base_pose_in_odom.position.x()) -
        yaw);
    const int sector =
        std::clamp(static_cast<int>(std::floor(
                       (representative_base_angle + static_cast<float>(M_PI)) /
                       sector_size)),
                   0, static_cast<int>(observability.sectors.size()) - 1);
    const auto sector_state = observability.sectors[sector].state;
    const float vertical_span = workspace.stats.max_z - workspace.stats.min_z;
    const float coverage = observability.sectors[sector].coverage_confidence;

    workspace.support_ref =
        has_local_history_anchor && has_support_anchor
            ? layers.support_height[static_cast<size_t>(cell)]
            : workspace.stats.min_z;
    workspace.stale_lower_anchor_mix =
        has_support_anchor &&
        workspace.upper_band_count >=
            std::max(2, workspace.anchor_reobserve_count) &&
        workspace.min_upper_band_z >=
            support_anchor + config.geometry.upper_min_height_above_support;

    if (sector_state != ObservabilityState::kMissingByDropout) {
      output.support_candidates.push_back(SupportCandidate{
          cell, workspace.stats.min_z, std::clamp(coverage, 0.0f, 1.0f)});
    }
    if (vertical_span > suspicious_vertical_span) {
      output.obstacle_suspicious[static_cast<size_t>(cell)] = 1U;
      suspicious_cells.push_back(cell);
    } else if (sector_state == ObservabilityState::kPartiallyObserved) {
      output.ambiguous_candidates.push_back(
          AmbiguousCandidate{cell, workspace.stats.min_z});
    }
  }

  return suspicious_cells;
}

void MarkUpperSupportCells(
    const ProcessedFrame &frame, const LocalTerrainMap &map,
    const Config &config,
    const std::vector<std::vector<size_t>> &sample_indices_by_cell,
    const std::vector<CellWorkspace> &cell_workspaces, FrontendOutput &output) {
  for (int cell = 0; cell < map.size(); ++cell) {
    const auto &workspace = cell_workspaces[static_cast<size_t>(cell)];
    if (!std::isfinite(workspace.support_ref)) {
      continue;
    }

    const float support_anchor =
        output.support_anchor_used[static_cast<size_t>(cell)];
    const bool has_support_anchor = std::isfinite(support_anchor);
    for (const size_t sample_index :
         sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (has_support_anchor &&
          sample.point_in_odom.z <
              support_anchor - config.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (sample.point_in_odom.z >=
          workspace.support_ref +
              config.geometry.upper_min_height_above_support) {
        output.raw_upper_support_cell[static_cast<size_t>(cell)] = 1U;
        output.explanation_adjusted_upper_support_cell[static_cast<size_t>(
            cell)] = 1U;
        break;
      }
    }
  }
}

void BuildAdjacencyContexts(const LocalTerrainMap &map, const Config &config,
                            std::vector<CellWorkspace> &cell_workspaces) {
  for (int cell = 0; cell < map.size(); ++cell) {
    auto &workspace = cell_workspaces[static_cast<size_t>(cell)];
    if (!std::isfinite(workspace.support_ref)) {
      continue;
    }
    workspace.ascending_stair_support_count = CountAscendingNeighborSupportRefs(
        cell, map, cell_workspaces, workspace.support_ref,
        config.geometry.upper_min_height_above_support,
        config.geometry.max_step_up);
  }
}

void BuildExplanationInputs(
    const ProcessedFrame &frame, const LocalTerrainMap &map,
    const TerrainLayers &layers, const Config &config,
    const std::vector<std::vector<size_t>> &sample_indices_by_cell,
    std::vector<CellWorkspace> &cell_workspaces, FrontendOutput &output) {
  for (int cell = 0; cell < map.size(); ++cell) {
    auto &workspace = cell_workspaces[static_cast<size_t>(cell)];
    if (!workspace.has_stats ||
        output.raw_upper_support_cell[static_cast<size_t>(cell)] == 0U) {
      continue;
    }

    const float support_anchor =
        output.support_anchor_used[static_cast<size_t>(cell)];
    const bool has_support_anchor = std::isfinite(support_anchor);
    if (!has_support_anchor) {
      continue;
    }

    const float upper_layer_consensus_candidate_z =
        ResolveNeighborUpperLayerConsensusHeight(
            cell, map, cell_workspaces,
            config.geometry.upper_min_height_above_support,
            config.geometry.max_step_up);
    const float anchored_support_candidate_z =
        ResolveNeighborSupportConsensusHeight(cell, map, layers, config);
    const float effective_support_candidate_z =
        std::isfinite(upper_layer_consensus_candidate_z)
            ? upper_layer_consensus_candidate_z
            : anchored_support_candidate_z;
    if (!std::isfinite(effective_support_candidate_z)) {
      continue;
    }

    int candidate_reobserve_count = 0;
    for (const size_t sample_index :
         sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (sample.point_in_odom.z <
          support_anchor - config.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (std::abs(sample.point_in_odom.z - effective_support_candidate_z) <=
          config.geometry.support_anchor_reobserve_tolerance) {
        ++candidate_reobserve_count;
      }
    }

    const bool using_upper_layer_consensus_candidate =
        std::isfinite(upper_layer_consensus_candidate_z);
    const int upper_layer_neighbor_match_count =
        using_upper_layer_consensus_candidate
            ? CountNeighborUpperLayersAlignedToHeight(
                  cell, map, cell_workspaces, effective_support_candidate_z,
                  config.geometry.support_anchor_reobserve_tolerance,
                  config.geometry.upper_min_height_above_support,
                  config.geometry.max_step_up)
            : CountNeighborAnchorsAlignedToHeight(
                  cell, map, layers, config, effective_support_candidate_z,
                  config.geometry.support_anchor_reobserve_tolerance);
    const bool use_elevated_effective_support_ref =
        ShouldUseElevatedEffectiveSupportRef(
            has_support_anchor,
            support_anchor - frame.base_pose_in_odom.position.z(),
            workspace.support_ref - frame.base_pose_in_odom.position.z(),
            effective_support_candidate_z -
                frame.base_pose_in_odom.position.z(),
            effective_support_candidate_z - workspace.support_ref,
            workspace.upper_band_count, workspace.anchor_reobserve_count,
            candidate_reobserve_count, upper_layer_neighbor_match_count,
            workspace.ascending_stair_support_count,
            std::max(1, config.geometry.min_neighbor_upper_support_cells),
            config.geometry.support_anchor_reobserve_tolerance,
            config.geometry.max_step_up, workspace.sub_support_leak_count,
            workspace.stale_lower_anchor_mix);
    if (!use_elevated_effective_support_ref) {
      continue;
    }

    bool has_elevated_upper_support = false;
    for (const size_t sample_index :
         sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.odom_samples[sample_index];
      if (sample.point_in_odom.z <
          support_anchor - config.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (sample.point_in_odom.z >=
          effective_support_candidate_z +
              config.geometry.upper_min_height_above_support) {
        has_elevated_upper_support = true;
        break;
      }
    }
    output.explanation_adjusted_upper_support_cell[static_cast<size_t>(cell)] =
        has_elevated_upper_support ? 1U : 0U;
  }
}

void FinalizeUpperSupportCells(FrontendOutput &output) {
  // `upper_support_cell` remains a compatibility alias for the
  // explanation-adjusted confirmation mask. Raw local upper-band facts stay
  // available via `raw_upper_support_cell`.
  output.upper_support_cell = output.explanation_adjusted_upper_support_cell;
}

void EmitCandidates(const ProcessedFrame &frame, const LocalTerrainMap &map,
                    const TerrainLayers &layers, const Config &config,
                    const std::vector<CellWorkspace> &cell_workspaces,
                    const std::vector<int> &suspicious_cells,
                    FrontendOutput &output) {
  const int min_neighbor_upper_support_cells =
      std::max(1, config.geometry.min_neighbor_upper_support_cells);
  for (const int cell : suspicious_cells) {
    const auto &workspace = cell_workspaces[static_cast<size_t>(cell)];
    if (!workspace.has_stats) {
      continue;
    }

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
        const bool is_upper_support =
            output.explanation_adjusted_upper_support_cell[static_cast<size_t>(
                neighbor)] != 0U;
        support_count += is_upper_support ? 1 : 0;
        if (IsValidSupportAnchorCell(layers, neighbor, config) &&
            std::abs(layers.support_height[static_cast<size_t>(neighbor)] -
                     workspace.stats.max_z) <=
                config.geometry.support_anchor_reobserve_tolerance) {
          ++aligned_neighbor_support_count;
        }
      }
    }
    output.neighbor_upper_support_count[static_cast<size_t>(cell)] =
        static_cast<int8_t>(std::clamp(support_count, 0, 9));
    if (support_count >= min_neighbor_upper_support_cells &&
        aligned_neighbor_support_count < min_neighbor_upper_support_cells) {
      const float support_anchor =
          output.support_anchor_used[static_cast<size_t>(cell)];
      const bool has_support_anchor = std::isfinite(support_anchor);
      const float vertical_span = workspace.stats.max_z - workspace.stats.min_z;
      const float relative_support_ref =
          workspace.support_ref - frame.base_pose_in_odom.position.z();
      const float relative_upper_z =
          workspace.stats.max_z - frame.base_pose_in_odom.position.z();
      const ExplanationDecision explanation_decision =
          ClassifyExplanationDecision(
              has_support_anchor,
              support_anchor - frame.base_pose_in_odom.position.z(),
              relative_support_ref, relative_upper_z, vertical_span,
              support_count, workspace.ascending_stair_support_count,
              aligned_neighbor_support_count, min_neighbor_upper_support_cells,
              config.geometry.max_step_down,
              config.geometry.upper_min_height_above_support,
              config.geometry.max_step_up, workspace.sub_support_leak_count,
              workspace.stale_lower_anchor_mix);
      if (explanation_decision != ExplanationDecision::kNone) {
        output
            .obstacle_rejected_by_neighbor_support[static_cast<size_t>(cell)] =
            1U;
        continue;
      }

      const float wall_like_gain_scale =
          workspace.sub_support_leak_count == 0U
              ? ComputeWallLikeObstacleGainScale(workspace.stats, vertical_span,
                                                 relative_upper_z,
                                                 support_count)
              : 1.0f;
      output.obstacle_candidate_cell[static_cast<size_t>(cell)] = 1U;
      output.obstacle_candidates.push_back(ObstacleCandidate{
          cell, workspace.stats.max_z, std::clamp(vertical_span, 0.0f, 1.0f),
          wall_like_gain_scale});
    } else {
      output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(cell)] =
          1U;
    }
  }
}

} // namespace

FrontendOutput PolarFrontend::run(const ProcessedFrame &frame,
                                  const FrameObservability &observability,
                                  const LocalTerrainMap &map) const {
  FrontendOutput output;
  output.support_anchor_used.assign(static_cast<size_t>(map.size()),
                                    std::numeric_limits<float>::quiet_NaN());
  output.sub_support_leak_count.assign(static_cast<size_t>(map.size()), 0U);
  output.raw_upper_support_cell.assign(static_cast<size_t>(map.size()), 0U);
  output.explanation_adjusted_upper_support_cell.assign(
      static_cast<size_t>(map.size()), 0U);
  output.upper_support_cell.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_suspicious.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_candidate_cell.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_rejected_by_neighbor_support.assign(
      static_cast<size_t>(map.size()), 0U);
  output.neighbor_upper_support_count.assign(static_cast<size_t>(map.size()),
                                             0);
  const auto sample_indices_by_cell = GroupSampleIndicesByCell(frame, map);
  const int active_cell_count = CountActiveCells(sample_indices_by_cell);
  output.support_candidates.reserve(static_cast<size_t>(active_cell_count));
  output.obstacle_candidates.reserve(static_cast<size_t>(active_cell_count));
  output.ambiguous_candidates.reserve(static_cast<size_t>(active_cell_count));

  std::vector<CellWorkspace> cell_workspaces(static_cast<size_t>(map.size()));
  const auto &layers = map.layers();

  ResolveAnchors(frame, map, layers, config_, sample_indices_by_cell,
                 cell_workspaces);
  const auto suspicious_cells = BuildLocalProfilesAndSeedCandidates(
      frame, observability, map, layers, config_, sample_indices_by_cell,
      cell_workspaces, output);
  MarkUpperSupportCells(frame, map, config_, sample_indices_by_cell,
                        cell_workspaces, output);
  BuildAdjacencyContexts(map, config_, cell_workspaces);
  BuildExplanationInputs(frame, map, layers, config_, sample_indices_by_cell,
                         cell_workspaces, output);
  FinalizeUpperSupportCells(output);
  EmitCandidates(frame, map, layers, config_, cell_workspaces, suspicious_cells,
                 output);
  return output;
}

} // namespace passable_area::core
