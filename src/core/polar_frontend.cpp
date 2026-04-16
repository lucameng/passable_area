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

struct ResolvedSupportAnchor {
  float z = std::numeric_limits<float>::quiet_NaN();
  SupportAnchorOrigin origin = SupportAnchorOrigin::kNone;
};

enum class AnchorValidityDecision : uint8_t {
  kInvalidInsufficientSupport = 0U,
  kValid = 1U,
  kInvalidStale = 2U,
  kInvalidWallOnly = 3U,
};

struct CellWorkspace {
  CellStats raw_stats;
  CellStats filtered_stats;
  CellStats trigger_stats;
  float support_anchor_candidate = std::numeric_limits<float>::quiet_NaN();
  float support_ref = std::numeric_limits<float>::quiet_NaN();
  float min_upper_band_z = std::numeric_limits<float>::infinity();
  SupportAnchorOrigin support_anchor_origin = SupportAnchorOrigin::kNone;
  SupportAnchorAuthority support_anchor_authority =
      SupportAnchorAuthority::kInvalid;
  int anchor_reobserve_count = 0;
  int below_anchor_count = 0;
  int upper_band_count = 0;
  int ascending_stair_support_count = 0;
  uint16_t anchor_below_observation_count = 0U;
  uint16_t sub_support_leak_count = 0U;
  uint16_t stale_anchor_residual_filtered_count = 0U;
  uint8_t local_history_anchor_valid = 0U;
  AnchorValidityDecision anchor_validity =
      AnchorValidityDecision::kInvalidInsufficientSupport;
  bool anchor_leak_suppression_enabled = false;
  bool has_stats = false;
  bool stale_lower_anchor_mix = false;
};

struct FacadeEvidence {
  bool lower_upper_coexisting = false;
  bool upper_edge_aligned_with_supported_neighbors = false;

  bool any() const {
    return lower_upper_coexisting ||
           upper_edge_aligned_with_supported_neighbors;
  }
};

// 解决代表角落在 -pi/pi 边界附近时扇区索引跳变的 case。
// 做法是把角度统一规约到 [-pi, pi]，避免 observability 分桶不稳定。
float NormalizeAngle(float angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

void AccumulateCellStats(CellStats &stats, float z) {
  stats.min_z = std::min(stats.min_z, z);
  stats.max_z = std::max(stats.max_z, z);
  ++stats.count;
}

void SaturatingIncrement(uint16_t &value) {
  if (value < std::numeric_limits<uint16_t>::max()) {
    ++value;
  }
}

// 解决“地图里有 support_height，但这格当前并不该再被当可靠锚点”的 case。
// 这里把 finite height、confidence、support_state、age
// 几个条件统一成一个可复用的锚点有效性判断。
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

// 解决本 cell 没有可用历史支撑，但 3x3 邻域里其实有稳定地面的 case。
// 做法是用邻域有效 support 的中位数借锚，但如果本 cell
// 自己已经有效，就不重复借邻居。
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

SupportAnchorOrigin ResolveSupportAnchorOrigin(int cell,
                                               const LocalTerrainMap &map,
                                               const TerrainLayers &layers,
                                               const Config &config) {
  if (IsValidSupportAnchorCell(layers, cell, config)) {
    return SupportAnchorOrigin::kLocalSupport;
  }
  return std::isfinite(ResolveNeighborSupportAnchor(cell, map, layers, config))
             ? SupportAnchorOrigin::kBorrowedNeighbor
             : SupportAnchorOrigin::kNone;
}

ResolvedSupportAnchor ResolveSupportAnchorCandidate(int cell,
                                                    const LocalTerrainMap &map,
                                                    const TerrainLayers &layers,
                                                    const Config &config,
                                                    float raw_min_z) {
  if (!std::isfinite(raw_min_z)) {
    return {};
  }

  if (IsValidSupportAnchorCell(layers, cell, config)) {
    const float local_anchor = layers.support_height[static_cast<size_t>(cell)];
    return {local_anchor, SupportAnchorOrigin::kLocalSupport};
  }

  const float neighbor_anchor =
      ResolveNeighborSupportAnchor(cell, map, layers, config);
  if (std::isfinite(neighbor_anchor) &&
      neighbor_anchor >=
          raw_min_z - config.geometry.sub_support_leak_tolerance) {
    return {neighbor_anchor, SupportAnchorOrigin::kBorrowedNeighbor};
  }

  return {};
}

// 解决 explanation 需要一个“邻域稳定支撑共识高度”来判断 layered-ground 的
// case。 做法是只看 8 邻域的有效
// support，高度用中位数聚合，避免被单侧异常邻居带偏。
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

// 解决 explanation 需要知道“目标高度是否真有邻域支撑对齐”而不是单点偶然命中的
// case。 做法是在 3x3 里数与目标高度接近的有效 support anchor，给
// layered-ground / ground-mix 做约束。
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

// 解决某些规则只需要知道“周围到底有没有足够多可借锚邻居”的 case。
// 这里不看高度对齐，只统计 3x3 里的有效锚点数量，保持语义和对齐计数分开。
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

// 解决机器人下方 layered structure 中，邻居上层带比本 cell 低层 support
// 更可信的 case。 做法是从邻居的 upper band
// 中提取一个上层共识高度，只接受高度差落在合法 step_up 区间的邻居。
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
          neighbor_workspace.anchor_below_observation_count != 0U) {
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

// 解决“上层共识高度是否真的在邻域里形成连续结构”这个 case。做法是只统计 upper
// band 合法且与目标高度对齐的邻居，避免把稀疏高点误当成稳定上层。
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
          neighbor_workspace.anchor_below_observation_count != 0U) {
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

// 解决楼梯上行趋势和机器人下方 layered-ground 容易混淆的 case。
// 做法是统计邻域 support_ref 是否持续抬升；一旦形成上行趋势，后面 explanation
// 就不能随意抬 ref 去吃掉 obstacle。
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

// 解决 suspicious cell 虽有邻域 upper
// support，但其实更像楼梯混层/脚下低层地面的 case。 做法是把 reject 侧的
// below-robot stair mix、downstairs ground mix、upstairs ground mix
// 统一收敛成 explanation reject 决策。
FrontendExplanationDecision ClassifyExplanationRejectDecision(
    bool has_support_anchor, float relative_support_anchor,
    float relative_support_ref, float relative_upper_z, float vertical_span,
    int support_count, int ascending_stair_support_count,
    int aligned_neighbor_support_count, int min_neighbor_upper_support_cells,
    float max_step_down, float upper_height_threshold, float max_step_up,
    uint16_t anchor_below_observation_count, bool stale_lower_anchor_mix) {
  if (std::isfinite(relative_support_ref) &&
      relative_support_ref <= -max_step_down &&
      relative_upper_z <= -upper_height_threshold && support_count >= 3 &&
      (anchor_below_observation_count > 0U || stale_lower_anchor_mix)) {
    return FrontendExplanationDecision::kBelowRobotStairMix;
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
      anchor_below_observation_count == 0U && !stale_lower_anchor_mix) {
    return FrontendExplanationDecision::kBelowRobotGroundLayerMix;
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
      aligned_neighbor_support_count == 0 &&
      anchor_below_observation_count == 0U) {
    return FrontendExplanationDecision::kBelowRobotUpstairGroundMix;
  }

  return FrontendExplanationDecision::kNone;
}

// 解决 keep-side 仍然靠“没 reject 就通过”的 case。做法是把 keep verdict
// 单独显式化，只有 facade evidence 命中时才返回 KeepAsObstacle。
FrontendExplanationDecision
ClassifyExplanationKeepDecision(const FacadeEvidence &facade_evidence) {
  return facade_evidence.any() ? FrontendExplanationDecision::kKeepAsObstacle
                               : FrontendExplanationDecision::kNone;
}

FacadeEvidence BuildFacadeEvidence(
    int cell, const ProcessedFrame &frame, const LocalTerrainMap &map,
    const TerrainLayers &layers, const Config &config,
    const std::vector<std::vector<size_t>> &sample_indices_by_cell,
    const std::vector<CellWorkspace> &cell_workspaces,
    int aligned_neighbor_support_count) {
  FacadeEvidence evidence;
  const auto &workspace = cell_workspaces[static_cast<size_t>(cell)];
  const float support_anchor = workspace.support_anchor_candidate;
  const bool has_support_anchor =
      std::isfinite(support_anchor) &&
      workspace.anchor_validity == AnchorValidityDecision::kValid;
  const float lower_sample_ceiling =
      has_support_anchor
          ? support_anchor + config.geometry.support_anchor_reobserve_tolerance
          : workspace.support_ref +
                config.geometry.support_anchor_reobserve_tolerance;
  const float upper_band_floor =
      workspace.support_ref + config.geometry.upper_min_height_above_support;
  bool has_lower_structure_sample = false;
  bool has_upper_return = false;
  float min_upper_return_z = std::numeric_limits<float>::infinity();
  if (std::isfinite(workspace.support_ref)) {
    for (const size_t sample_index :
         sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.map_samples[sample_index];
      if (has_support_anchor &&
          sample.point_in_map.z <
              support_anchor - config.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (sample.point_in_map.z <= lower_sample_ceiling) {
        has_lower_structure_sample = true;
      }
      if (sample.point_in_map.z >= upper_band_floor) {
        has_upper_return = true;
        min_upper_return_z =
            std::min(min_upper_return_z, sample.point_in_map.z);
      }
    }
    evidence.lower_upper_coexisting =
        has_lower_structure_sample && has_upper_return;
  }

  if (std::isfinite(workspace.support_ref) && has_upper_return) {
    const int upper_layer_neighbor_match_count =
        CountNeighborUpperLayersAlignedToHeight(
            cell, map, cell_workspaces, min_upper_return_z,
            config.geometry.support_anchor_reobserve_tolerance,
            config.geometry.upper_min_height_above_support,
            config.geometry.max_step_up);
    evidence.upper_edge_aligned_with_supported_neighbors =
        upper_layer_neighbor_match_count >=
            std::max(1, config.geometry.min_neighbor_upper_support_cells) &&
        aligned_neighbor_support_count >=
            std::max(1, config.geometry.min_neighbor_upper_support_cells) &&
        min_upper_return_z >= upper_band_floor;
  }
  return evidence;
}

// 解决 support_ref 被脚下更低一层拖低，导致本来可解释的上层踏面被误当 obstacle
// 的 case。 做法是在 explanation
// 内部临时评估是否允许抬高参考层，但这个量只服务当前 cell
// 的解释，不进入主链状态。
bool ShouldUseElevatedEffectiveSupportRef(
    bool has_support_anchor, float relative_support_anchor,
    float relative_support_ref, float relative_effective_support_candidate_z,
    float candidate_gap_above_support_ref, int upper_band_count,
    int anchor_reobserve_count, int candidate_reobserve_count,
    int upper_layer_neighbor_match_count, int ascending_stair_support_count,
    int min_neighbor_upper_support_cells,
    float support_anchor_reobserve_tolerance, float max_step_up,
    uint16_t anchor_below_observation_count, bool stale_lower_anchor_mix) {
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
         anchor_below_observation_count == 0U && !stale_lower_anchor_mix;
}

// 解决后续每个 phase 都重复扫全帧点云找 cell 样本的 case。
// 做法是先把样本按 cell 分组，后面的 anchor/profile/explanation
// 都只消费这个索引表。
std::vector<std::vector<size_t>>
GroupSampleIndicesByCell(const ProcessedFrame &frame,
                         const LocalTerrainMap &map) {
  std::vector<std::vector<size_t>> sample_indices_by_cell(
      static_cast<size_t>(map.size()));
  for (size_t sample_index = 0; sample_index < frame.map_samples.size();
       ++sample_index) {
    const auto &sample = frame.map_samples[sample_index];
    int cell = -1;
    if (!map.mapToIndex(sample.point_in_map.x, sample.point_in_map.y,
                         cell)) {
      continue;
    }
    sample_indices_by_cell[static_cast<size_t>(cell)].push_back(sample_index);
  }
  return sample_indices_by_cell;
}

// 解决 reserve 容量只能靠整张地图大小粗估，导致输出 vector 反复扩容的 case。
// 做法是先统计当前真正有样本的 active cells，后续输出容器按这个量预分配。
int CountActiveCells(
    const std::vector<std::vector<size_t>> &sample_indices_by_cell) {
  return static_cast<int>(std::count_if(
      sample_indices_by_cell.begin(), sample_indices_by_cell.end(),
      [](const auto &sample_indices) { return !sample_indices.empty(); }));
}

// 解决每个 cell 的 anchor 来源、借锚和 upper-band 初始分层散落在多个 pass 里的
// case。 做法是先统一解析
// anchor，并顺手统计低于锚点、重观测锚点、上层带这些局部事实，供后续 phase
// 复用。
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
      AccumulateCellStats(workspace.raw_stats,
                          frame.map_samples[sample_index].point_in_map.z);
    }

    const bool has_local_history_anchor =
        IsValidSupportAnchorCell(layers, cell, config);
    workspace.local_history_anchor_valid = has_local_history_anchor ? 1U : 0U;
    workspace.support_anchor_origin =
        ResolveSupportAnchorOrigin(cell, map, layers, config);

    const ResolvedSupportAnchor resolved_anchor = ResolveSupportAnchorCandidate(
        cell, map, layers, config, workspace.raw_stats.min_z);
    if (!std::isfinite(resolved_anchor.z)) {
      workspace.anchor_validity =
          AnchorValidityDecision::kInvalidInsufficientSupport;
      continue;
    }

    workspace.support_anchor_candidate = resolved_anchor.z;
    workspace.anchor_validity = AnchorValidityDecision::kValid;
    for (const size_t sample_index : sample_indices) {
      const auto &sample = frame.map_samples[sample_index];
      if (sample.point_in_map.z <
          resolved_anchor.z - config.geometry.sub_support_leak_tolerance) {
        ++workspace.below_anchor_count;
        SaturatingIncrement(workspace.anchor_below_observation_count);
        continue;
      }
      if (sample.point_in_map.z <=
          resolved_anchor.z +
              config.geometry.support_anchor_reobserve_tolerance) {
        ++workspace.anchor_reobserve_count;
      }
      if (sample.point_in_map.z >=
          resolved_anchor.z + config.geometry.upper_min_height_above_support) {
        ++workspace.upper_band_count;
        workspace.min_upper_band_z =
            std::min(workspace.min_upper_band_z, sample.point_in_map.z);
      }
    }
  }
}

// 解决 stale-anchor 判断只看本 cell 容易把局部观测稀疏误判成“旧锚点失效”的
// case。 做法是把 3x3 邻域的 anchor 重观测数加起来，给 anchor validity
// 一个更稳的上下文。
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

// 解决历史锚点虽然存在，但实际上已经 stale 或
// wall-only，继续参与解释会把当前帧带偏的 case。 做法是把锚点统一归类为 Valid /
// InvalidStale / InvalidWallOnly /
// InvalidInsufficientSupport，后面只消费枚举结果。
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

// 判定这个 anchor 是否有 trigger 前的 hard leak suppression 权限。
// borrowed / weak anchor 只能作为 explanation 参考，不能提前删 lower samples；
// 只有足够可靠的 local/history anchor 才能做 hard suppression。
// 这样既防 borrowed anchor 越权压掉 lower structure，
// 也保留 strong local anchor 应有的 suppression 能力。
SupportAnchorAuthority ClassifyAnchorAuthority(const CellWorkspace &workspace) {
  if (!std::isfinite(workspace.support_anchor_candidate) ||
      workspace.anchor_validity != AnchorValidityDecision::kValid) {
    return SupportAnchorAuthority::kInvalid;
  }
  if (workspace.support_anchor_origin == SupportAnchorOrigin::kLocalSupport &&
      workspace.local_history_anchor_valid != 0U) {
    return SupportAnchorAuthority::kLeakEligible;
  }
  return SupportAnchorAuthority::kExplanationOnly;
}

// 解决 local trigger、support candidate 和 leak 过滤各自重复扫样本的 case。
// 做法是基于统一 anchor 过滤后的样本一次性建立 local profile，再用
// vertical_span 标记 local trigger。 这里不做连续支撑解释，也不做 candidate
// 否决。
std::vector<int> BuildLocalProfilesAndMarkTriggers(
    const ProcessedFrame &frame, const FrameObservability &observability,
    const LocalTerrainMap &map, const TerrainLayers &layers,
    const Config &config,
    const std::vector<std::vector<size_t>> &sample_indices_by_cell,
    std::vector<CellWorkspace> &cell_workspaces, FrontendOutput &output) {
  std::vector<int> locally_triggered_cells;
  locally_triggered_cells.reserve(cell_workspaces.size() / 8U + 1U);
  const float sector_size = 2.0f * static_cast<float>(M_PI) /
                            static_cast<float>(observability.sectors.size());
  const float yaw = YawFromQuaternion(frame.base_pose_in_map.orientation);
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
    }
    const bool has_support_anchor =
        workspace.anchor_validity == AnchorValidityDecision::kValid;
    workspace.support_anchor_authority = ClassifyAnchorAuthority(workspace);
    workspace.anchor_leak_suppression_enabled =
        workspace.support_anchor_authority ==
        SupportAnchorAuthority::kLeakEligible;
    if (has_support_anchor && !workspace.anchor_leak_suppression_enabled) {
      workspace.support_anchor_authority =
          SupportAnchorAuthority::kExplanationOnly;
    }

    if (has_support_anchor) {
      output.support_anchor_used[static_cast<size_t>(cell)] = support_anchor;
    }
    output.support_anchor_origin[static_cast<size_t>(cell)] =
        static_cast<uint8_t>(workspace.support_anchor_origin);
    output.support_anchor_authority[static_cast<size_t>(cell)] =
        static_cast<uint8_t>(workspace.support_anchor_authority);
    output.anchor_leak_suppression_enabled[static_cast<size_t>(cell)] =
        workspace.anchor_leak_suppression_enabled ? 1U : 0U;
    output.raw_sample_min_z[static_cast<size_t>(cell)] =
        workspace.raw_stats.count > 0 ? workspace.raw_stats.min_z
                                      : std::numeric_limits<float>::quiet_NaN();
    output.raw_sample_max_z[static_cast<size_t>(cell)] =
        workspace.raw_stats.count > 0 ? workspace.raw_stats.max_z
                                      : std::numeric_limits<float>::quiet_NaN();
    output.raw_sample_count[static_cast<size_t>(cell)] = static_cast<uint16_t>(
        std::clamp(workspace.raw_stats.count, 0,
                   static_cast<int>(std::numeric_limits<uint16_t>::max())));

    for (const size_t sample_index : sample_indices) {
      const auto &sample = frame.map_samples[sample_index];
      const bool is_leak =
          workspace.anchor_leak_suppression_enabled &&
          sample.point_in_map.z <
              support_anchor - config.geometry.sub_support_leak_tolerance;
      if (is_leak) {
        SaturatingIncrement(workspace.sub_support_leak_count);
        continue;
      }
      if (workspace.anchor_validity == AnchorValidityDecision::kInvalidStale &&
          sample.point_in_map.z <=
              support_anchor +
                  config.geometry.support_anchor_reobserve_tolerance) {
        SaturatingIncrement(workspace.stale_anchor_residual_filtered_count);
        continue;
      }
      AccumulateCellStats(workspace.filtered_stats, sample.point_in_map.z);
    }
    workspace.trigger_stats = workspace.filtered_stats;
    output.sub_support_leak_count[static_cast<size_t>(cell)] =
        workspace.sub_support_leak_count;
    output.anchor_below_observation_count[static_cast<size_t>(cell)] =
        workspace.anchor_below_observation_count;
    output.stale_anchor_residual_filtered_count[static_cast<size_t>(cell)] =
        workspace.stale_anchor_residual_filtered_count;
    output.filtered_sample_min_z[static_cast<size_t>(cell)] =
        workspace.filtered_stats.count > 0
            ? workspace.filtered_stats.min_z
            : std::numeric_limits<float>::quiet_NaN();
    output.filtered_sample_max_z[static_cast<size_t>(cell)] =
        workspace.filtered_stats.count > 0
            ? workspace.filtered_stats.max_z
            : std::numeric_limits<float>::quiet_NaN();
    output.filtered_sample_count[static_cast<size_t>(cell)] =
        static_cast<uint16_t>(
            std::clamp(workspace.filtered_stats.count, 0,
                       static_cast<int>(std::numeric_limits<uint16_t>::max())));
    if (workspace.trigger_stats.count == 0) {
      continue;
    }
    workspace.has_stats = true;

    const Eigen::Vector2f center = map.indexToMap(cell);
    const float representative_base_angle = NormalizeAngle(
        std::atan2(center.y() - frame.base_pose_in_map.position.y(),
                   center.x() - frame.base_pose_in_map.position.x()) -
        yaw);
    const int sector =
        std::clamp(static_cast<int>(std::floor(
                       (representative_base_angle + static_cast<float>(M_PI)) /
                       sector_size)),
                   0, static_cast<int>(observability.sectors.size()) - 1);
    const auto sector_state = observability.sectors[sector].state;
    const CellStats &trigger_stats = workspace.trigger_stats;
    const float vertical_span = trigger_stats.max_z - trigger_stats.min_z;
    const float coverage = observability.sectors[sector].coverage_confidence;

    workspace.support_ref =
        has_local_history_anchor && has_support_anchor
            ? layers.support_height[static_cast<size_t>(cell)]
            : trigger_stats.min_z;
    workspace.stale_lower_anchor_mix =
        has_support_anchor &&
        workspace.upper_band_count >=
            std::max(2, workspace.anchor_reobserve_count) &&
        workspace.min_upper_band_z >=
            support_anchor + config.geometry.upper_min_height_above_support;

    if (sector_state != ObservabilityState::kMissingByDropout) {
      output.support_candidates.push_back(SupportCandidate{
          cell, trigger_stats.min_z, std::clamp(coverage, 0.0f, 1.0f)});
    }
    if (vertical_span > suspicious_vertical_span) {
      output.obstacle_local_triggered[static_cast<size_t>(cell)] = 1U;
      output.obstacle_suspicious[static_cast<size_t>(cell)] = 1U;
      locally_triggered_cells.push_back(cell);
    } else if (sector_state == ObservabilityState::kPartiallyObserved) {
      output.ambiguous_candidates.push_back(
          AmbiguousCandidate{cell, trigger_stats.min_z});
    }
  }

  return locally_triggered_cells;
}

// 解决“当前格是否存在 upper-support 事实”被后续 confirm 与 explanation 混用的
// case。 做法是先只按本 cell 样本和 support_ref 标出 raw upper-support，再把
// adjusted mask 初始化成 raw，后面只允许 explanation 单向改写。
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
      const auto &sample = frame.map_samples[sample_index];
      if (has_support_anchor &&
          sample.point_in_map.z <
              support_anchor - config.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (sample.point_in_map.z >=
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

// 解决 explanation 需要知道邻域是上行楼梯趋势还是平稳层状结构的 case。
// 做法是提前给每个 cell 计算 ascending stair support
// count，但它只提供上下文，不直接造 trigger 或 candidate。
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

// 解决 raw upper-support 在脚下 layered-ground / stair transition
// 场景里会把低层混入误解释成 obstacle 的 case。 做法是只在 explanation
// 内部用局部候选参考层重新检查 upper-support 是否仍成立，并只改 adjusted
// mask，不回写 support_ref。
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
      const auto &sample = frame.map_samples[sample_index];
      if (sample.point_in_map.z <
          support_anchor - config.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (std::abs(sample.point_in_map.z - effective_support_candidate_z) <=
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
            support_anchor - frame.base_pose_in_map.position.z(),
            workspace.support_ref - frame.base_pose_in_map.position.z(),
            effective_support_candidate_z -
                frame.base_pose_in_map.position.z(),
            effective_support_candidate_z - workspace.support_ref,
            workspace.upper_band_count, workspace.anchor_reobserve_count,
            candidate_reobserve_count, upper_layer_neighbor_match_count,
            workspace.ascending_stair_support_count,
            std::max(1, config.geometry.min_neighbor_upper_support_cells),
            config.geometry.support_anchor_reobserve_tolerance,
            config.geometry.max_step_up,
            workspace.anchor_below_observation_count,
            workspace.stale_lower_anchor_mix);
    if (!use_elevated_effective_support_ref) {
      continue;
    }

    bool has_elevated_upper_support = false;
    for (const size_t sample_index :
         sample_indices_by_cell[static_cast<size_t>(cell)]) {
      const auto &sample = frame.map_samples[sample_index];
      if (sample.point_in_map.z <
          support_anchor - config.geometry.sub_support_leak_tolerance) {
        continue;
      }
      if (sample.point_in_map.z >=
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

// 解决外部 debug/analyzer 还需要一个兼容字段，但内部已经拆成 raw/adjusted
// 两层的 case。 做法是把 legacy `upper_support_cell` 固定收口为 adjusted
// mask，避免 legacy 语义继续漂移。
void FinalizeUpperSupportCells(FrontendOutput &output) {
  // `upper_support_cell` remains a compatibility alias for the
  // explanation-adjusted confirmation mask. Raw local upper-band facts stay
  // available via `raw_upper_support_cell`.
  output.upper_support_cell = output.explanation_adjusted_upper_support_cell;
}

// 解决 confirm、explanation reject、keep verdict、candidate 形成混在一起的
// case。做法是：
// 1. local trigger 已经单独完成；
// 2. 这里先判断 upper patch confirmation；
// 3. 再统一进入 explanation reject；
// 4. reject 没命中后，必须再拿到显式 keep verdict；
// 5. aligned neighbor support 只作为 explanation evidence，不再 pre-veto。
void EvaluateCandidates(
    const ProcessedFrame &frame, const LocalTerrainMap &map,
    const TerrainLayers &layers, const Config &config,
    const std::vector<std::vector<size_t>> &sample_indices_by_cell,
    const std::vector<CellWorkspace> &cell_workspaces,
    const std::vector<int> &locally_triggered_cells, FrontendOutput &output) {
  const int min_neighbor_upper_support_cells =
      std::max(1, config.geometry.min_neighbor_upper_support_cells);
  for (const int cell : locally_triggered_cells) {
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
                     workspace.filtered_stats.max_z) <=
                config.geometry.support_anchor_reobserve_tolerance) {
          ++aligned_neighbor_support_count;
        }
      }
    }
    output.neighbor_upper_support_count[static_cast<size_t>(cell)] =
        static_cast<int8_t>(std::clamp(support_count, 0, 9));
    output.aligned_neighbor_support_count[static_cast<size_t>(cell)] =
        static_cast<int8_t>(std::clamp(aligned_neighbor_support_count, 0, 9));
    const bool upper_patch_confirmed =
        support_count >= min_neighbor_upper_support_cells;
    output.obstacle_upper_patch_confirmed[static_cast<size_t>(cell)] =
        upper_patch_confirmed ? 1U : 0U;
    if (!upper_patch_confirmed) {
      output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(cell)] =
          1U;
      output.obstacle_explanation_rejected[static_cast<size_t>(cell)] = 0U;
      output.explanation_decision[static_cast<size_t>(cell)] =
          static_cast<uint8_t>(FrontendExplanationDecision::kNone);
      continue;
    }

    const float support_anchor =
        output.support_anchor_used[static_cast<size_t>(cell)];
    const bool has_support_anchor = std::isfinite(support_anchor);
    const float vertical_span =
        workspace.filtered_stats.max_z - workspace.filtered_stats.min_z;
    const float relative_support_ref =
        workspace.support_ref - frame.base_pose_in_map.position.z();
    const float relative_upper_z =
        workspace.filtered_stats.max_z - frame.base_pose_in_map.position.z();
    const FrontendExplanationDecision reject_decision =
        ClassifyExplanationRejectDecision(
            has_support_anchor,
            support_anchor - frame.base_pose_in_map.position.z(),
            relative_support_ref, relative_upper_z, vertical_span,
            support_count, workspace.ascending_stair_support_count,
            aligned_neighbor_support_count, min_neighbor_upper_support_cells,
            config.geometry.max_step_down,
            config.geometry.upper_min_height_above_support,
            config.geometry.max_step_up,
            workspace.anchor_below_observation_count,
            workspace.stale_lower_anchor_mix);
    if (reject_decision != FrontendExplanationDecision::kNone) {
      output.explanation_decision[static_cast<size_t>(cell)] =
          static_cast<uint8_t>(reject_decision);
      output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(cell)] =
          1U;
      output.obstacle_explanation_rejected[static_cast<size_t>(cell)] = 1U;
      continue;
    }

    const FacadeEvidence facade_evidence = BuildFacadeEvidence(
        cell, frame, map, layers, config, sample_indices_by_cell,
        cell_workspaces, aligned_neighbor_support_count);
    output.facade_lower_upper_coexisting[static_cast<size_t>(cell)] =
        facade_evidence.lower_upper_coexisting ? 1U : 0U;
    output
        .facade_upper_edge_aligned_with_supported_neighbors[static_cast<size_t>(
            cell)] =
        facade_evidence.upper_edge_aligned_with_supported_neighbors ? 1U : 0U;
    const FrontendExplanationDecision keep_decision =
        ClassifyExplanationKeepDecision(facade_evidence);
    if (keep_decision == FrontendExplanationDecision::kNone) {
      output.explanation_decision[static_cast<size_t>(cell)] =
          static_cast<uint8_t>(FrontendExplanationDecision::kNone);
      continue;
    }
    output.explanation_decision[static_cast<size_t>(cell)] =
        static_cast<uint8_t>(keep_decision);

    const bool dense_facade_cell = workspace.filtered_stats.count >= 4;
    const int confirmed_facade_support_count_threshold =
        std::max(config.geometry.min_neighbor_upper_support_cells, 2);
    const ObstacleCandidateSemantic semantic =
        dense_facade_cell && workspace.sub_support_leak_count == 0U &&
                facade_evidence.lower_upper_coexisting &&
                relative_upper_z >= 0.0f &&
                support_count >= confirmed_facade_support_count_threshold
            ? ObstacleCandidateSemantic::kConfirmedFacade
            : ObstacleCandidateSemantic::kDefault;
    output.obstacle_candidate_cell[static_cast<size_t>(cell)] = 1U;
    output.obstacle_candidates.push_back(ObstacleCandidate{
        cell, workspace.filtered_stats.max_z,
        std::clamp(vertical_span, 0.0f, 1.0f), semantic});
  }
}

} // namespace

FrontendOutput PolarFrontend::run(const ProcessedFrame &frame,
                                  const FrameObservability &observability,
                                  const LocalTerrainMap &map) const {
  FrontendOutput output;
  output.support_anchor_used.assign(static_cast<size_t>(map.size()),
                                    std::numeric_limits<float>::quiet_NaN());
  output.support_anchor_origin.assign(static_cast<size_t>(map.size()), 0U);
  output.support_anchor_authority.assign(static_cast<size_t>(map.size()), 0U);
  output.anchor_leak_suppression_enabled.assign(static_cast<size_t>(map.size()),
                                                0U);
  output.sub_support_leak_count.assign(static_cast<size_t>(map.size()), 0U);
  output.anchor_below_observation_count.assign(static_cast<size_t>(map.size()),
                                               0U);
  output.stale_anchor_residual_filtered_count.assign(
      static_cast<size_t>(map.size()), 0U);
  output.raw_sample_min_z.assign(static_cast<size_t>(map.size()),
                                 std::numeric_limits<float>::quiet_NaN());
  output.raw_sample_max_z.assign(static_cast<size_t>(map.size()),
                                 std::numeric_limits<float>::quiet_NaN());
  output.raw_sample_count.assign(static_cast<size_t>(map.size()), 0U);
  output.filtered_sample_min_z.assign(static_cast<size_t>(map.size()),
                                      std::numeric_limits<float>::quiet_NaN());
  output.filtered_sample_max_z.assign(static_cast<size_t>(map.size()),
                                      std::numeric_limits<float>::quiet_NaN());
  output.filtered_sample_count.assign(static_cast<size_t>(map.size()), 0U);
  output.raw_upper_support_cell.assign(static_cast<size_t>(map.size()), 0U);
  output.explanation_adjusted_upper_support_cell.assign(
      static_cast<size_t>(map.size()), 0U);
  output.upper_support_cell.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_local_triggered.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_upper_patch_confirmed.assign(static_cast<size_t>(map.size()),
                                               0U);
  output.obstacle_explanation_rejected.assign(static_cast<size_t>(map.size()),
                                              0U);
  output.obstacle_suspicious.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_candidate_cell.assign(static_cast<size_t>(map.size()), 0U);
  output.obstacle_rejected_by_neighbor_support.assign(
      static_cast<size_t>(map.size()), 0U);
  output.neighbor_upper_support_count.assign(static_cast<size_t>(map.size()),
                                             0);
  output.aligned_neighbor_support_count.assign(static_cast<size_t>(map.size()),
                                               0);
  output.explanation_decision.assign(static_cast<size_t>(map.size()), 0U);
  output.facade_lower_upper_coexisting.assign(static_cast<size_t>(map.size()),
                                              0U);
  output.facade_upper_edge_aligned_with_supported_neighbors.assign(
      static_cast<size_t>(map.size()), 0U);
  const auto sample_indices_by_cell = GroupSampleIndicesByCell(frame, map);
  const int active_cell_count = CountActiveCells(sample_indices_by_cell);
  output.support_candidates.reserve(static_cast<size_t>(active_cell_count));
  output.obstacle_candidates.reserve(static_cast<size_t>(active_cell_count));
  output.ambiguous_candidates.reserve(static_cast<size_t>(active_cell_count));

  std::vector<CellWorkspace> cell_workspaces(static_cast<size_t>(map.size()));
  const auto &layers = map.layers();

  ResolveAnchors(frame, map, layers, config_, sample_indices_by_cell,
                 cell_workspaces);
  const auto locally_triggered_cells = BuildLocalProfilesAndMarkTriggers(
      frame, observability, map, layers, config_, sample_indices_by_cell,
      cell_workspaces, output);
  MarkUpperSupportCells(frame, map, config_, sample_indices_by_cell,
                        cell_workspaces, output);
  BuildAdjacencyContexts(map, config_, cell_workspaces);
  BuildExplanationInputs(frame, map, layers, config_, sample_indices_by_cell,
                         cell_workspaces, output);
  FinalizeUpperSupportCells(output);
  EvaluateCandidates(frame, map, layers, config_, sample_indices_by_cell,
                     cell_workspaces, locally_triggered_cells, output);
  return output;
}

} // namespace passable_area::core
