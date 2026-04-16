#ifndef PASSABLE_AREA_CORE_TYPES_FRAME_TYPES_HPP_
#define PASSABLE_AREA_CORE_TYPES_FRAME_TYPES_HPP_

#include "passable_area/core/types/basic_types.hpp"
#include "passable_area/core/types/state_types.hpp"

#include <vector>

namespace passable_area::core {

enum class FrontendExplanationDecision : uint8_t {
  kNone = 0U,
  kBelowRobotStairMix = 1U,
  kBelowRobotGroundLayerMix = 2U,
  kBelowRobotUpstairGroundMix = 3U,
  kKeepAsObstacle = 4U,
};

enum class SupportAnchorOrigin : uint8_t {
  kNone = 0U,
  kLocalSupport = 1U,
  kBorrowedNeighbor = 2U,
};

enum class SupportAnchorAuthority : uint8_t {
  kInvalid = 0U,
  kExplanationOnly = 1U,
  kLeakEligible = 2U,
};

struct FrameInput {
  Timestamp stamp = 0;
  // Historical field name. The numeric pose semantics now follow the external
  // map frame even though core code still uses the legacy *_in_map suffix.
  Pose3D base_pose_in_map;
  PointCloud input_cloud_in_base;
  bool processing_enabled = true;
};

struct MapPointSample {
  Point3f point_in_base;
  // Historical field name. The numeric coordinates now live in map semantics.
  Point3f point_in_map;
};

struct ProcessedFrame {
  Timestamp stamp = 0;
  // Historical field name retained to avoid broad algorithm churn in this pass.
  Pose3D base_pose_in_map;
  PointCloud cloud_in_base;
  // Historical field names retained; these points are interpreted in map.
  PointCloud cloud_in_map;
  std::vector<MapPointSample> map_samples;
  bool processing_enabled = true;
};

struct SectorObservability {
  ObservabilityState state = ObservabilityState::kPartiallyObserved;
  float coverage_confidence = 0.0f;
};

struct FrameObservability {
  bool frame_partial = false;
  bool rear_dropout = false;
  uint32_t base_point_count = 0;
  uint32_t map_point_count = 0;
  std::vector<SectorObservability> sectors;
};

struct CellDebugPoint {
  Point3f point;
  int source_cell = -1;
};

inline CellDebugPoint MakeCellDebugPoint(const Point3f &point, int source_cell) {
  return CellDebugPoint{point, source_cell};
}

inline CellDebugPoint MakeCellDebugPointWithoutSource(const Point3f &point) {
  return CellDebugPoint{point, -1};
}

struct FrameOutput {
  Timestamp stamp = 0;
  // Publish-context only. Internal map semantics remain defined by
  // origin/resolution/layers in map; the field keeps its historical name to
  // avoid broad internal renames in this pass.
  Pose3D base_pose_in_map;
  int rows = 0;
  int cols = 0;
  float resolution = 0.1f;
  Eigen::Vector2f origin = Eigen::Vector2f::Zero();
  uint32_t base_point_count = 0;
  uint32_t map_point_count = 0;
  std::vector<int8_t> passability;
  std::vector<int8_t> traversal_cost;
  std::vector<float> support_height;
  std::vector<float> overhead_height;
  std::vector<float> support_confidence;
  std::vector<float> obstacle_evidence;
  std::vector<float> coverage_confidence;
  std::vector<float> slope;
  std::vector<float> step_up;
  std::vector<float> step_down;
  std::vector<float> roughness;
  std::vector<float> clearance;
  std::vector<float> support_continuity;
  std::vector<float> support_anchor_used;
  std::vector<uint8_t> support_anchor_origin;
  std::vector<uint8_t> support_anchor_authority;
  std::vector<uint8_t> anchor_leak_suppression_enabled;
  std::vector<uint16_t> sub_support_leak_count;
  std::vector<uint16_t> anchor_below_observation_count;
  std::vector<uint16_t> stale_anchor_residual_filtered_count;
  std::vector<float> raw_sample_min_z;
  std::vector<float> raw_sample_max_z;
  std::vector<uint16_t> raw_sample_count;
  std::vector<float> filtered_sample_min_z;
  std::vector<float> filtered_sample_max_z;
  std::vector<uint16_t> filtered_sample_count;
  // Raw local upper-band fact relative to the current cell support_ref.
  std::vector<uint8_t> raw_upper_support_cell;
  // Explanation-adjusted upper-support mask used by neighborhood confirmation.
  std::vector<uint8_t> explanation_adjusted_upper_support_cell;
  // Compatibility alias kept equal to explanation_adjusted_upper_support_cell.
  std::vector<uint8_t> upper_support_cell;
  std::vector<uint8_t> obstacle_local_triggered;
  std::vector<uint8_t> obstacle_upper_patch_confirmed;
  std::vector<uint8_t> obstacle_explanation_rejected;
  std::vector<uint8_t> obstacle_suspicious;
  std::vector<uint8_t> obstacle_candidate_cell;
  // Legacy compatibility flag. It is set when a locally triggered cell fails
  // upper-patch confirmation or is explicitly rejected during explanation.
  // Use obstacle_upper_patch_confirmed / obstacle_explanation_rejected /
  // explanation_decision as the primary stage indicators.
  std::vector<uint8_t> obstacle_rejected_by_neighbor_support;
  std::vector<int8_t> neighbor_upper_support_count;
  std::vector<int8_t> aligned_neighbor_support_count;
  std::vector<uint8_t> explanation_decision;
  std::vector<uint8_t> facade_lower_upper_coexisting;
  std::vector<uint8_t> facade_upper_edge_aligned_with_supported_neighbors;
  std::vector<uint8_t> support_state;
  std::vector<CellDebugPoint> base_gravity_cloud_points;
  std::vector<CellDebugPoint> support_points;
  std::vector<CellDebugPoint> obstacle_points;
  std::vector<CellDebugPoint> unknown_points;
  FrameObservability observability;
  bool valid = false;
};

} // namespace passable_area::core

#endif
