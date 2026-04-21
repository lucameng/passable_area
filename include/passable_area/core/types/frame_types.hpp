#ifndef PASSABLE_AREA_CORE_TYPES_FRAME_TYPES_HPP_
#define PASSABLE_AREA_CORE_TYPES_FRAME_TYPES_HPP_

#include "passable_area/core/types/basic_types.hpp"
#include "passable_area/core/types/state_types.hpp"

#include <vector>

namespace passable_area::core {

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
  ObservabilityState state = ObservabilityState::kObserved;
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

inline CellDebugPoint MakeCellDebugPoint(const Point3f &point,
                                         int source_cell) {
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
  std::vector<float> protrusion_height;
  std::vector<float> support_confidence;
  std::vector<float> protrusion_evidence;
  std::vector<float> overhead_evidence;
  std::vector<float> obstacle_evidence;
  std::vector<float> coverage_confidence;
  std::vector<float> slope;
  std::vector<float> step_up;
  std::vector<float> step_down;
  std::vector<float> roughness;
  std::vector<float> clearance;
  std::vector<float> support_continuity;
  std::vector<uint8_t> block_reason;
  std::vector<uint8_t> protrusion_stage;
  std::vector<uint8_t> overhead_stage;
  std::vector<uint8_t> obstacle_point_publish_status;
  std::vector<float> raw_sample_min_z;
  std::vector<float> raw_sample_max_z;
  std::vector<uint16_t> raw_sample_count;
  std::vector<float> filtered_sample_min_z;
  std::vector<float> filtered_sample_max_z;
  std::vector<uint16_t> filtered_sample_count;
  std::vector<uint8_t> obstacle_suspicious;
  std::vector<uint8_t> obstacle_candidate_cell;
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
