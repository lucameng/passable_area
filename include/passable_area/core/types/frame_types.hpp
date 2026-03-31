#ifndef PASSABLE_AREA_CORE_TYPES_FRAME_TYPES_HPP_
#define PASSABLE_AREA_CORE_TYPES_FRAME_TYPES_HPP_

#include "passable_area/core/types/basic_types.hpp"
#include "passable_area/core/types/state_types.hpp"

#include <vector>

namespace passable_area::core {

struct FrameInput {
  Timestamp stamp = 0;
  Pose3D base_pose_in_local;
  PointCloud merged_cloud;
  bool processing_enabled = true;
};

struct SectorObservability {
  ObservabilityState state = ObservabilityState::kPartiallyObserved;
  float coverage_confidence = 0.0f;
};

struct FrameObservability {
  bool frame_partial = false;
  bool rear_dropout = false;
  std::vector<SectorObservability> sectors;
};

struct CellDebugPoint {
  Point3f point;
};

struct FrameOutput {
  Timestamp stamp = 0;
  int rows = 0;
  int cols = 0;
  float resolution = 0.1f;
  Eigen::Vector2f origin = Eigen::Vector2f::Zero();
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
  std::vector<uint8_t> support_state;
  std::vector<CellDebugPoint> support_points;
  std::vector<CellDebugPoint> obstacle_points;
  std::vector<CellDebugPoint> unknown_points;
  FrameObservability observability;
  bool valid = false;
};

} // namespace passable_area::core

#endif
