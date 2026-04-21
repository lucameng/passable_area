#include "passable_area/core/processor.hpp"
#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace passable_area::core {
namespace {

Point3f TransformMapPointToBaseGravity(const Point3f &point_in_map,
                                       const Pose3D &base_pose_in_map) {
  const float yaw = YawFromQuaternion(base_pose_in_map.orientation);
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  const float dx = point_in_map.x - base_pose_in_map.position.x();
  const float dy = point_in_map.y - base_pose_in_map.position.y();
  return Point3f{cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy,
                 point_in_map.z - base_pose_in_map.position.z()};
}

bool IsNear(float lhs, float rhs, float tolerance) {
  return std::isfinite(lhs) && std::isfinite(rhs) &&
         std::abs(lhs - rhs) <= tolerance;
}

bool IsRearPointInBaseGravity(const CellDebugPoint &point) {
  return point.point.x < 0.0f;
}

bool PassesObstaclePointPublishHeightGates(
    const passable_area::core::MapPointSample &sample,
    const passable_area::core::Point3f &point_in_base_gravity,
    float support_ref, const passable_area::core::Config &config) {
  (void)point_in_base_gravity;
  return std::isfinite(support_ref) &&
         sample.point_in_map.z >=
             support_ref + config.obstacle_points_min_height &&
         sample.point_in_base.z <=
             config.obstacle_points_max_height_in_base_link;
}

bool HasLowClearanceObstaclePointBridge(const TerrainLayers &layers,
                                        const FrameOutput &output, int cell,
                                        const Config &config) {
  const auto index = static_cast<size_t>(cell);
  if (index >= output.block_reason.size() ||
      index >= layers.overhead_evidence.size()) {
    return false;
  }
  if (output.block_reason[index] !=
      static_cast<uint8_t>(BlockReason::kLowClearance)) {
    return false;
  }
  const float clearance = index < layers.clearance.size()
                              ? layers.clearance[index]
                              : std::numeric_limits<float>::quiet_NaN();
  if (!std::isfinite(clearance) || clearance <= config.geometry.max_step_up) {
    return false;
  }
  return layers.overhead_evidence[index] >= config.obstacle_points_min_evidence;
}

bool HasDenseNearThresholdProtrusionSource(const TerrainLayers &layers,
                                           const FrameOutput &output, int cell,
                                           const Config &config) {
  const auto index = static_cast<size_t>(cell);
  if (index >= layers.protrusion_evidence.size() ||
      index >= output.raw_sample_count.size() ||
      index >= output.obstacle_candidate_cell.size()) {
    return false;
  }
  if (output.obstacle_candidate_cell[index] == 0U) {
    return false;
  }
  const float near_threshold =
      std::max(0.0f, config.obstacle_points_min_evidence -
                         config.persistence.obstacle_evidence_gain * 0.1f);
  const int dense_source_count =
      std::max(1, config.observability.min_points_per_sector - 1);
  return layers.protrusion_evidence[index] >= near_threshold &&
         output.raw_sample_count[index] >= dense_source_count;
}

bool HasObstaclePointPublishEvidence(const TerrainLayers &layers,
                                     const FrameOutput &output, int cell,
                                     const Config &config) {
  const auto index = static_cast<size_t>(cell);
  const bool is_low_clearance =
      index < output.block_reason.size() &&
      output.block_reason[index] ==
          static_cast<uint8_t>(BlockReason::kLowClearance);
  const bool protrusion_publish =
      !is_low_clearance && index < layers.protrusion_evidence.size() &&
      (layers.protrusion_evidence[index] >=
           config.obstacle_points_min_evidence ||
       HasDenseNearThresholdProtrusionSource(layers, output, cell, config));
  return protrusion_publish ||
         HasLowClearanceObstaclePointBridge(layers, output, cell, config);
}

MapGeometry MakeMapGeometry(const LocalTerrainMap &map) {
  return MapGeometry{map.rows(), map.cols(), map.size(), map.resolution(),
                     map.origin()};
}

} // namespace

Processor::Processor(const Config &config)
    : config_(config), preprocessor_(config), observability_estimator_(config),
      frontend_(config), map_(config), map_updater_(config),
      feature_updater_(config), traversability_solver_(config),
      obstacle_reasoner_(config) {}

FrameOutput Processor::update(const FrameInput &input) {
  ProcessedFrame preprocessed;
  if (!preprocessor_.process(input, preprocessed)) {
    return FrameOutput{};
  }

  map_.recenter(preprocessed.base_pose_in_map.position.head<2>());
  if (last_stamp_ > 0 && preprocessed.stamp > last_stamp_) {
    const double dt_sec =
        static_cast<double>(preprocessed.stamp - last_stamp_) * 1e-9;
    map_.setFramePeriodSec(static_cast<float>(std::clamp(dt_sec, 0.02, 0.5)));
  }
  last_stamp_ = preprocessed.stamp;

  const FrameObservability observability =
      observability_estimator_.estimate(preprocessed);
  const FrontendOutput frontend_output =
      frontend_.run(preprocessed, observability, MakeMapGeometry(map_));
  const std::vector<int> dirty_cells =
      map_updater_.update(frontend_output, observability, map_);
  feature_updater_.update(dirty_cells, map_);
  const auto reasoner_output = obstacle_reasoner_.evaluate(map_.layers());
  traversability_solver_.update(map_, reasoner_output);
  return buildOutput(preprocessed, observability, frontend_output,
                     reasoner_output);
}

FrameOutput
Processor::buildOutput(const ProcessedFrame &frame,
                       const FrameObservability &observability,
                       const FrontendOutput &frontend_output,
                       const ObstacleReasonerOutput &reasoner_output) {
  FrameOutput output;
  output.stamp = frame.stamp;
  output.base_pose_in_map = frame.base_pose_in_map;
  output.rows = map_.rows();
  output.cols = map_.cols();
  output.resolution = map_.resolution();
  output.origin = map_.origin();
  output.base_point_count = static_cast<uint32_t>(frame.cloud_in_base.size());
  output.map_point_count = static_cast<uint32_t>(frame.cloud_in_map.size());
  output.observability = observability;

  const auto &layers = map_.layers();
  output.passability = layers.passability_state;
  output.traversal_cost = layers.traversal_cost;
  output.support_height = layers.support_height;
  output.overhead_height = layers.overhead_height;
  output.protrusion_height = layers.protrusion_height;
  output.support_confidence = layers.support_confidence;
  output.protrusion_evidence = layers.protrusion_evidence;
  output.overhead_evidence = layers.overhead_evidence;
  output.obstacle_evidence = layers.obstacle_evidence;
  output.coverage_confidence = layers.coverage_confidence;
  output.slope = layers.slope;
  output.step_up = layers.step_up;
  output.step_down = layers.step_down;
  output.roughness = layers.roughness;
  output.clearance = layers.clearance;
  output.support_continuity = layers.support_continuity;
  output.block_reason = reasoner_output.block_reason;
  output.protrusion_stage = reasoner_output.protrusion_stage;
  output.overhead_stage = reasoner_output.overhead_stage;
  output.obstacle_point_publish_status =
      reasoner_output.obstacle_point_publish_status;
  output.raw_sample_min_z = frontend_output.raw_sample_min_z;
  output.raw_sample_max_z = frontend_output.raw_sample_max_z;
  output.raw_sample_count = frontend_output.raw_sample_count;
  output.filtered_sample_min_z = frontend_output.filtered_sample_min_z;
  output.filtered_sample_max_z = frontend_output.filtered_sample_max_z;
  output.filtered_sample_count = frontend_output.filtered_sample_count;
  output.obstacle_suspicious = frontend_output.obstacle_suspicious;
  output.obstacle_candidate_cell = frontend_output.obstacle_candidate_cell;
  output.support_state = layers.support_state;

  if (config_.debug.publish_base_gravity_cloud) {
    output.base_gravity_cloud_points.reserve(frame.cloud_in_map.size());
  }
  output.support_points.reserve(frame.map_samples.size() / 4);
  output.obstacle_points.reserve(frame.map_samples.size() / 8);
  output.unknown_points.reserve(map_.size() / 4);
  std::vector<CellDebugPoint> native_rear_obstacle_points;
  const float support_tolerance =
      std::max(config_.preprocess.voxel_size * 1.5f, config_.map.resolution);
  std::unordered_map<int, float> fallback_support_ref_by_cell;
  fallback_support_ref_by_cell.reserve(frame.map_samples.size() / 8U + 1U);

  for (const auto &sample : frame.map_samples) {
    int cell = -1;
    if (!map_.mapToIndex(sample.point_in_map.x, sample.point_in_map.y, cell)) {
      continue;
    }
    if (!HasObstaclePointPublishEvidence(layers, output, cell, config_) ||
        std::isfinite(layers.support_height[cell])) {
      continue;
    }
    auto [it, inserted] =
        fallback_support_ref_by_cell.emplace(cell, sample.point_in_map.z);
    if (!inserted) {
      it->second = std::min(it->second, sample.point_in_map.z);
    }
  }

  for (const auto &sample : frame.map_samples) {
    const Point3f point_in_base_gravity = TransformMapPointToBaseGravity(
        sample.point_in_map, frame.base_pose_in_map);
    if (config_.debug.publish_base_gravity_cloud) {
      output.base_gravity_cloud_points.push_back(
          MakeCellDebugPointWithoutSource(point_in_base_gravity));
    }

    int cell = -1;
    if (!map_.mapToIndex(sample.point_in_map.x, sample.point_in_map.y, cell)) {
      continue;
    }

    if (layers.support_confidence[cell] > 0.15f &&
        IsNear(sample.point_in_map.z, layers.support_height[cell],
               support_tolerance)) {
      output.support_points.push_back(
          MakeCellDebugPoint(point_in_base_gravity, cell));
    }

    float support_ref = layers.support_height[cell];
    if (!std::isfinite(support_ref)) {
      const auto it = fallback_support_ref_by_cell.find(cell);
      support_ref = it != fallback_support_ref_by_cell.end()
                        ? it->second
                        : std::numeric_limits<float>::infinity();
    }
    if (HasObstaclePointPublishEvidence(layers, output, cell, config_) &&
        PassesObstaclePointPublishHeightGates(sample, point_in_base_gravity,
                                              support_ref, config_)) {
      const auto point = MakeCellDebugPoint(point_in_base_gravity, cell);
      output.obstacle_points.push_back(point);
      if (IsRearPointInBaseGravity(point)) {
        native_rear_obstacle_points.push_back(point);
      }
    }
  }

  if (observability.rear_dropout && native_rear_obstacle_points.empty() &&
      rear_obstacle_point_cache_.valid &&
      !rear_obstacle_point_cache_.consumed_for_bridge) {
    for (const auto &cached_point : rear_obstacle_point_cache_.points) {
      if (cached_point.source_cell < 0 ||
          cached_point.source_cell >= map_.size()) {
        continue;
      }
      output.obstacle_points.push_back(cached_point);
    }
    rear_obstacle_point_cache_.consumed_for_bridge = true;
  }

  for (int cell = 0; cell < map_.size(); ++cell) {
    if (layers.passability_state[cell] ==
        static_cast<int8_t>(PassabilityState::kUnknown)) {
      const auto xy = map_.indexToMap(cell);
      const float z = std::isfinite(layers.support_height[cell])
                          ? layers.support_height[cell]
                          : 0.0f;
      output.unknown_points.push_back(MakeCellDebugPoint(
          TransformMapPointToBaseGravity(Point3f{xy.x(), xy.y(), z},
                                         frame.base_pose_in_map),
          cell));
    }
  }

  if (!observability.rear_dropout) {
    rear_obstacle_point_cache_.valid = !native_rear_obstacle_points.empty();
    rear_obstacle_point_cache_.consumed_for_bridge = false;
    rear_obstacle_point_cache_.stamp = frame.stamp;
    rear_obstacle_point_cache_.points = std::move(native_rear_obstacle_points);
  }

  output.valid = true;
  return output;
}

} // namespace passable_area::core
