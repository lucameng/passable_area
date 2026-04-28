#include "passable_area/core/processor.hpp"
#include "passable_area/core/config_validation.hpp"
#include "passable_area/core/obstacle_publication.hpp"
#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace passable_area::core {
namespace {

constexpr float kRearBridgeMaxTranslationCells = 3.0f;
constexpr float kRearBridgeMaxYawChangeRad = 0.78539816339f;
constexpr float kPi = 3.14159265359f;
constexpr float kSupportDebugToleranceVoxelScale = 1.5f;

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

Point3f TransformMapPointToBaseLink(const Point3f &point_in_map,
                                    const Pose3D &base_pose_in_map) {
  const Eigen::Vector3f point(point_in_map.x, point_in_map.y, point_in_map.z);
  const Eigen::Vector3f point_in_base =
      base_pose_in_map.orientation.normalized().inverse() *
      (point - base_pose_in_map.position);
  return Point3f{point_in_base.x(), point_in_base.y(), point_in_base.z()};
}

bool IsNear(float lhs, float rhs, float tolerance) {
  return std::isfinite(lhs) && std::isfinite(rhs) &&
         std::abs(lhs - rhs) <= tolerance;
}

bool IsRearPointInBaseGravity(const CellDebugPoint &point) {
  return point.point.x < 0.0f;
}

float NormalizeAngle(float angle) {
  while (angle > kPi) {
    angle -= 2.0f * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0f * kPi;
  }
  return angle;
}

bool IsRearBridgeCacheFresh(Timestamp cached_stamp, Timestamp current_stamp,
                            float frame_period_sec) {
  if (cached_stamp <= 0 || current_stamp <= cached_stamp) {
    return false;
  }
  const double age_sec =
      static_cast<double>(current_stamp - cached_stamp) * 1e-9;
  const double max_age_sec =
      std::max(0.2, static_cast<double>(frame_period_sec) * 2.0);
  return age_sec <= max_age_sec;
}

bool IsRearBridgeMotionValid(const Pose3D &cached_pose,
                             const Pose3D &current_pose, float map_resolution) {
  const Eigen::Vector2f delta =
      (current_pose.position - cached_pose.position).head<2>();
  const float max_translation =
      std::max(map_resolution, map_resolution * kRearBridgeMaxTranslationCells);
  if (delta.norm() > max_translation) {
    return false;
  }
  const float yaw_delta =
      NormalizeAngle(YawFromQuaternion(current_pose.orientation) -
                     YawFromQuaternion(cached_pose.orientation));
  return std::abs(yaw_delta) <= kRearBridgeMaxYawChangeRad;
}

bool HasObstaclePointPublishDecision(const FrameOutput &output, int cell) {
  const auto index = static_cast<size_t>(cell);
  return index < output.obstacle_point_publish_status.size() &&
         IsObstaclePointPublishStatusPublished(
             output.obstacle_point_publish_status[index]);
}

bool HasObstaclePointPublishDecision(const std::vector<uint8_t> &statuses,
                                     int cell) {
  const auto index = static_cast<size_t>(cell);
  return index < statuses.size() &&
         IsObstaclePointPublishStatusPublished(statuses[index]);
}

MapGeometry MakeMapGeometry(const LocalTerrainMap &map) {
  return MapGeometry{map.rows(), map.cols(), map.size(), map.resolution(),
                     map.origin()};
}

} // namespace

Processor::Processor(const Config &config)
    : config_(config), preprocessor_(config_), observability_estimator_(config_),
      frontend_(config_), map_(config_), map_updater_(config_),
      feature_updater_(config_), traversability_solver_(config_),
      obstacle_reasoner_(config_) {
  ValidateConfigOrThrow(config_);
}

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
  const std::vector<int> dirty_cells = map_updater_.update(
      frontend_output, observability, preprocessed.base_pose_in_map, map_);
  feature_updater_.update(dirty_cells, map_);
  std::vector<float> current_protrusion_evidence_gain(map_.size(), 0.0f);
  std::vector<float> current_obstacle_candidate_height(
      map_.size(), std::numeric_limits<float>::quiet_NaN());
  for (size_t cell = 0; cell < frontend_output.obstacle_candidate_cell.size() &&
                        cell < current_obstacle_candidate_height.size() &&
                        cell < frontend_output.raw_sample_max_z.size();
       ++cell) {
    if (frontend_output.obstacle_candidate_cell[cell] == 0U ||
        !std::isfinite(frontend_output.raw_sample_max_z[cell])) {
      continue;
    }
    current_obstacle_candidate_height[cell] =
        frontend_output.raw_sample_max_z[cell];
  }
  for (const auto &candidate : frontend_output.protrusion_candidates) {
    if (candidate.cell < 0 || candidate.cell >= map_.size()) {
      continue;
    }
    const auto cell = static_cast<size_t>(candidate.cell);
    current_protrusion_evidence_gain[cell] =
        std::max(current_protrusion_evidence_gain[cell],
                 config_.persistence.obstacle_evidence_gain *
                     std::max(1.0f, candidate.gain_scale) *
                     candidate.evidence);
    current_obstacle_candidate_height[cell] =
        std::isfinite(current_obstacle_candidate_height[cell])
            ? std::max(current_obstacle_candidate_height[cell], candidate.z)
            : candidate.z;
  }
  const ObstaclePublicationContext publication_context{
      &frontend_output.raw_sample_count,
      &frontend_output.obstacle_candidate_cell,
      &current_protrusion_evidence_gain,
      &current_obstacle_candidate_height};
  const auto reasoner_output =
      obstacle_reasoner_.evaluate(map_.layers(), publication_context);
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
  const std::vector<uint8_t> reasoner_publish_status =
      output.obstacle_point_publish_status;
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
  std::vector<CachedRearObstaclePoint> native_rear_obstacle_points;
  const float support_tolerance =
      std::max(config_.preprocess.voxel_size * kSupportDebugToleranceVoxelScale,
               config_.map.resolution);
  std::vector<uint8_t> publish_decision_sample_seen(map_.size(), 0U);
  std::vector<uint8_t> native_obstacle_point_published(map_.size(), 0U);

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

    if (!HasObstaclePointPublishDecision(output, cell)) {
      continue;
    }
    publish_decision_sample_seen[static_cast<size_t>(cell)] = 1U;
    const uint8_t publish_status =
        output.obstacle_point_publish_status[static_cast<size_t>(cell)];
    ObstaclePublicationSampleStats sample_stats;
    AccumulateObstaclePublicationSample(
        config_, publish_status, layers.support_height[cell],
        ObstaclePublicationSample{sample.point_in_map.z, sample.point_in_base.z,
                                  point_in_base_gravity.z},
        sample_stats);
    const auto publication_trace = EvaluateObstaclePublicationDecisionTrace(
        config_, publish_status, layers.support_height[cell], sample_stats);
    if (publication_trace.height_gate.passes) {
      const auto point = MakeCellDebugPoint(point_in_base_gravity, cell);
      output.obstacle_points.push_back(point);
      native_obstacle_point_published[static_cast<size_t>(cell)] = 1U;
      if (IsRearPointInBaseGravity(point)) {
        native_rear_obstacle_points.push_back(
            CachedRearObstaclePoint{sample.point_in_map, cell});
      }
    }
  }

  if (observability.rear_dropout && native_rear_obstacle_points.empty() &&
      rear_obstacle_point_cache_.valid &&
      !rear_obstacle_point_cache_.consumed_for_bridge &&
      IsRearBridgeCacheFresh(rear_obstacle_point_cache_.stamp, frame.stamp,
                             map_.framePeriodSec()) &&
      IsRearBridgeMotionValid(rear_obstacle_point_cache_.base_pose_in_map,
                              frame.base_pose_in_map, map_.resolution())) {
    for (const auto &cached_point : rear_obstacle_point_cache_.points) {
      int current_cell = -1;
      if (!map_.mapToIndex(cached_point.point_in_map.x,
                           cached_point.point_in_map.y, current_cell) ||
          !HasObstaclePointPublishDecision(reasoner_publish_status,
                                           current_cell)) {
        continue;
      }
      const Point3f point_in_base_gravity = TransformMapPointToBaseGravity(
          cached_point.point_in_map, frame.base_pose_in_map);
      const Point3f point_in_base = TransformMapPointToBaseLink(
          cached_point.point_in_map, frame.base_pose_in_map);
      const float support_ref = layers.support_height[current_cell];
      const uint8_t publish_status =
          reasoner_publish_status[static_cast<size_t>(current_cell)];
      ObstaclePublicationSampleStats sample_stats;
      AccumulateObstaclePublicationSample(
          config_, publish_status, support_ref,
          ObstaclePublicationSample{cached_point.point_in_map.z,
                                    point_in_base.z, point_in_base_gravity.z},
          sample_stats);
      const auto publication_trace = EvaluateObstaclePublicationDecisionTrace(
          config_, publish_status, support_ref, sample_stats);
      if (!publication_trace.height_gate.passes) {
        continue;
      }
      output.obstacle_points.push_back(
          MakeCellDebugPoint(point_in_base_gravity, current_cell));
      native_obstacle_point_published[static_cast<size_t>(current_cell)] = 1U;
      output.obstacle_point_publish_status[static_cast<size_t>(current_cell)] =
          publish_status;
    }
    rear_obstacle_point_cache_.consumed_for_bridge = true;
  }

  for (int cell = 0; cell < map_.size(); ++cell) {
    if (!HasObstaclePointPublishDecision(reasoner_publish_status, cell) ||
        native_obstacle_point_published[static_cast<size_t>(cell)] != 0U) {
      continue;
    }
    output.obstacle_point_publish_status[static_cast<size_t>(cell)] =
        static_cast<uint8_t>(FinalizeObstaclePublicationStatus(
            reasoner_publish_status[static_cast<size_t>(cell)],
            publish_decision_sample_seen[static_cast<size_t>(cell)] != 0U,
            false));
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
    rear_obstacle_point_cache_.base_pose_in_map = frame.base_pose_in_map;
    rear_obstacle_point_cache_.points = std::move(native_rear_obstacle_points);
  }

  output.valid = true;
  return output;
}

} // namespace passable_area::core
