#include "passable_area/core/pipeline/processor.hpp"
#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace passable_area::core {
namespace {

Point3f TransformOdomPointToBaseGravity(const Point3f &point_in_odom, const Pose3D &base_pose_in_odom) {
  const float yaw = YawFromQuaternion(base_pose_in_odom.orientation);
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  const float dx = point_in_odom.x - base_pose_in_odom.position.x();
  const float dy = point_in_odom.y - base_pose_in_odom.position.y();
  return Point3f{cos_yaw * dx + sin_yaw * dy,
                 -sin_yaw * dx + cos_yaw * dy,
                 point_in_odom.z - base_pose_in_odom.position.z()};
}

bool IsNear(float lhs, float rhs, float tolerance) {
  return std::isfinite(lhs) && std::isfinite(rhs) && std::abs(lhs - rhs) <= tolerance;
}

} // namespace

Processor::Processor(const Config &config)
    : config_(config), preprocessor_(config), observability_estimator_(config), frontend_(config),
      map_(config), map_updater_(config), feature_updater_(config), traversability_solver_(config) {}

FrameOutput Processor::update(const FrameInput &input) {
  ProcessedFrame preprocessed;
  if (!preprocessor_.process(input, preprocessed)) {
    return FrameOutput{};
  }

  map_.recenter(preprocessed.base_pose_in_odom.position.head<2>());
  if (last_stamp_ > 0 && preprocessed.stamp > last_stamp_) {
    const double dt_sec = static_cast<double>(preprocessed.stamp - last_stamp_) * 1e-9;
    map_.setFramePeriodSec(static_cast<float>(std::clamp(dt_sec, 0.02, 0.5)));
  }
  last_stamp_ = preprocessed.stamp;

  const FrameObservability observability = observability_estimator_.estimate(preprocessed);
  const FrontendOutput frontend_output = frontend_.run(preprocessed, observability, map_);
  const std::vector<int> dirty_cells = map_updater_.update(frontend_output, observability, map_);
  feature_updater_.update(dirty_cells, map_);
  traversability_solver_.update(map_);
  return buildOutput(preprocessed, observability);
}

FrameOutput Processor::buildOutput(const ProcessedFrame &frame,
                                   const FrameObservability &observability) const {
  FrameOutput output;
  output.stamp = frame.stamp;
  output.base_pose_in_odom = frame.base_pose_in_odom;
  output.rows = map_.rows();
  output.cols = map_.cols();
  output.resolution = map_.resolution();
  output.origin = map_.origin();
  output.base_point_count = static_cast<uint32_t>(frame.cloud_in_base.size());
  output.odom_point_count = static_cast<uint32_t>(frame.cloud_in_odom.size());
  output.observability = observability;

  const auto &layers = map_.layers();
  output.passability = layers.passability_state;
  output.traversal_cost = layers.traversal_cost;
  output.support_height = layers.support_height;
  output.overhead_height = layers.overhead_height;
  output.support_confidence = layers.support_confidence;
  output.obstacle_evidence = layers.obstacle_evidence;
  output.coverage_confidence = layers.coverage_confidence;
  output.slope = layers.slope;
  output.step_up = layers.step_up;
  output.step_down = layers.step_down;
  output.roughness = layers.roughness;
  output.clearance = layers.clearance;
  output.support_continuity = layers.support_continuity;
  output.support_state = layers.support_state;

  if (config_.debug.publish_base_gravity_cloud) {
    output.base_gravity_cloud_points.reserve(frame.cloud_in_odom.size());
  }
  output.support_points.reserve(frame.odom_samples.size() / 4);
  output.obstacle_points.reserve(frame.odom_samples.size() / 8);
  output.unknown_points.reserve(map_.size() / 4);
  const float support_tolerance =
      std::max(config_.preprocess.voxel_size * 1.5f, config_.map.resolution);

  for (const auto &sample : frame.odom_samples) {
    if (config_.debug.publish_base_gravity_cloud) {
      output.base_gravity_cloud_points.push_back(CellDebugPoint{
          TransformOdomPointToBaseGravity(sample.point_in_odom, frame.base_pose_in_odom)});
    }

    int cell = -1;
    if (!map_.odomToIndex(sample.point_in_odom.x, sample.point_in_odom.y, cell)) {
      continue;
    }

    if (layers.support_confidence[cell] > 0.15f &&
        IsNear(sample.point_in_odom.z, layers.support_height[cell], support_tolerance)) {
      output.support_points.push_back(
          CellDebugPoint{TransformOdomPointToBaseGravity(sample.point_in_odom, frame.base_pose_in_odom)});
    }

    if (layers.obstacle_evidence[cell] >= config_.debug.obstacle_points_min_evidence) {
      output.obstacle_points.push_back(
          CellDebugPoint{TransformOdomPointToBaseGravity(sample.point_in_odom, frame.base_pose_in_odom)});
    }
  }

  for (int cell = 0; cell < map_.size(); ++cell) {
    if (layers.passability_state[cell] == static_cast<int8_t>(PassabilityState::kUnknown)) {
      const auto xy = map_.indexToOdom(cell);
      const float z =
          std::isfinite(layers.support_height[cell]) ? layers.support_height[cell] : 0.0f;
      output.unknown_points.push_back(CellDebugPoint{TransformOdomPointToBaseGravity(
          Point3f{xy.x(), xy.y(), z}, frame.base_pose_in_odom)});
    }
  }

  output.valid = true;
  return output;
}

} // namespace passable_area::core
