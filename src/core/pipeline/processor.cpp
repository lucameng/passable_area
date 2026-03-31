#include "passable_area/core/pipeline/processor.hpp"

#include <cmath>

namespace passable_area::core {

Processor::Processor(const Config &config)
    : config_(config), preprocessor_(config), observability_estimator_(config), frontend_(config),
      map_(config), map_updater_(config), feature_updater_(config), traversability_solver_(config) {}

FrameOutput Processor::update(const FrameInput &input) {
  ProcessedFrame preprocessed;
  if (!preprocessor_.process(input, preprocessed)) {
    return FrameOutput{};
  }

  map_.recenter(preprocessed.base_pose_in_local.position.head<2>());
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
  output.rows = map_.rows();
  output.cols = map_.cols();
  output.resolution = map_.resolution();
  output.origin = map_.origin();
  output.base_point_count = static_cast<uint32_t>(frame.cloud_in_base.size());
  output.gravity_point_count = static_cast<uint32_t>(frame.cloud_in_gravity.size());
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

  output.support_points.reserve(map_.size() / 4);
  output.obstacle_points.reserve(map_.size() / 8);
  output.unknown_points.reserve(map_.size() / 4);
  for (int cell = 0; cell < map_.size(); ++cell) {
    const auto xy = map_.indexToWorld(cell);
    if (std::isfinite(layers.support_height[cell]) && layers.support_confidence[cell] > 0.15f) {
      output.support_points.push_back(CellDebugPoint{{xy.x(), xy.y(), layers.support_height[cell]}});
    }
    if (std::isfinite(layers.overhead_height[cell]) && layers.obstacle_evidence[cell] > 0.2f) {
      output.obstacle_points.push_back(CellDebugPoint{{xy.x(), xy.y(), layers.overhead_height[cell]}});
    }
    if (layers.passability_state[cell] == static_cast<int8_t>(PassabilityState::kUnknown)) {
      const float z = std::isfinite(layers.support_height[cell]) ? layers.support_height[cell] : 0.0f;
      output.unknown_points.push_back(CellDebugPoint{{xy.x(), xy.y(), z}});
    }
  }

  output.valid = true;
  return output;
}

} // namespace passable_area::core
