#include "passable_area/core/processor.hpp"
#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
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

bool PassesObstaclePointPublishHeightGates(
    const passable_area::core::MapPointSample &sample, float support_ref,
    const passable_area::core::Config &config) {
  return std::isfinite(support_ref) &&
         sample.point_in_map.z >=
             support_ref + config.obstacle_points_min_height &&
         sample.point_in_base.z <=
             config.obstacle_points_max_height_in_base_link;
}

float LookupObstacleSupportRef(
    int cell, const passable_area::core::TerrainLayers &layers,
    const std::unordered_map<int, float> &fallback_support_ref_by_cell) {
  if (std::isfinite(layers.support_height[cell])) {
    return layers.support_height[cell];
  }
  const auto it = fallback_support_ref_by_cell.find(cell);
  return it != fallback_support_ref_by_cell.end()
             ? it->second
             : std::numeric_limits<float>::infinity();
}

bool IsPublishableObstacleCell(
    int sample_cell, const passable_area::core::MapPointSample &sample,
    const passable_area::core::TerrainLayers &layers,
    const std::unordered_map<int, float> &fallback_support_ref_by_cell,
    const passable_area::core::Config &config) {
  if (layers.obstacle_publishable[sample_cell] ==
      static_cast<uint8_t>(
          passable_area::core::ObstaclePublishabilityState::kNotPublishable)) {
    return false;
  }
  const float direct_support_ref = LookupObstacleSupportRef(
      sample_cell, layers, fallback_support_ref_by_cell);
  return PassesObstaclePointPublishHeightGates(sample, direct_support_ref,
                                               config);
}

std::vector<uint8_t> BuildCurrentCandidateMask(
    const passable_area::core::FrontendOutput &frontend_output, int cell_count) {
  std::vector<uint8_t> current_candidate(static_cast<size_t>(cell_count), 0U);
  for (const auto &candidate : frontend_output.obstacle_candidates) {
    if (candidate.cell >= 0 && candidate.cell < cell_count) {
      current_candidate[static_cast<size_t>(candidate.cell)] = 1U;
    }
  }
  if (frontend_output.obstacle_candidate_cell.size() ==
      static_cast<size_t>(cell_count)) {
    current_candidate = frontend_output.obstacle_candidate_cell;
  }
  return current_candidate;
}

std::vector<uint8_t> BuildConfirmedFacadeMask(
    const passable_area::core::FrontendOutput &frontend_output, int cell_count) {
  std::vector<uint8_t> confirmed_facade(static_cast<size_t>(cell_count), 0U);
  for (const auto &candidate : frontend_output.obstacle_candidates) {
    if (candidate.cell >= 0 && candidate.cell < cell_count &&
        candidate.semantic ==
            passable_area::core::ObstacleCandidateSemantic::kConfirmedFacade) {
      confirmed_facade[static_cast<size_t>(candidate.cell)] = 1U;
    }
  }
  return confirmed_facade;
}

bool CellHasObstacleEvidenceForOwnership(
    int cell, const passable_area::core::TerrainLayers &layers,
    const passable_area::core::Config &config) {
  return layers.obstacle_evidence[cell] >= config.obstacle_points_min_evidence &&
         std::isfinite(layers.overhead_height[cell]);
}

bool CellHasPublishableHeightEnvelope(
    int cell, const passable_area::core::TerrainLayers &layers,
    const passable_area::core::Config &config) {
  if (!std::isfinite(layers.overhead_height[cell])) {
    return false;
  }
  if (!std::isfinite(layers.support_height[cell])) {
    return true;
  }
  return layers.overhead_height[cell] >=
         layers.support_height[cell] + config.obstacle_points_min_height;
}

bool IsCeilingLikeOverheadCell(int cell,
                               const passable_area::core::TerrainLayers &layers,
                               const passable_area::core::Config &config) {
  return std::isfinite(layers.support_height[cell]) &&
         std::isfinite(layers.clearance[cell]) &&
         layers.clearance[cell] < config.geometry.min_clearance &&
         layers.support_continuity[cell] >= 0.99f;
}

bool CellsSharePublishableObstacleStructure(
    int lhs_cell, int rhs_cell, int cols,
    const passable_area::core::TerrainLayers &layers,
    const passable_area::core::Config &config) {
  const int lhs_row = lhs_cell / cols;
  const int lhs_col = lhs_cell % cols;
  const int rhs_row = rhs_cell / cols;
  const int rhs_col = rhs_cell % cols;
  if (std::abs(lhs_row - rhs_row) > 1 || std::abs(lhs_col - rhs_col) > 1) {
    return false;
  }
  if (!std::isfinite(layers.overhead_height[lhs_cell]) ||
      !std::isfinite(layers.overhead_height[rhs_cell])) {
    return false;
  }

  const float overhead_tolerance =
      std::max({config.map.resolution, config.geometry.max_step_up,
                config.geometry.support_anchor_reobserve_tolerance});
  if (std::abs(layers.overhead_height[lhs_cell] -
               layers.overhead_height[rhs_cell]) > overhead_tolerance) {
    return false;
  }

  const bool lhs_has_support = std::isfinite(layers.support_height[lhs_cell]);
  const bool rhs_has_support = std::isfinite(layers.support_height[rhs_cell]);
  if (lhs_has_support != rhs_has_support) {
    return false;
  }
  if (!lhs_has_support) {
    return true;
  }

  const float support_tolerance =
      std::max({config.map.resolution, config.geometry.max_step_down,
                config.geometry.support_anchor_reobserve_tolerance});
  return std::abs(layers.support_height[lhs_cell] -
                  layers.support_height[rhs_cell]) <= support_tolerance;
}

std::vector<uint8_t> RecomputeObstaclePublishableMask(
    const passable_area::core::FrontendOutput &frontend_output,
    const std::vector<uint8_t> &previous_publishable,
    const passable_area::core::LocalTerrainMap &map,
    const passable_area::core::Config &config) {
  const auto &layers = map.layers();
  const auto current_candidate =
      BuildCurrentCandidateMask(frontend_output, map.size());
  const auto confirmed_facade =
      BuildConfirmedFacadeMask(frontend_output, map.size());

  std::vector<uint8_t> publishable(static_cast<size_t>(map.size()), 0U);
  std::deque<int> queue;

  for (int cell = 0; cell < map.size(); ++cell) {
    if (current_candidate[static_cast<size_t>(cell)] == 0U ||
        !CellHasObstacleEvidenceForOwnership(cell, layers, config) ||
        !CellHasPublishableHeightEnvelope(cell, layers, config) ||
        IsCeilingLikeOverheadCell(cell, layers, config)) {
      continue;
    }
    const bool structure_seed =
        confirmed_facade[static_cast<size_t>(cell)] != 0U ||
        !std::isfinite(layers.support_height[cell]) ||
        layers.support_continuity[cell] < 0.99f ||
        previous_publishable[static_cast<size_t>(cell)] != 0U;
    if (!structure_seed) {
      continue;
    }
    publishable[static_cast<size_t>(cell)] = 1U;
    queue.push_back(cell);
  }

  while (!queue.empty()) {
    const int cell = queue.front();
    queue.pop_front();
    const int row = cell / map.cols();
    const int col = cell % map.cols();
    for (int row_offset = -1; row_offset <= 1; ++row_offset) {
      for (int col_offset = -1; col_offset <= 1; ++col_offset) {
        if (row_offset == 0 && col_offset == 0) {
          continue;
        }
        const int neighbor_row = row + row_offset;
        const int neighbor_col = col + col_offset;
        if (neighbor_row < 0 || neighbor_row >= map.rows() || neighbor_col < 0 ||
            neighbor_col >= map.cols()) {
          continue;
        }
        const int neighbor = neighbor_row * map.cols() + neighbor_col;
        if (publishable[static_cast<size_t>(neighbor)] != 0U ||
            !CellHasObstacleEvidenceForOwnership(neighbor, layers, config) ||
            !CellHasPublishableHeightEnvelope(neighbor, layers, config) ||
            IsCeilingLikeOverheadCell(neighbor, layers, config) ||
            !CellsSharePublishableObstacleStructure(cell, neighbor, map.cols(),
                                                    layers, config)) {
          continue;
        }
        const bool current_or_owned =
            current_candidate[static_cast<size_t>(neighbor)] != 0U ||
            previous_publishable[static_cast<size_t>(neighbor)] != 0U;
        if (!current_or_owned) {
          continue;
        }
        publishable[static_cast<size_t>(neighbor)] = 1U;
        queue.push_back(neighbor);
      }
    }
  }

  return publishable;
}

} // namespace

Processor::Processor(const Config &config)
    : config_(config), preprocessor_(config), observability_estimator_(config),
      frontend_(config), map_(config), map_updater_(config),
      feature_updater_(config), traversability_solver_(config) {}

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
      frontend_.run(preprocessed, observability, map_);
  const std::vector<uint8_t> previous_publishable =
      map_.layers().obstacle_publishable;
  const std::vector<int> dirty_cells =
      map_updater_.update(frontend_output, observability, map_);
  feature_updater_.update(dirty_cells, map_);
  traversability_solver_.update(map_);
  map_.layers().obstacle_publishable = RecomputeObstaclePublishableMask(
      frontend_output, previous_publishable, map_, config_);
  return buildOutput(preprocessed, observability, frontend_output);
}

FrameOutput
Processor::buildOutput(const ProcessedFrame &frame,
                       const FrameObservability &observability,
                       const FrontendOutput &frontend_output) const {
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
  output.support_confidence = layers.support_confidence;
  output.obstacle_evidence = layers.obstacle_evidence;
  output.coverage_confidence = layers.coverage_confidence;
  output.slope = layers.slope;
  output.step_up = layers.step_up;
  output.step_down = layers.step_down;
  output.roughness = layers.roughness;
  output.clearance = layers.clearance;
  output.support_continuity = layers.support_continuity;
  output.obstacle_publishable = layers.obstacle_publishable;
  output.support_anchor_used = frontend_output.support_anchor_used;
  output.support_anchor_origin = frontend_output.support_anchor_origin;
  output.support_anchor_authority = frontend_output.support_anchor_authority;
  output.anchor_leak_suppression_enabled =
      frontend_output.anchor_leak_suppression_enabled;
  output.sub_support_leak_count = frontend_output.sub_support_leak_count;
  output.anchor_below_observation_count =
      frontend_output.anchor_below_observation_count;
  output.stale_anchor_residual_filtered_count =
      frontend_output.stale_anchor_residual_filtered_count;
  output.raw_sample_min_z = frontend_output.raw_sample_min_z;
  output.raw_sample_max_z = frontend_output.raw_sample_max_z;
  output.raw_sample_count = frontend_output.raw_sample_count;
  output.filtered_sample_min_z = frontend_output.filtered_sample_min_z;
  output.filtered_sample_max_z = frontend_output.filtered_sample_max_z;
  output.filtered_sample_count = frontend_output.filtered_sample_count;
  output.raw_upper_support_cell = frontend_output.raw_upper_support_cell;
  output.explanation_adjusted_upper_support_cell =
      frontend_output.explanation_adjusted_upper_support_cell;
  output.upper_support_cell = frontend_output.upper_support_cell;
  output.obstacle_local_triggered = frontend_output.obstacle_local_triggered;
  output.obstacle_upper_patch_confirmed =
      frontend_output.obstacle_upper_patch_confirmed;
  output.obstacle_explanation_rejected =
      frontend_output.obstacle_explanation_rejected;
  output.obstacle_suspicious = frontend_output.obstacle_suspicious;
  output.obstacle_candidate_cell = frontend_output.obstacle_candidate_cell;
  output.obstacle_rejected_by_neighbor_support =
      frontend_output.obstacle_rejected_by_neighbor_support;
  output.neighbor_upper_support_count =
      frontend_output.neighbor_upper_support_count;
  output.aligned_neighbor_support_count =
      frontend_output.aligned_neighbor_support_count;
  output.explanation_decision = frontend_output.explanation_decision;
  output.facade_lower_upper_coexisting =
      frontend_output.facade_lower_upper_coexisting;
  output.facade_upper_edge_aligned_with_supported_neighbors =
      frontend_output.facade_upper_edge_aligned_with_supported_neighbors;
  output.support_state = layers.support_state;

  if (config_.debug.publish_base_gravity_cloud) {
    output.base_gravity_cloud_points.reserve(frame.cloud_in_map.size());
  }
  output.support_points.reserve(frame.map_samples.size() / 4);
  output.obstacle_points.reserve(frame.map_samples.size() / 8);
  output.unknown_points.reserve(map_.size() / 4);
  const float support_tolerance =
      std::max(config_.preprocess.voxel_size * 1.5f, config_.map.resolution);
  std::unordered_map<int, float> fallback_support_ref_by_cell;
  fallback_support_ref_by_cell.reserve(frame.map_samples.size() / 8U + 1U);

  for (const auto &sample : frame.map_samples) {
    int cell = -1;
    if (!map_.mapToIndex(sample.point_in_map.x, sample.point_in_map.y, cell)) {
      continue;
    }
    if (layers.obstacle_publishable[cell] ==
            static_cast<uint8_t>(
                ObstaclePublishabilityState::kNotPublishable) ||
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
    if (config_.debug.publish_base_gravity_cloud) {
      output.base_gravity_cloud_points.push_back(
          MakeCellDebugPointWithoutSource(TransformMapPointToBaseGravity(
              sample.point_in_map, frame.base_pose_in_map)));
    }

    int cell = -1;
    if (!map_.mapToIndex(sample.point_in_map.x, sample.point_in_map.y, cell)) {
      continue;
    }

    if (layers.support_confidence[cell] > 0.15f &&
        IsNear(sample.point_in_map.z, layers.support_height[cell],
               support_tolerance)) {
      output.support_points.push_back(
          MakeCellDebugPoint(TransformMapPointToBaseGravity(
                                 sample.point_in_map, frame.base_pose_in_map),
                             cell));
    }

    if (IsPublishableObstacleCell(cell, sample, layers,
                                  fallback_support_ref_by_cell, config_)) {
      output.obstacle_points.push_back(
          MakeCellDebugPoint(TransformMapPointToBaseGravity(
                                 sample.point_in_map, frame.base_pose_in_map),
                             cell));
    }
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

  output.valid = true;
  return output;
}

} // namespace passable_area::core
