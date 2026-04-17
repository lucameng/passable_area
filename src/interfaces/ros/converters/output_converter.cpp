#include "passable_area/interfaces/ros/converters/output_converter.hpp"

#include "passable_area/core/utils/math_utils.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <cmath>
#include <limits>

namespace passable_area::interfaces::ros {
namespace {

struct RobotCentricGeometry {
  int rows = 0;
  int cols = 0;
  float resolution = 0.1f;
  Eigen::Vector2f origin = Eigen::Vector2f::Zero();
  grid_map::Length length = grid_map::Length::Zero();
  grid_map::Position center = grid_map::Position::Zero();
};

struct RobotCentricResamplingPlan {
  RobotCentricGeometry geometry;
  std::vector<int> source_indices;
  std::vector<grid_map::Index> grid_map_indices;
};

RobotCentricGeometry
MakeRobotCentricGeometry(const passable_area::core::FrameOutput &output) {
  RobotCentricGeometry geometry;
  geometry.rows = output.rows;
  geometry.cols = output.cols;
  geometry.resolution = output.resolution;
  geometry.length = grid_map::Length(output.cols * output.resolution,
                                     output.rows * output.resolution);
  geometry.center = grid_map::Position(0.0, 0.0);
  geometry.origin = Eigen::Vector2f(-0.5f * output.cols * output.resolution,
                                    -0.5f * output.rows * output.resolution);
  return geometry;
}

Eigen::Vector2f CellCenter(const RobotCentricGeometry &geometry, int row,
                           int col) {
  return Eigen::Vector2f(
      geometry.origin.x() +
          (static_cast<float>(col) + 0.5f) * geometry.resolution,
      geometry.origin.y() +
          (static_cast<float>(row) + 0.5f) * geometry.resolution);
}

bool MapCellIndexForBaseGravityCell(
    const passable_area::core::FrameOutput &output,
    const RobotCentricGeometry &geometry, int row, int col, int &source_index) {
  const Eigen::Vector2f point_in_base_gravity = CellCenter(geometry, row, col);
  const float yaw = passable_area::core::YawFromQuaternion(
      output.base_pose_in_map.orientation);
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  // output.base_pose_in_map keeps a historical name; its numeric parent-frame
  // semantics follow map after the map-frame cleanup.
  const float map_x = output.base_pose_in_map.position.x() +
                      cos_yaw * point_in_base_gravity.x() -
                      sin_yaw * point_in_base_gravity.y();
  const float map_y = output.base_pose_in_map.position.y() +
                      sin_yaw * point_in_base_gravity.x() +
                      cos_yaw * point_in_base_gravity.y();
  const int source_col = static_cast<int>(
      std::floor((map_x - output.origin.x()) / output.resolution));
  const int source_row = static_cast<int>(
      std::floor((map_y - output.origin.y()) / output.resolution));
  if (source_row < 0 || source_row >= output.rows || source_col < 0 ||
      source_col >= output.cols) {
    return false;
  }
  source_index = source_row * output.cols + source_col;
  return true;
}

RobotCentricResamplingPlan
MakeResamplingPlan(const passable_area::core::FrameOutput &output) {
  RobotCentricResamplingPlan plan;
  plan.geometry = MakeRobotCentricGeometry(output);
  const int cell_count = plan.geometry.rows * plan.geometry.cols;
  plan.source_indices.assign(static_cast<size_t>(cell_count), -1);
  plan.grid_map_indices.resize(static_cast<size_t>(cell_count),
                               grid_map::Index::Zero());
  const grid_map::Size buffer_size(plan.geometry.rows, plan.geometry.cols);

  for (int row = 0; row < plan.geometry.rows; ++row) {
    for (int col = 0; col < plan.geometry.cols; ++col) {
      const int target_index = row * plan.geometry.cols + col;
      MapCellIndexForBaseGravityCell(output, plan.geometry, row, col,
                                     plan.source_indices[target_index]);

      const auto position = CellCenter(plan.geometry, row, col);
      grid_map::Index index;
      const bool inside = grid_map::getIndexFromPosition(
          index, grid_map::Position(position.x(), position.y()),
          plan.geometry.length, plan.geometry.center, plan.geometry.resolution,
          buffer_size);
      if (!inside) {
        throw std::runtime_error(
            "Robot-centric grid_map geometry lookup failed");
      }
      plan.grid_map_indices[target_index] = index;
    }
  }

  return plan;
}

template <typename T>
std::vector<T> ResampleLayer(const RobotCentricResamplingPlan &plan,
                             const std::vector<T> &source_values,
                             const T &default_value) {
  std::vector<T> resampled(plan.source_indices.size(), default_value);
  for (size_t target_index = 0; target_index < plan.source_indices.size();
       ++target_index) {
    const int source_index = plan.source_indices[target_index];
    if (source_index < 0 ||
        static_cast<size_t>(source_index) >= source_values.size()) {
      continue;
    }
    resampled[target_index] = source_values[static_cast<size_t>(source_index)];
  }
  return resampled;
}

nav_msgs::msg::OccupancyGrid CreateGrid(const RobotCentricResamplingPlan &plan,
                                        const std_msgs::msg::Header &header,
                                        const std::vector<int8_t> &values) {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = header;
  msg.info.resolution = plan.geometry.resolution;
  msg.info.width = plan.geometry.cols;
  msg.info.height = plan.geometry.rows;
  msg.info.origin.position.x = plan.geometry.origin.x();
  msg.info.origin.position.y = plan.geometry.origin.y();
  msg.info.origin.orientation.w = 1.0;
  msg.data = values;
  return msg;
}

void AddLayer(grid_map::GridMap &map, const std::string &name,
              const RobotCentricResamplingPlan &plan,
              const std::vector<float> &values) {
  grid_map::Matrix matrix(plan.geometry.rows, plan.geometry.cols);
  matrix.setConstant(std::numeric_limits<float>::quiet_NaN());
  for (size_t target_index = 0; target_index < values.size(); ++target_index) {
    const auto &index = plan.grid_map_indices[target_index];
    matrix(index(0), index(1)) = values[target_index];
  }
  map.add(name, matrix);
}

} // namespace

nav_msgs::msg::OccupancyGrid
OutputConverter::toTerrainState(const passable_area::core::FrameOutput &output,
                                const std_msgs::msg::Header &header) const {
  return toMapOutputs(output, header).terrain_state;
}

nav_msgs::msg::OccupancyGrid
OutputConverter::toTerrainCost(const passable_area::core::FrameOutput &output,
                               const std_msgs::msg::Header &header) const {
  return toMapOutputs(output, header).terrain_cost;
}

ConvertedMapOutputs
OutputConverter::toMapOutputs(const passable_area::core::FrameOutput &output,
                              const std_msgs::msg::Header &header) const {
  const auto plan = MakeResamplingPlan(output);
  const auto passability =
      ResampleLayer(plan, output.passability, static_cast<int8_t>(-1));
  const auto traversal_cost =
      ResampleLayer(plan, output.traversal_cost, static_cast<int8_t>(-1));
  grid_map::GridMap map(
      {"support_height", "support_confidence", "overhead_height",
       "obstacle_evidence", "coverage_confidence", "slope", "step_up",
       "step_down", "roughness", "clearance", "support_continuity",
       "obstacle_publishable", "support_anchor_used",
       "sub_support_leak_count", "passability"});
  map.setFrameId(header.frame_id);
  map.setGeometry(plan.geometry.length, plan.geometry.resolution,
                  plan.geometry.center);
  AddLayer(map, "support_height", plan,
           ResampleLayer(plan, output.support_height,
                         std::numeric_limits<float>::quiet_NaN()));
  AddLayer(map, "support_confidence", plan,
           ResampleLayer(plan, output.support_confidence, 0.0f));
  AddLayer(map, "overhead_height", plan,
           ResampleLayer(plan, output.overhead_height,
                         std::numeric_limits<float>::quiet_NaN()));
  AddLayer(map, "obstacle_evidence", plan,
           ResampleLayer(plan, output.obstacle_evidence, 0.0f));
  AddLayer(map, "coverage_confidence", plan,
           ResampleLayer(plan, output.coverage_confidence, 0.0f));
  AddLayer(map, "slope", plan, ResampleLayer(plan, output.slope, 0.0f));
  AddLayer(map, "step_up", plan, ResampleLayer(plan, output.step_up, 0.0f));
  AddLayer(map, "step_down", plan, ResampleLayer(plan, output.step_down, 0.0f));
  AddLayer(map, "roughness", plan, ResampleLayer(plan, output.roughness, 0.0f));
  AddLayer(map, "clearance", plan,
           ResampleLayer(plan, output.clearance,
                         std::numeric_limits<float>::quiet_NaN()));
  AddLayer(map, "support_continuity", plan,
           ResampleLayer(plan, output.support_continuity, 0.0f));
  std::vector<float> obstacle_publishable_float(output.obstacle_publishable.size(),
                                                0.0f);
  for (size_t i = 0; i < output.obstacle_publishable.size(); ++i) {
    obstacle_publishable_float[i] =
        output.obstacle_publishable[i] != 0U ? 1.0f : 0.0f;
  }
  AddLayer(map, "obstacle_publishable", plan,
           ResampleLayer(plan, obstacle_publishable_float, 0.0f));
  AddLayer(map, "support_anchor_used", plan,
           ResampleLayer(plan, output.support_anchor_used,
                         std::numeric_limits<float>::quiet_NaN()));
  std::vector<float> sub_support_leak_count_float(
      output.sub_support_leak_count.size(), 0.0f);
  for (size_t i = 0; i < output.sub_support_leak_count.size(); ++i) {
    sub_support_leak_count_float[i] =
        static_cast<float>(output.sub_support_leak_count[i]);
  }
  AddLayer(map, "sub_support_leak_count", plan,
           ResampleLayer(plan, sub_support_leak_count_float, 0.0f));

  std::vector<float> passability_float(passability.begin(), passability.end());
  AddLayer(map, "passability", plan, passability_float);

  auto msg = grid_map::GridMapRosConverter::toMessage(map);
  ConvertedMapOutputs outputs;
  outputs.terrain_state = CreateGrid(plan, header, passability);
  outputs.terrain_cost = CreateGrid(plan, header, traversal_cost);
  outputs.grid_map = *msg;
  return outputs;
}

grid_map_msgs::msg::GridMap
OutputConverter::toGridMap(const passable_area::core::FrameOutput &output,
                           const std_msgs::msg::Header &header) const {
  return toMapOutputs(output, header).grid_map;
}

} // namespace passable_area::interfaces::ros
