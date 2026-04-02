#include "passable_area/interfaces/ros/converters/output_converter.hpp"

#include "passable_area/core/utils/math_utils.hpp"

#include <grid_map_core/GridMap.hpp>
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

RobotCentricGeometry MakeRobotCentricGeometry(const passable_area::core::FrameOutput &output) {
  RobotCentricGeometry geometry;
  geometry.rows = output.rows;
  geometry.cols = output.cols;
  geometry.resolution = output.resolution;
  geometry.length = grid_map::Length(output.cols * output.resolution, output.rows * output.resolution);
  geometry.center = grid_map::Position(0.0, 0.0);
  geometry.origin = Eigen::Vector2f(-0.5f * output.cols * output.resolution,
                                    -0.5f * output.rows * output.resolution);
  return geometry;
}

Eigen::Vector2f CellCenter(const RobotCentricGeometry &geometry, int row, int col) {
  return Eigen::Vector2f(geometry.origin.x() + (static_cast<float>(col) + 0.5f) * geometry.resolution,
                         geometry.origin.y() + (static_cast<float>(row) + 0.5f) * geometry.resolution);
}

bool OdomCellIndexForBaseGravityCell(const passable_area::core::FrameOutput &output,
                                     const RobotCentricGeometry &geometry, int row, int col,
                                     int &source_index) {
  const Eigen::Vector2f point_in_base_gravity = CellCenter(geometry, row, col);
  const float yaw = passable_area::core::YawFromQuaternion(output.base_pose_in_odom.orientation);
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  const float odom_x = output.base_pose_in_odom.position.x() + cos_yaw * point_in_base_gravity.x() -
                       sin_yaw * point_in_base_gravity.y();
  const float odom_y = output.base_pose_in_odom.position.y() + sin_yaw * point_in_base_gravity.x() +
                       cos_yaw * point_in_base_gravity.y();
  const int source_col = static_cast<int>(std::floor((odom_x - output.origin.x()) / output.resolution));
  const int source_row = static_cast<int>(std::floor((odom_y - output.origin.y()) / output.resolution));
  if (source_row < 0 || source_row >= output.rows || source_col < 0 || source_col >= output.cols) {
    return false;
  }
  source_index = source_row * output.cols + source_col;
  return true;
}

template <typename T>
std::vector<T> ResampleLayer(const passable_area::core::FrameOutput &output,
                             const RobotCentricGeometry &geometry,
                             const std::vector<T> &source_values,
                             const T &default_value) {
  std::vector<T> resampled(static_cast<size_t>(geometry.rows * geometry.cols), default_value);
  for (int row = 0; row < geometry.rows; ++row) {
    for (int col = 0; col < geometry.cols; ++col) {
      int source_index = -1;
      if (!OdomCellIndexForBaseGravityCell(output, geometry, row, col, source_index)) {
        continue;
      }
      const int target_index = row * geometry.cols + col;
      resampled[target_index] = source_values[source_index];
    }
  }
  return resampled;
}

nav_msgs::msg::OccupancyGrid CreateGrid(const RobotCentricGeometry &geometry,
                                        const std_msgs::msg::Header &header,
                                        const std::vector<int8_t> &values) {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = header;
  msg.info.resolution = geometry.resolution;
  msg.info.width = geometry.cols;
  msg.info.height = geometry.rows;
  msg.info.origin.position.x = geometry.origin.x();
  msg.info.origin.position.y = geometry.origin.y();
  msg.info.origin.orientation.w = 1.0;
  msg.data = values;
  return msg;
}

void AddLayer(grid_map::GridMap &map, const std::string &name, const std::vector<float> &values,
              int rows, int cols) {
  grid_map::Matrix matrix(rows, cols);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      matrix(row, col) = values[row * cols + col];
    }
  }
  map.add(name, matrix);
}

} // namespace

nav_msgs::msg::OccupancyGrid
OutputConverter::toTerrainState(const passable_area::core::FrameOutput &output,
                                const std_msgs::msg::Header &header) const {
  const auto geometry = MakeRobotCentricGeometry(output);
  const auto passability =
      ResampleLayer(output, geometry, output.passability, static_cast<int8_t>(-1));
  return CreateGrid(geometry, header, passability);
}

nav_msgs::msg::OccupancyGrid
OutputConverter::toTerrainCost(const passable_area::core::FrameOutput &output,
                               const std_msgs::msg::Header &header) const {
  const auto geometry = MakeRobotCentricGeometry(output);
  const auto traversal_cost =
      ResampleLayer(output, geometry, output.traversal_cost, static_cast<int8_t>(-1));
  return CreateGrid(geometry, header, traversal_cost);
}

grid_map_msgs::msg::GridMap OutputConverter::toGridMap(
    const passable_area::core::FrameOutput &output, const std_msgs::msg::Header &header) const {
  const auto geometry = MakeRobotCentricGeometry(output);
  grid_map::GridMap map({"support_height", "support_confidence", "overhead_height",
                         "obstacle_evidence", "coverage_confidence", "slope", "step_up",
                         "step_down", "roughness", "clearance", "support_continuity",
                         "passability"});
  map.setFrameId(header.frame_id);
  map.setGeometry(geometry.length, geometry.resolution, geometry.center);
  AddLayer(map, "support_height",
           ResampleLayer(output, geometry, output.support_height,
                         std::numeric_limits<float>::quiet_NaN()),
           geometry.rows, geometry.cols);
  AddLayer(map, "support_confidence",
           ResampleLayer(output, geometry, output.support_confidence, 0.0f),
           geometry.rows, geometry.cols);
  AddLayer(map, "overhead_height",
           ResampleLayer(output, geometry, output.overhead_height,
                         std::numeric_limits<float>::quiet_NaN()),
           geometry.rows, geometry.cols);
  AddLayer(map, "obstacle_evidence",
           ResampleLayer(output, geometry, output.obstacle_evidence, 0.0f),
           geometry.rows, geometry.cols);
  AddLayer(map, "coverage_confidence",
           ResampleLayer(output, geometry, output.coverage_confidence, 0.0f),
           geometry.rows, geometry.cols);
  AddLayer(map, "slope", ResampleLayer(output, geometry, output.slope, 0.0f), geometry.rows,
           geometry.cols);
  AddLayer(map, "step_up", ResampleLayer(output, geometry, output.step_up, 0.0f), geometry.rows,
           geometry.cols);
  AddLayer(map, "step_down", ResampleLayer(output, geometry, output.step_down, 0.0f),
           geometry.rows, geometry.cols);
  AddLayer(map, "roughness", ResampleLayer(output, geometry, output.roughness, 0.0f),
           geometry.rows, geometry.cols);
  AddLayer(map, "clearance",
           ResampleLayer(output, geometry, output.clearance,
                         std::numeric_limits<float>::quiet_NaN()),
           geometry.rows, geometry.cols);
  AddLayer(map, "support_continuity",
           ResampleLayer(output, geometry, output.support_continuity, 0.0f),
           geometry.rows, geometry.cols);

  const auto passability =
      ResampleLayer(output, geometry, output.passability, static_cast<int8_t>(-1));
  std::vector<float> passability_float(passability.begin(), passability.end());
  AddLayer(map, "passability", passability_float, geometry.rows, geometry.cols);

  auto msg = grid_map::GridMapRosConverter::toMessage(map);
  (void)header;
  return *msg;
}

} // namespace passable_area::interfaces::ros
