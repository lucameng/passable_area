#include "passable_area/interfaces/ros/converters/output_converter.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <limits>

namespace passable_area::interfaces::ros {
namespace {

nav_msgs::msg::OccupancyGrid CreateGrid(const passable_area::core::FrameOutput &output,
                                        const std_msgs::msg::Header &header,
                                        const std::vector<int8_t> &values) {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = header;
  msg.info.resolution = output.resolution;
  msg.info.width = output.cols;
  msg.info.height = output.rows;
  msg.info.origin.position.x = output.origin.x();
  msg.info.origin.position.y = output.origin.y();
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
  return CreateGrid(output, header, output.passability);
}

nav_msgs::msg::OccupancyGrid
OutputConverter::toTerrainCost(const passable_area::core::FrameOutput &output,
                               const std_msgs::msg::Header &header) const {
  return CreateGrid(output, header, output.traversal_cost);
}

grid_map_msgs::msg::GridMap OutputConverter::toGridMap(
    const passable_area::core::FrameOutput &output, const std_msgs::msg::Header &header) const {
  grid_map::GridMap map({"support_height", "support_confidence", "overhead_height",
                         "obstacle_evidence", "coverage_confidence", "slope", "step_up",
                         "step_down", "roughness", "clearance", "support_continuity",
                         "passability"});
  map.setFrameId(header.frame_id);
  map.setGeometry(grid_map::Length(output.cols * output.resolution, output.rows * output.resolution),
                  output.resolution,
                  grid_map::Position(output.origin.x() + output.cols * output.resolution * 0.5,
                                     output.origin.y() + output.rows * output.resolution * 0.5));
  AddLayer(map, "support_height", output.support_height, output.rows, output.cols);
  AddLayer(map, "support_confidence", output.support_confidence, output.rows, output.cols);
  AddLayer(map, "overhead_height", output.overhead_height, output.rows, output.cols);
  AddLayer(map, "obstacle_evidence", output.obstacle_evidence, output.rows, output.cols);
  AddLayer(map, "coverage_confidence", output.coverage_confidence, output.rows, output.cols);
  AddLayer(map, "slope", output.slope, output.rows, output.cols);
  AddLayer(map, "step_up", output.step_up, output.rows, output.cols);
  AddLayer(map, "step_down", output.step_down, output.rows, output.cols);
  AddLayer(map, "roughness", output.roughness, output.rows, output.cols);
  AddLayer(map, "clearance", output.clearance, output.rows, output.cols);
  AddLayer(map, "support_continuity", output.support_continuity, output.rows, output.cols);

  std::vector<float> passability_float(output.passability.begin(), output.passability.end());
  AddLayer(map, "passability", passability_float, output.rows, output.cols);

  auto msg = grid_map::GridMapRosConverter::toMessage(map);
  (void)header;
  return *msg;
}

} // namespace passable_area::interfaces::ros
