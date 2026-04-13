#include "passable_area/interfaces/ros/converters/output_converter.hpp"

#include <gtest/gtest.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <cmath>
#include <limits>

namespace {

using passable_area::core::FrameOutput;
using passable_area::core::PassabilityState;
using passable_area::interfaces::ros::OutputConverter;

FrameOutput MakeOutput(float yaw_rad = 0.0f) {
  FrameOutput output;
  output.rows = 3;
  output.cols = 3;
  output.resolution = 1.0f;
  output.origin = Eigen::Vector2f(-1.5f, -1.5f);
  output.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  output.base_pose_in_odom.orientation =
      Eigen::Quaternionf(Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()));
  output.passability.assign(9, static_cast<int8_t>(PassabilityState::kUnknown));
  output.traversal_cost.assign(9, -1);
  output.support_height.assign(9, 0.0f);
  output.support_confidence.assign(9, 0.0f);
  output.overhead_height.assign(9, std::numeric_limits<float>::quiet_NaN());
  output.obstacle_evidence.assign(9, 0.0f);
  output.coverage_confidence.assign(9, 0.0f);
  output.slope.assign(9, 0.0f);
  output.step_up.assign(9, 0.0f);
  output.step_down.assign(9, 0.0f);
  output.roughness.assign(9, 0.0f);
  output.clearance.assign(9, std::numeric_limits<float>::quiet_NaN());
  output.support_continuity.assign(9, 0.0f);
  output.support_anchor_used.assign(9, std::numeric_limits<float>::quiet_NaN());
  output.sub_support_leak_count.assign(9, 0U);
  output.support_state.assign(9, 0U);
  return output;
}

int IndexForCellCenter(const nav_msgs::msg::OccupancyGrid &grid, float x,
                       float y) {
  const int col = static_cast<int>(
      std::floor((x - static_cast<float>(grid.info.origin.position.x)) /
                 grid.info.resolution));
  const int row = static_cast<int>(
      std::floor((y - static_cast<float>(grid.info.origin.position.y)) /
                 grid.info.resolution));
  return row * static_cast<int>(grid.info.width) + col;
}

grid_map::GridMap ToGridMap(const grid_map_msgs::msg::GridMap &msg) {
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);
  return map;
}

} // namespace

TEST(OutputConverterTest, GridOutputsUseBaseGravityFrame) {
  OutputConverter converter;
  auto output = MakeOutput();

  std_msgs::msg::Header header;
  header.frame_id = "base_gravity";

  const auto terrain_state = converter.toTerrainState(output, header);
  const auto terrain_cost = converter.toTerrainCost(output, header);
  const auto grid_map_msg = converter.toGridMap(output, header);
  const auto grid_map = ToGridMap(grid_map_msg);

  EXPECT_EQ(terrain_state.header.frame_id, "base_gravity");
  EXPECT_EQ(terrain_cost.header.frame_id, "base_gravity");
  EXPECT_EQ(grid_map.getFrameId(), "base_gravity");
}

TEST(OutputConverterTest, GridOutputsShareRobotCentricGeometry) {
  OutputConverter converter;
  auto output = MakeOutput();

  std_msgs::msg::Header header;
  header.frame_id = "base_gravity";

  const auto terrain_state = converter.toTerrainState(output, header);
  const auto terrain_cost = converter.toTerrainCost(output, header);
  const auto grid_map_msg = converter.toGridMap(output, header);
  const auto grid_map = ToGridMap(grid_map_msg);

  EXPECT_FLOAT_EQ(static_cast<float>(terrain_state.info.resolution), 1.0f);
  EXPECT_EQ(terrain_state.info.width, 3U);
  EXPECT_EQ(terrain_state.info.height, 3U);
  EXPECT_FLOAT_EQ(static_cast<float>(terrain_state.info.origin.position.x),
                  -1.5f);
  EXPECT_FLOAT_EQ(static_cast<float>(terrain_state.info.origin.position.y),
                  -1.5f);
  EXPECT_EQ(terrain_state.info.width, terrain_cost.info.width);
  EXPECT_EQ(terrain_state.info.height, terrain_cost.info.height);
  EXPECT_FLOAT_EQ(static_cast<float>(terrain_state.info.origin.position.x),
                  static_cast<float>(terrain_cost.info.origin.position.x));
  EXPECT_FLOAT_EQ(static_cast<float>(terrain_state.info.origin.position.y),
                  static_cast<float>(terrain_cost.info.origin.position.y));

  const auto length = grid_map.getLength();
  const auto position = grid_map.getPosition();
  EXPECT_FLOAT_EQ(static_cast<float>(length.x()), 3.0f);
  EXPECT_FLOAT_EQ(static_cast<float>(length.y()), 3.0f);
  EXPECT_FLOAT_EQ(static_cast<float>(position.x()), 0.0f);
  EXPECT_FLOAT_EQ(static_cast<float>(position.y()), 0.0f);
}

TEST(OutputConverterTest, GridOutputsRespectNonZeroYawResampling) {
  OutputConverter converter;
  auto output = MakeOutput(static_cast<float>(M_PI_2));

  const int source_index = 2 * output.cols + 1; // odom center (0, 1)
  output.passability[source_index] =
      static_cast<int8_t>(PassabilityState::kImpassable);
  output.traversal_cost[source_index] = 42;

  std_msgs::msg::Header header;
  header.frame_id = "base_gravity";

  const auto terrain_state = converter.toTerrainState(output, header);
  const auto terrain_cost = converter.toTerrainCost(output, header);

  const int target_index = IndexForCellCenter(terrain_state, 1.0f, 0.0f);
  ASSERT_GE(target_index, 0);
  ASSERT_LT(target_index, static_cast<int>(terrain_state.data.size()));
  EXPECT_EQ(terrain_state.data[target_index],
            static_cast<int8_t>(PassabilityState::kImpassable));
  EXPECT_EQ(terrain_cost.data[target_index], 42);
}

TEST(OutputConverterTest, GridMapAndOccupancyOutputsStayAligned) {
  OutputConverter converter;
  auto output = MakeOutput();
  const int center_index = 1 * output.cols + 1;
  output.passability[center_index] =
      static_cast<int8_t>(PassabilityState::kPassable);
  output.traversal_cost[center_index] = 7;

  std_msgs::msg::Header header;
  header.frame_id = "base_gravity";

  const auto terrain_state = converter.toTerrainState(output, header);
  const auto terrain_cost = converter.toTerrainCost(output, header);
  const auto grid_map_msg = converter.toGridMap(output, header);
  const auto grid_map = ToGridMap(grid_map_msg);

  const int occupancy_index = IndexForCellCenter(terrain_state, 0.0f, 0.0f);
  ASSERT_GE(occupancy_index, 0);
  EXPECT_EQ(terrain_state.data[occupancy_index],
            static_cast<int8_t>(PassabilityState::kPassable));
  EXPECT_EQ(terrain_cost.data[occupancy_index], 7);
  EXPECT_FLOAT_EQ(
      grid_map.atPosition("passability", grid_map::Position(0.0, 0.0)),
      static_cast<float>(static_cast<int8_t>(PassabilityState::kPassable)));
}

TEST(OutputConverterTest,
     GridMapUsesSameRobotCentricDirectionAsOccupancyOutputs) {
  OutputConverter converter;
  auto output = MakeOutput();

  const int forward_index = 2 * output.cols + 1; // odom/base_gravity (0, 1)
  output.passability[forward_index] =
      static_cast<int8_t>(PassabilityState::kImpassable);
  output.support_height[forward_index] = 3.0f;

  std_msgs::msg::Header header;
  header.frame_id = "base_gravity";

  const auto terrain_state = converter.toTerrainState(output, header);
  const auto grid_map_msg = converter.toGridMap(output, header);
  const auto grid_map = ToGridMap(grid_map_msg);

  const int forward_target_index =
      IndexForCellCenter(terrain_state, 0.0f, 1.0f);
  ASSERT_GE(forward_target_index, 0);
  ASSERT_LT(forward_target_index, static_cast<int>(terrain_state.data.size()));
  EXPECT_EQ(terrain_state.data[forward_target_index],
            static_cast<int8_t>(PassabilityState::kImpassable));
  EXPECT_FLOAT_EQ(
      grid_map.atPosition("passability", grid_map::Position(0.0, 1.0)),
      static_cast<float>(static_cast<int8_t>(PassabilityState::kImpassable)));
  EXPECT_FLOAT_EQ(
      grid_map.atPosition("support_height", grid_map::Position(0.0, 1.0)),
      3.0f);
}
