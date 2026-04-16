#include "passable_area/interfaces/ros/converters/output_converter.hpp"
#include "passable_area/interfaces/ros/runtime/debug_publishers.hpp"

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

namespace {

using passable_area::core::CellDebugPoint;
using passable_area::core::FrameOutput;
using passable_area::core::ObservabilityState;
using passable_area::core::PassabilityState;
using passable_area::interfaces::ros::DebugPublishers;
using passable_area::interfaces::ros::OutputConverter;
using passable_area::interfaces::ros::RosTopicConfig;

class RosFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

FrameOutput MakeDebugOutput() {
  FrameOutput output;
  output.rows = 3;
  output.cols = 3;
  output.resolution = 1.0f;
  output.origin = Eigen::Vector2f(-1.5f, -1.5f);
  output.base_pose_in_map.position = Eigen::Vector3f::Zero();
  output.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  output.passability.assign(9, static_cast<int8_t>(PassabilityState::kUnknown));
  output.traversal_cost.assign(9, -1);
  output.support_height.assign(9, 0.0f);
  output.overhead_height.assign(9, std::numeric_limits<float>::quiet_NaN());
  output.support_confidence.assign(9, 0.0f);
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
  output.unknown_points.push_back(
      passable_area::core::MakeCellDebugPointWithoutSource({0.0f, 0.0f, 0.0f}));
  output.observability.sectors.resize(1);
  output.observability.sectors.front().state = ObservabilityState::kObserved;
  output.observability.sectors.front().coverage_confidence = 1.0f;
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

} // namespace

TEST_F(RosFixture, UnknownMaskAndObservabilityStayInBaseGravityContext) {
  auto node = std::make_shared<rclcpp::Node>("test_debug_publishers");
  RosTopicConfig topics;
  topics.unknown_mask_topic = "/test/unknown_mask";
  topics.observability_topic = "/test/observability";
  topics.support_points_topic = "/test/support_points";
  topics.obstacle_points_topic = "/test/obstacle_points";
  topics.base_gravity_cloud_topic = "/test/base_gravity_cloud";

  passable_area::core::DebugConfig debug_config;
  debug_config.publish_base_gravity_cloud = false;

  DebugPublishers publishers;
  publishers.initialize(*node, topics, debug_config);

  sensor_msgs::msg::PointCloud2::SharedPtr received_unknown;
  passable_area::msg::TerrainObservability::SharedPtr received_observability;
  auto unknown_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      topics.unknown_mask_topic, 10,
      [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        received_unknown = std::move(msg);
      });
  auto observability_sub =
      node->create_subscription<passable_area::msg::TerrainObservability>(
          topics.observability_topic, 10,
          [&](passable_area::msg::TerrainObservability::SharedPtr msg) {
            received_observability = std::move(msg);
          });

  OutputConverter converter;
  const auto output = MakeDebugOutput();
  std_msgs::msg::Header header;
  header.frame_id = "base_gravity";
  publishers.publish(output, header);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline &&
         (!received_unknown || !received_observability)) {
    executor.spin_some();
  }

  ASSERT_TRUE(received_unknown);
  ASSERT_TRUE(received_observability);
  EXPECT_EQ(received_unknown->header.frame_id, "base_gravity");
  EXPECT_EQ(received_observability->header.frame_id, "base_gravity");

  pcl::PointCloud<pcl::PointXYZ> unknown_cloud;
  pcl::fromROSMsg(*received_unknown, unknown_cloud);
  ASSERT_EQ(unknown_cloud.size(), 1U);
  EXPECT_FLOAT_EQ(unknown_cloud.front().x, 0.0f);
  EXPECT_FLOAT_EQ(unknown_cloud.front().y, 0.0f);

  const auto terrain_state = converter.toTerrainState(output, header);
  const int center_index = IndexForCellCenter(terrain_state, 0.0f, 0.0f);
  ASSERT_GE(center_index, 0);
  ASSERT_LT(center_index, static_cast<int>(terrain_state.data.size()));
  EXPECT_EQ(terrain_state.data[center_index],
            static_cast<int8_t>(PassabilityState::kUnknown));
}
