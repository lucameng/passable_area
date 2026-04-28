#include "passable_area/interfaces/ros/passable_area_node.hpp"

#include "passable_area/interfaces/common/logging/node_logger.hpp"
#include "passable_area/msg/terrain_observability.hpp"

#include <gtest/gtest.h>

#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace passable_area::interfaces::ros {

class PassableAreaNodeRosTestAccess {
public:
  static void setNodeLogger(PassableAreaNode &node,
                            std::shared_ptr<NodeLogger> logger) {
    node.node_logger_ = std::move(logger);
  }
};

} // namespace passable_area::interfaces::ros

namespace {

using passable_area::interfaces::ros::NodeLogger;
using passable_area::interfaces::ros::PassableAreaNode;
using passable_area::interfaces::ros::PassableAreaNodeRosTestAccess;
using passable_area::msg::TerrainObservability;

void PrepareRosLogDir() {
  const auto log_dir = std::filesystem::path("/tmp/passable_area_test_logs");
  std::filesystem::create_directories(log_dir);
  ::setenv("ROS_LOG_DIR", log_dir.c_str(), 1);
}

class RosFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    PrepareRosLogDir();
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

sensor_msgs::msg::PointCloud2 MakeCloud(const std::string &frame_id,
                                        const rclcpp::Time &stamp,
                                        const std::vector<float> &xs) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const float x : xs) {
    cloud.emplace_back(x, 0.0f, 0.0f);
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  return msg;
}

nav_msgs::msg::Odometry MakeOdom(const std::string &frame_id,
                                 const rclcpp::Time &stamp) {
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.pose.pose.orientation.w = 1.0;
  return msg;
}

void SpinUntil(rclcpp::executors::SingleThreadedExecutor &executor,
               const std::function<bool()> &done) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline && !done()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

TEST_F(RosFixture, MergesLatestAuxiliaryCloudIntoPrimaryProcessing) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("input_cloud_topic", "/test/input_cloud"),
       rclcpp::Parameter("aux_input_cloud_topic", "/test/aux_cloud"),
       rclcpp::Parameter("odom_topic", "/test/odom"),
       rclcpp::Parameter("odom_frame", "odom"),
       rclcpp::Parameter("body_frame", "base_link"),
       rclcpp::Parameter("downsample.enable", false),
       rclcpp::Parameter("preprocess.body_filter.enable", false),
       rclcpp::Parameter("preprocess.crop_to_map.enable", false),
       rclcpp::Parameter("output.observability_topic", "/test/observability"),
       rclcpp::Parameter("output.terrain_state_topic", "/test/terrain_state"),
       rclcpp::Parameter("output.terrain_cost_topic", "/test/terrain_cost"),
       rclcpp::Parameter("output.debug_grid_map_topic", "/test/grid_map"),
       rclcpp::Parameter("output.support_points_topic", "/test/support_points"),
       rclcpp::Parameter("output.obstacle_points_topic",
                         "/test/obstacle_points"),
       rclcpp::Parameter("output.unknown_mask_topic", "/test/unknown_mask")});
  auto node = std::make_shared<PassableAreaNode>(options);
  auto io_node = std::make_shared<rclcpp::Node>("test_aux_cloud_io");

  TerrainObservability::SharedPtr received_observability;
  auto observability_sub =
      io_node->create_subscription<TerrainObservability>(
          "/test/observability", 10,
          [&](TerrainObservability::SharedPtr msg) {
            received_observability = std::move(msg);
          });
  auto aux_pub =
      io_node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/aux_cloud",
                                                               10);
  auto cloud_pub =
      io_node->create_publisher<sensor_msgs::msg::PointCloud2>("/test/input_cloud",
                                                               10);
  auto odom_pub =
      io_node->create_publisher<nav_msgs::msg::Odometry>("/test/odom", 10);

  (void)observability_sub;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(io_node);

  const auto stamp = io_node->now();
  aux_pub->publish(MakeCloud("base_link", stamp, {0.7f}));
  for (int i = 0; i < 5; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  cloud_pub->publish(MakeCloud("base_link", stamp, {0.3f, 0.5f}));
  odom_pub->publish(MakeOdom("odom", stamp));

  SpinUntil(executor, [&]() { return received_observability != nullptr; });
  ASSERT_TRUE(received_observability != nullptr);
  EXPECT_EQ(received_observability->base_point_count, 3U);
}

TEST_F(RosFixture, AuxiliaryCloudFrameMismatchWarnsButDoesNotBlockOutput) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("input_cloud_topic", "/test/input_cloud_warn"),
       rclcpp::Parameter("aux_input_cloud_topic", "/test/aux_cloud_warn"),
       rclcpp::Parameter("odom_topic", "/test/odom_warn"),
       rclcpp::Parameter("odom_frame", "odom"),
       rclcpp::Parameter("body_frame", "base_link"),
       rclcpp::Parameter("downsample.enable", false),
       rclcpp::Parameter("preprocess.body_filter.enable", false),
       rclcpp::Parameter("preprocess.crop_to_map.enable", false),
       rclcpp::Parameter("output.observability_topic",
                         "/test/observability_warn"),
       rclcpp::Parameter("output.terrain_state_topic",
                         "/test/terrain_state_warn"),
       rclcpp::Parameter("output.terrain_cost_topic", "/test/terrain_cost_warn"),
       rclcpp::Parameter("output.debug_grid_map_topic", "/test/grid_map_warn"),
       rclcpp::Parameter("output.support_points_topic",
                         "/test/support_points_warn"),
       rclcpp::Parameter("output.obstacle_points_topic",
                         "/test/obstacle_points_warn"),
       rclcpp::Parameter("output.unknown_mask_topic",
                         "/test/unknown_mask_warn")});
  auto node = std::make_shared<PassableAreaNode>(options);

  std::vector<std::pair<NodeLogger::Level, std::string>> logs;
  PassableAreaNodeRosTestAccess::setNodeLogger(
      *node, std::make_shared<NodeLogger>(
                 nullptr,
                 [&](NodeLogger::Level level, const std::string &message) {
                   logs.emplace_back(level, message);
                 },
                 []() { return 0.0; }));

  auto io_node = std::make_shared<rclcpp::Node>("test_aux_cloud_warn_io");
  TerrainObservability::SharedPtr received_observability;
  auto observability_sub =
      io_node->create_subscription<TerrainObservability>(
          "/test/observability_warn", 10,
          [&](TerrainObservability::SharedPtr msg) {
            received_observability = std::move(msg);
          });
  auto aux_pub = io_node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/test/aux_cloud_warn", 10);
  auto cloud_pub = io_node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/test/input_cloud_warn", 10);
  auto odom_pub = io_node->create_publisher<nav_msgs::msg::Odometry>(
      "/test/odom_warn", 10);

  (void)observability_sub;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(io_node);

  const auto stamp = io_node->now();
  aux_pub->publish(MakeCloud("aux_frame", stamp, {0.7f}));
  for (int i = 0; i < 5; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  cloud_pub->publish(MakeCloud("base_link", stamp, {0.3f, 0.5f}));
  odom_pub->publish(MakeOdom("odom", stamp));

  SpinUntil(executor, [&]() { return received_observability != nullptr; });
  ASSERT_TRUE(received_observability != nullptr);

  bool saw_aux_warning = false;
  for (const auto &[level, message] : logs) {
    if (level == NodeLogger::Level::kWarn &&
        message.find("Auxiliary PointCloud2 header.frame_id") !=
            std::string::npos &&
        message.find("body_frame='base_link'") != std::string::npos) {
      saw_aux_warning = true;
      break;
    }
  }
  EXPECT_TRUE(saw_aux_warning);
}

} // namespace
