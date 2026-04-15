#include "passable_area/interfaces/ros/passable_area_node.hpp"

#include "passable_area/interfaces/common/logging/node_logger.hpp"

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <optional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace passable_area::interfaces::ros {

class PassableAreaNodeTestAccess {
public:
  static void onSynced(
      PassableAreaNode &node,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
      const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg) {
    node.onSynced(cloud_msg, odom_msg);
  }

  static void setNodeLogger(PassableAreaNode &node,
                            std::shared_ptr<NodeLogger> logger) {
    node.node_logger_ = std::move(logger);
  }
};

} // namespace passable_area::interfaces::ros

namespace {

using passable_area::interfaces::ros::NodeLogger;
using passable_area::interfaces::ros::PassableAreaNode;
using passable_area::interfaces::ros::PassableAreaNodeTestAccess;

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

sensor_msgs::msg::PointCloud2 MakeCloud(const std::string &frame_id,
                                        const rclcpp::Time &stamp) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.emplace_back(0.6f, 0.0f, 0.0f);
  cloud.emplace_back(0.9f, 0.2f, 0.0f);

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
  msg.pose.pose.position.x = 1.0;
  msg.pose.pose.position.y = 2.0;
  msg.pose.pose.position.z = 0.5;
  msg.pose.pose.orientation.w = 1.0;
  return msg;
}

void SpinUntil(rclcpp::executors::SingleThreadedExecutor &executor,
               const std::function<bool()> &done) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
  while (std::chrono::steady_clock::now() < deadline && !done()) {
    executor.spin_some();
  }
}

} // namespace

TEST_F(RosFixture, DefaultConfigDoesNotPublishMapToBaseGravityTf) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("input_cloud_topic", "/test/input_cloud_unused"),
       rclcpp::Parameter("odom_topic", "/test/odom_unused"),
       rclcpp::Parameter("output.terrain_state_topic", "/test/terrain_state"),
       rclcpp::Parameter("output.terrain_cost_topic", "/test/terrain_cost"),
       rclcpp::Parameter("output.debug_grid_map_topic", "/test/grid_map"),
       rclcpp::Parameter("output.support_points_topic", "/test/support_points"),
       rclcpp::Parameter("output.obstacle_points_topic",
                         "/test/obstacle_points"),
       rclcpp::Parameter("output.unknown_mask_topic", "/test/unknown_mask"),
       rclcpp::Parameter("output.observability_topic", "/test/observability"),
       rclcpp::Parameter("debug.publish_base_gravity_cloud", false)});
  auto node = std::make_shared<PassableAreaNode>(options);

  auto observer = std::make_shared<rclcpp::Node>("test_tf_observer_default");
  tf2_msgs::msg::TFMessage::SharedPtr received_tf;
  auto tf_sub = observer->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, [&](tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received_tf = std::move(msg);
      });

  (void)tf_sub;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);

  const auto stamp = observer->now();
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(
      MakeCloud("base_link", stamp));
  auto odom =
      std::make_shared<nav_msgs::msg::Odometry>(MakeOdom("map", stamp));
  PassableAreaNodeTestAccess::onSynced(*node, cloud, odom);

  SpinUntil(executor, [&]() { return received_tf != nullptr; });
  EXPECT_FALSE(received_tf);
}

TEST_F(RosFixture, EnabledConfigPublishesConfiguredMapToBaseGravityTf) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map_frame", "test_map"),
       rclcpp::Parameter("base_gravity_frame", "test_base_gravity"),
       rclcpp::Parameter("debug.publish_map_to_base_gravity_tf", true),
       rclcpp::Parameter("input_cloud_topic", "/test/input_cloud_unused"),
       rclcpp::Parameter("odom_topic", "/test/odom_unused"),
       rclcpp::Parameter("output.terrain_state_topic", "/test/terrain_state"),
       rclcpp::Parameter("output.terrain_cost_topic", "/test/terrain_cost"),
       rclcpp::Parameter("output.debug_grid_map_topic", "/test/grid_map"),
       rclcpp::Parameter("output.support_points_topic", "/test/support_points"),
       rclcpp::Parameter("output.obstacle_points_topic",
                         "/test/obstacle_points"),
       rclcpp::Parameter("output.unknown_mask_topic", "/test/unknown_mask"),
       rclcpp::Parameter("output.observability_topic", "/test/observability"),
       rclcpp::Parameter("debug.publish_base_gravity_cloud", false)});
  auto node = std::make_shared<PassableAreaNode>(options);

  auto observer = std::make_shared<rclcpp::Node>("test_tf_observer_enabled");
  std::optional<geometry_msgs::msg::TransformStamped> received_transform;
  auto tf_sub = observer->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, [&](tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto &transform : msg->transforms) {
          if (transform.header.frame_id == "test_map" &&
              transform.child_frame_id == "test_base_gravity") {
            received_transform = transform;
            break;
          }
        }
      });

  (void)tf_sub;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);

  const auto stamp = observer->now();
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(
      MakeCloud("base_link", stamp));
  auto odom =
      std::make_shared<nav_msgs::msg::Odometry>(MakeOdom("test_map", stamp));
  PassableAreaNodeTestAccess::onSynced(*node, cloud, odom);

  SpinUntil(executor, [&]() { return received_transform.has_value(); });
  ASSERT_TRUE(received_transform.has_value());
  const auto &transform = *received_transform;
  EXPECT_EQ(transform.header.frame_id, "test_map");
  EXPECT_EQ(transform.child_frame_id, "test_base_gravity");
  EXPECT_FLOAT_EQ(transform.transform.translation.x, 1.0f);
  EXPECT_FLOAT_EQ(transform.transform.translation.y, 2.0f);
  EXPECT_FLOAT_EQ(transform.transform.translation.z, 0.5f);
}

TEST_F(RosFixture, MismatchedInputFramesProduceWarnings) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map_frame", "test_map"),
       rclcpp::Parameter("body_frame", "test_body"),
       rclcpp::Parameter("input_cloud_topic", "/test/input_cloud_unused"),
       rclcpp::Parameter("odom_topic", "/test/odom_unused"),
       rclcpp::Parameter("output.terrain_state_topic", "/test/terrain_state"),
       rclcpp::Parameter("output.terrain_cost_topic", "/test/terrain_cost"),
       rclcpp::Parameter("output.debug_grid_map_topic", "/test/grid_map"),
       rclcpp::Parameter("output.support_points_topic", "/test/support_points"),
       rclcpp::Parameter("output.obstacle_points_topic",
                         "/test/obstacle_points"),
       rclcpp::Parameter("output.unknown_mask_topic", "/test/unknown_mask"),
       rclcpp::Parameter("output.observability_topic", "/test/observability"),
       rclcpp::Parameter("debug.publish_base_gravity_cloud", false)});
  auto node = std::make_shared<PassableAreaNode>(options);

  std::vector<std::pair<NodeLogger::Level, std::string>> logs;
  PassableAreaNodeTestAccess::setNodeLogger(
      *node, std::make_shared<NodeLogger>(
      nullptr,
      [&](NodeLogger::Level level, const std::string &message) {
        logs.emplace_back(level, message);
      },
      []() { return 0.0; }));

  const auto stamp = node->now();
  auto cloud =
      std::make_shared<sensor_msgs::msg::PointCloud2>(MakeCloud("base_link", stamp));
  auto odom =
      std::make_shared<nav_msgs::msg::Odometry>(MakeOdom("odom", stamp));
  PassableAreaNodeTestAccess::onSynced(*node, cloud, odom);

  bool saw_map_warning = false;
  for (const auto &[level, message] : logs) {
    if (level == NodeLogger::Level::kWarn &&
        message.find("map_frame='test_map'") != std::string::npos) {
      saw_map_warning = true;
      break;
    }
  }
  EXPECT_TRUE(saw_map_warning);

  logs.clear();
  auto cloud_mismatch = std::make_shared<sensor_msgs::msg::PointCloud2>(
      MakeCloud("base_link", stamp + rclcpp::Duration::from_seconds(0.01)));
  auto odom_match = std::make_shared<nav_msgs::msg::Odometry>(
      MakeOdom("test_map", stamp + rclcpp::Duration::from_seconds(0.01)));
  PassableAreaNodeTestAccess::onSynced(*node, cloud_mismatch, odom_match);

  bool saw_body_warning = false;
  for (const auto &[level, message] : logs) {
    if (level == NodeLogger::Level::kWarn &&
        message.find("body_frame='test_body'") != std::string::npos) {
      saw_body_warning = true;
      break;
    }
  }
  EXPECT_TRUE(saw_body_warning);
}
