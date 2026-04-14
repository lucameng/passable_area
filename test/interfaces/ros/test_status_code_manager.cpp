#include "passable_area/interfaces/common/status_code.hpp"
#include "passable_area/interfaces/ros/runtime/status_code_manager.hpp"

#include <gtest/gtest.h>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/int32.hpp>

#include <chrono>
#include <limits>
#include <memory>
#include <thread>

namespace {

using passable_area::interfaces::common::StatusCode;
using passable_area::interfaces::common::toInt;
using passable_area::interfaces::ros::RosTopicConfig;
using passable_area::interfaces::ros::StatusCodeManager;
using passable_area::interfaces::ros::StatusCodeManagerOptions;

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

int32_t WaitForStatus(rclcpp::executors::SingleThreadedExecutor &executor,
                      int32_t &received, int32_t expected) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (received == expected) {
      return received;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return received;
}

} // namespace

TEST_F(RosFixture, PublishesOkRunningWhenInputsAndOutputsStayHealthy) {
  auto node = std::make_shared<rclcpp::Node>("test_status_code_ok");
  RosTopicConfig topics;
  topics.status_code_topic = "/test/status_code_ok";

  StatusCodeManager manager;
  StatusCodeManagerOptions options;
  options.input_timeout_sec = 0.2;
  options.sync_timeout_sec = 0.2;
  options.publish_period = std::chrono::milliseconds(10);
  manager.initialize(*node, topics, nullptr, options);

  int32_t received = std::numeric_limits<int32_t>::min();
  auto sub = node->create_subscription<std_msgs::msg::Int32>(
      topics.status_code_topic, 10,
      [&](const std_msgs::msg::Int32::SharedPtr msg) { received = msg->data; });

  (void)sub;
  const auto stamp = node->now();
  manager.markCloud(stamp);
  manager.markOdom(stamp);
  manager.markSynced(stamp);
  manager.markOutputSuccess();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  EXPECT_EQ(WaitForStatus(executor, received, toInt(StatusCode::OK_RUNNING)),
            toInt(StatusCode::OK_RUNNING));
  EXPECT_EQ(manager.currentStatusCode(), toInt(StatusCode::OK_RUNNING));
}

TEST_F(RosFixture, PublishesOdomTimeoutWhenCloudIsFreshButOdomMissing) {
  auto node = std::make_shared<rclcpp::Node>("test_status_code_odom_timeout");
  RosTopicConfig topics;
  topics.status_code_topic = "/test/status_code_odom_timeout";

  StatusCodeManager manager;
  StatusCodeManagerOptions options;
  options.input_timeout_sec = 0.08;
  options.sync_timeout_sec = 0.2;
  options.publish_period = std::chrono::milliseconds(10);
  manager.initialize(*node, topics, nullptr, options);

  int32_t received = std::numeric_limits<int32_t>::min();
  auto sub = node->create_subscription<std_msgs::msg::Int32>(
      topics.status_code_topic, 10,
      [&](const std_msgs::msg::Int32::SharedPtr msg) { received = msg->data; });

  (void)sub;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  manager.markCloud(node->now());

  EXPECT_EQ(WaitForStatus(executor, received, toInt(StatusCode::ERR_ODOM_TIMEOUT)),
            toInt(StatusCode::ERR_ODOM_TIMEOUT));
  EXPECT_EQ(manager.currentStatusCode(), toInt(StatusCode::ERR_ODOM_TIMEOUT));
}

TEST_F(RosFixture, PublishesSyncStallBeforeInputTimeoutsExpire) {
  auto node = std::make_shared<rclcpp::Node>("test_status_code_sync_stall");
  RosTopicConfig topics;
  topics.status_code_topic = "/test/status_code_sync_stall";

  StatusCodeManager manager;
  StatusCodeManagerOptions options;
  options.input_timeout_sec = 0.25;
  options.sync_timeout_sec = 0.06;
  options.publish_period = std::chrono::milliseconds(10);
  manager.initialize(*node, topics, nullptr, options);

  int32_t received = std::numeric_limits<int32_t>::min();
  auto sub = node->create_subscription<std_msgs::msg::Int32>(
      topics.status_code_topic, 10,
      [&](const std_msgs::msg::Int32::SharedPtr msg) { received = msg->data; });

  (void)sub;
  const auto stamp = node->now();
  manager.markCloud(stamp);
  manager.markOdom(stamp);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  EXPECT_EQ(WaitForStatus(executor, received, toInt(StatusCode::ERR_SYNC_STALL)),
            toInt(StatusCode::ERR_SYNC_STALL));
  EXPECT_EQ(manager.currentStatusCode(), toInt(StatusCode::ERR_SYNC_STALL));
}

TEST_F(RosFixture, PublishesOutputStallAfterConsecutiveOutputFailures) {
  auto node = std::make_shared<rclcpp::Node>("test_status_code_output_stall");
  RosTopicConfig topics;
  topics.status_code_topic = "/test/status_code_output_stall";

  StatusCodeManager manager;
  StatusCodeManagerOptions options;
  options.input_timeout_sec = 0.2;
  options.sync_timeout_sec = 0.2;
  options.output_stall_frames = 3;
  options.publish_period = std::chrono::milliseconds(10);
  manager.initialize(*node, topics, nullptr, options);

  int32_t received = std::numeric_limits<int32_t>::min();
  auto sub = node->create_subscription<std_msgs::msg::Int32>(
      topics.status_code_topic, 10,
      [&](const std_msgs::msg::Int32::SharedPtr msg) { received = msg->data; });

  (void)sub;
  const auto stamp = node->now();
  manager.markCloud(stamp);
  manager.markOdom(stamp);
  manager.markSynced(stamp);
  manager.markOutputFailure();
  manager.markOutputFailure();
  manager.markOutputFailure();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  EXPECT_EQ(WaitForStatus(executor, received, toInt(StatusCode::ERR_OUTPUT_STALL)),
            toInt(StatusCode::ERR_OUTPUT_STALL));
  EXPECT_EQ(manager.currentStatusCode(), toInt(StatusCode::ERR_OUTPUT_STALL));
}

TEST_F(RosFixture, StickyFatalOverridesRuntimeHealth) {
  auto node = std::make_shared<rclcpp::Node>("test_status_code_fatal");
  RosTopicConfig topics;
  topics.status_code_topic = "/test/status_code_fatal";

  StatusCodeManager manager;
  StatusCodeManagerOptions options;
  options.input_timeout_sec = 0.2;
  options.sync_timeout_sec = 0.2;
  options.publish_period = std::chrono::milliseconds(10);
  manager.initialize(*node, topics, nullptr, options);

  int32_t received = std::numeric_limits<int32_t>::min();
  auto sub = node->create_subscription<std_msgs::msg::Int32>(
      topics.status_code_topic, 10,
      [&](const std_msgs::msg::Int32::SharedPtr msg) { received = msg->data; });

  (void)sub;
  manager.markFatalRuntimeException("test");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  EXPECT_EQ(WaitForStatus(executor, received,
                          toInt(StatusCode::FATAL_RUNTIME_EXCEPTION)),
            toInt(StatusCode::FATAL_RUNTIME_EXCEPTION));
  EXPECT_EQ(manager.currentStatusCode(),
            toInt(StatusCode::FATAL_RUNTIME_EXCEPTION));
}
