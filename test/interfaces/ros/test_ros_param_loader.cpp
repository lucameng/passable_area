#include "passable_area/interfaces/ros/ros_param_loader.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <vector>

namespace {

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

passable_area::interfaces::ros::RosNodeParams
LoadParamsWithOverrides(const std::string &node_name,
                        const std::vector<rclcpp::Parameter> &overrides) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  auto node = std::make_shared<rclcpp::Node>(node_name, options);
  return passable_area::interfaces::ros::RosParamLoader{}.load(*node);
}

} // namespace

TEST_F(RosFixture, LoadsDefaultAuxInputCloudTopic) {
  const auto params =
      LoadParamsWithOverrides("test_ros_param_loader_defaults", {});
  EXPECT_EQ(params.topics.aux_input_cloud_topic, "/tof/merged_pointcloud");
}

TEST_F(RosFixture, LoadsAuxInputCloudTopicOverride) {
  const auto params = LoadParamsWithOverrides(
      "test_ros_param_loader_aux_override",
      {rclcpp::Parameter("aux_input_cloud_topic", "/test/aux_cloud")});
  EXPECT_EQ(params.topics.aux_input_cloud_topic, "/test/aux_cloud");
}
