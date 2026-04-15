#include "passable_area/interfaces/ros/ros_param_loader.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace {

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

passable_area::interfaces::ros::RosNodeParams
LoadParamsWithOverrides(const std::string &node_name,
                        const std::vector<rclcpp::Parameter> &overrides) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  auto node = std::make_shared<rclcpp::Node>(node_name, options);
  return passable_area::interfaces::ros::RosParamLoader{}.load(*node);
}

} // namespace

TEST_F(RosFixture, UsesMapFrameDefaultsAndTfPublishingDisabled) {
  const auto params = LoadParamsWithOverrides("test_ros_param_loader_defaults",
                                              {});
  EXPECT_EQ(params.config.map_frame, "map");
  EXPECT_EQ(params.config.base_gravity_frame, "base_gravity");
  EXPECT_FALSE(params.config.debug.publish_map_to_base_gravity_tf);
}

TEST_F(RosFixture, FallsBackToLegacyTopLevelParameters) {
  const auto params = LoadParamsWithOverrides(
      "test_ros_param_loader_legacy",
      {rclcpp::Parameter("odom_frame", "legacy_map"),
       rclcpp::Parameter("publish_map_to_base_gravity_tf", true)});
  EXPECT_EQ(params.config.map_frame, "legacy_map");
  EXPECT_TRUE(params.config.debug.publish_map_to_base_gravity_tf);
}

TEST_F(RosFixture, PrefersExplicitNewParameterLocations) {
  const auto params = LoadParamsWithOverrides(
      "test_ros_param_loader_map_override",
      {rclcpp::Parameter("map_frame", "map_override"),
       rclcpp::Parameter("odom_frame", "stale_legacy_value"),
       rclcpp::Parameter("debug.publish_map_to_base_gravity_tf", false),
       rclcpp::Parameter("publish_map_to_base_gravity_tf", true)});
  EXPECT_EQ(params.config.map_frame, "map_override");
  EXPECT_FALSE(params.config.debug.publish_map_to_base_gravity_tf);
}
