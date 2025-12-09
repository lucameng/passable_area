#include "passable_area_node.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "passable_area");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  auto node = std::make_shared<PassableAreaNode>(nh, pnh);

  node->initialize();

  ROS_INFO("Start spinning passable area node");
  ros::spin();
  return 0;
}
