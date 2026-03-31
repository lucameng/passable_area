#include "passable_area/interfaces/ros/nodes/passable_area_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<passable_area::interfaces::ros::PassableAreaNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
