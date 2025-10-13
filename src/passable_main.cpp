#include "passable_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PassableNode>();
    
    node->initialize();

    RCLCPP_INFO(node->get_logger(), "Start spinning passable node");

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}