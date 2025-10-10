#include <ros/ros.h>
#include "passable_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "passable_node");

    ros::NodeHandle nh("~");

    PassableNode passable_node(nh);

    passable_node.initialize();

    ros::AsyncSpinner spinner(0);
    
    ROS_INFO("Start spinning passable node");
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
