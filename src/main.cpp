#include <ros/ros.h>
#include "passable_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reachable_node");

    ros::NodeHandle nh("~");

    PassableNode reach_area(nh);

    ros::AsyncSpinner spinner(0);
    
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
