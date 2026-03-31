#ifndef PASSABLE_AREA_INTERFACES_ROS_CONVERTERS_ODOM_CONVERTER_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_CONVERTERS_ODOM_CONVERTER_HPP_

#include "passable_area/core/types/basic_types.hpp"

#include <nav_msgs/msg/odometry.hpp>

namespace passable_area::interfaces::ros {

class OdomConverter {
public:
  bool fromRos(const nav_msgs::msg::Odometry &msg, passable_area::core::Pose3D &pose) const;
};

} // namespace passable_area::interfaces::ros

#endif
