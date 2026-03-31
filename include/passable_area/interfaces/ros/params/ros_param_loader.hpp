#ifndef PASSABLE_AREA_INTERFACES_ROS_PARAMS_ROS_PARAM_LOADER_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_PARAMS_ROS_PARAM_LOADER_HPP_

#include "passable_area/core/types/config_types.hpp"

#include <rclcpp/rclcpp.hpp>

namespace passable_area::interfaces::ros {

class RosParamLoader {
public:
  passable_area::core::Config load(rclcpp::Node &node) const;
};

} // namespace passable_area::interfaces::ros

#endif
