#ifndef PASSABLE_AREA_INTERFACES_ROS_CONVERTERS_OUTPUT_CONVERTER_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_CONVERTERS_OUTPUT_CONVERTER_HPP_

#include "passable_area/core/types/frame_types.hpp"

#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>

namespace passable_area::interfaces::ros {

class OutputConverter {
public:
  nav_msgs::msg::OccupancyGrid toTerrainState(const passable_area::core::FrameOutput &output,
                                              const std_msgs::msg::Header &header) const;
  nav_msgs::msg::OccupancyGrid toTerrainCost(const passable_area::core::FrameOutput &output,
                                             const std_msgs::msg::Header &header) const;
  grid_map_msgs::msg::GridMap toGridMap(const passable_area::core::FrameOutput &output,
                                        const std_msgs::msg::Header &header) const;
};

} // namespace passable_area::interfaces::ros

#endif
