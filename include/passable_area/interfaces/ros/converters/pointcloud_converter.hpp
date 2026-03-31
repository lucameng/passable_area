#ifndef PASSABLE_AREA_INTERFACES_ROS_CONVERTERS_POINTCLOUD_CONVERTER_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_CONVERTERS_POINTCLOUD_CONVERTER_HPP_

#include "passable_area/core/types/frame_types.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace passable_area::interfaces::ros {

class PointCloudConverter {
public:
  bool fromRos(const sensor_msgs::msg::PointCloud2 &msg,
               passable_area::core::PointCloud &cloud) const;
  sensor_msgs::msg::PointCloud2 toRos(
      const std::vector<passable_area::core::CellDebugPoint> &points,
      const std_msgs::msg::Header &header) const;
};

} // namespace passable_area::interfaces::ros

#endif
