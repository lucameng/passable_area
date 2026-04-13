#include "passable_area/interfaces/ros/converters/pointcloud_converter.hpp"

#include "passable_area/core/types/frame_types.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace passable_area::interfaces::ros {

bool PointCloudConverter::fromRos(
    const sensor_msgs::msg::PointCloud2 &msg,
    passable_area::core::PointCloud &cloud) const {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  try {
    pcl::fromROSMsg(msg, pcl_cloud);
  } catch (...) {
    return false;
  }
  cloud.clear();
  cloud.reserve(pcl_cloud.points.size());
  for (const auto &point : pcl_cloud.points) {
    cloud.push_back(passable_area::core::Point3f{point.x, point.y, point.z});
  }
  return true;
}

sensor_msgs::msg::PointCloud2 PointCloudConverter::toRos(
    const std::vector<passable_area::core::CellDebugPoint> &points,
    const std_msgs::msg::Header &header) const {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.reserve(points.size());
  for (const auto &point : points) {
    pcl_cloud.emplace_back(point.point.x, point.point.y, point.point.z);
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pcl_cloud, msg);
  msg.header = header;
  return msg;
}

} // namespace passable_area::interfaces::ros
