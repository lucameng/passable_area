#ifndef PASSABLE_AREA_NODE_HPP
#define PASSABLE_AREA_NODE_HPP

#include "common.hpp"
#include "elevation_map.hpp"
#include "lidar_coverage.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <unordered_set>

class PassableAreaNode : public rclcpp::Node {
public:
  explicit PassableAreaNode();
  void initialize();

private:
  void elevationInit();
  void lidarCoverInit();
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void bodyVisual();
  void publishPassableInfo();
  void publishGridMap();
  Eigen::Affine3f getTransform() const;

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr impassable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr expanded_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr body_vis_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;

  PointCloudXYZ passable_cloud_;
  PointCloudXYZ impassable_cloud_;
  PointCloudXYZ expanded_cloud_;

  std::string w_frame_;    // world frame
  std::string g_frame_;    // gravity frame
  std::string b_frame_;    // body frame
  std::string used_frame_; // frame to be used
  builtin_interfaces::msg::Time stamp_;

  std::unique_ptr<ElevationMap> ele_map_;
  std::unique_ptr<LidarCoverage> lidar_cov_;
  mutable std::mutex imu_mutex_;
  Eigen::Affine3f T_g2b_;

  float map_width_;
  float map_height_;
  float voxel_width_;
  float body_length_;
  float body_width_;
  float max_drop_;
  float rough_thres_;

  int body_l_;
  int body_w_;
  bool ele_init_;
  bool lidar_init_;
  bool enable_blind_check_;
};

#endif // PASSABLE_AREA_NODE_HPP
