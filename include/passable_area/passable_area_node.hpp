#ifndef PASSABLE_AREA_NODE_HPP
#define PASSABLE_AREA_NODE_HPP

#include "maths.hpp"
#include "common.hpp"
#include "elevation_map.hpp"
#include "lidar_coverage.hpp"
#include "traversal_cost.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
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
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr impassable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr expanded_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr body_vis_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr traversal_cost_pub_;

  PointCloudXYZ passable_cloud_;
  PointCloudXYZ impassable_cloud_;
  PointCloudXYZ expanded_cloud_;

  std::string w_frame_;    // world frame
  std::string g_frame_;    // gravity frame
  std::string b_frame_;    // body frame
  std::string used_frame_; // frame to be used
  std::string dog_model_;
  std::string accumulate_cloud_topic_;
  std::string imu_topic_;
  std::string passable_cloud_topic_;
  std::string impassable_cloud_topic_;
  std::string grid_map_topic_;
  std::string traversal_cost_topic_;
  builtin_interfaces::msg::Time stamp_;

  std::unique_ptr<ElevationMap> ele_map_;
  std::unique_ptr<LidarCoverage> lidar_cov_;
  std::unique_ptr<TraversalCost> traversal_cost_;
  mutable std::mutex imu_mutex_;
  Eigen::Affine3f T_g2b_;

  float map_length_;
  float map_width_;
  float min_height_;
  float max_height_;
  float voxel_width_;
  int max_inpaint_pixels_;
  bool enable_center_padding_;
  float center_dist_thresh_;

  float clearance_threshold_;
  float baseline_radius_;

  float body_length_;
  float body_width_;
  float body_height_;
  bool ele_init_;
  bool lidar_init_;
  bool enable_blind_check_;
  PassabilityParams passability_params_;
  TraversalCostParams traversal_cost_params_;
  ElevationSolverParams solver_params_;
  RaycastParams raycast_params_;
  std::vector<LidarParams> lidar_params_;

private:
  void elevationInit();
  void lidarCoverInit();
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void bodyVisual();
  void publishPassableInfo();
  void publishGridMap();
  void publishTraversalCost();
  Eigen::Affine3f getTransform() const;
  void loadBodyGeometry();
  void loadPassabilityParams();
  void loadElevationSolverParams();
  void loadTraversalCostParams();
  void loadRaycastParams();
  void loadLidarParams();
  std::string normalizeModelKey(const std::string &dog_model) const;
  BodyGeometry defaultBodyGeometry() const { return {0.9f, 0.4f, 0.5f}; }
};

#endif // PASSABLE_AREA_NODE_HPP
