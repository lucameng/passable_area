#ifndef PASSABLE_AREA_NODE_HPP
#define PASSABLE_AREA_NODE_HPP

#include "common.hpp"
#include "common_types.hpp"
#include "elevation_map.hpp"
#include "lidar_coverage.hpp"
#include "traversal_cost.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

class PassableAreaNode {
public:
  PassableAreaNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  void initialize();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber imu_sub_;

  ros::Publisher passable_pub_;
  ros::Publisher impassable_pub_;
  ros::Publisher expanded_pub_;
  ros::Publisher body_vis_pub_;
  ros::Publisher grid_map_pub_;
  ros::Publisher traversal_cost_pub_;

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
  ros::Time stamp_;

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

private:
  void elevationInit();
  void lidarCoverInit();
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void bodyVisual();
  void publishPassableInfo();
  void publishGridMap();
  void publishTraversalCost();
  Eigen::Affine3f getTransform() const;
  void loadBodyGeometry();
  void loadPassabilityParams();
  void loadElevationSolverParams();
  void loadTraversalCostParams();
  std::string normalizeModelKey(const std::string &dog_model) const;
  BodyGeometry defaultBodyGeometry() const { return {0.9f, 0.4f, 0.5f}; }
};

#endif // PASSABLE_AREA_NODE_HPP
