#ifndef PASSABLE_AREA_NODE_HPP
#define PASSABLE_AREA_NODE_HPP

#include "common.hpp"
#include "elevation_map.hpp"
#include "lidar_coverage.hpp"
#include "maths.hpp"
#include "traversal_cost.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_set>

class PassableAreaNode : public rclcpp::Node {
public:
  explicit PassableAreaNode();
  void initialize();

private:
  using SyncPolicy = message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passable_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr impassable_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr passable_status_code_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr expanded_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr body_vis_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      traversal_cost_pub_;

  PointCloudXYZ passable_cloud_;
  PointCloudXYZ impassable_cloud_;
  PointCloudXYZ expanded_cloud_;

  std::string w_frame_;    // world frame
  std::string g_frame_;    // gravity frame
  std::string b_frame_;    // body frame
  std::string dog_model_;
  std::string input_cloud_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string passable_cloud_topic_;
  std::string impassable_cloud_topic_;
  std::string passable_status_code_topic_;
  std::string grid_map_topic_;
  std::string traversal_cost_topic_;
  builtin_interfaces::msg::Time stamp_;
  rclcpp::Time last_odom_msg_time_;
  rclcpp::Time last_cloud_msg_time_;
  rclcpp::Time last_synced_msg_time_;
  rclcpp::Time perf_window_start_time_;
  bool odom_received_;
  bool cloud_received_;
  bool synced_received_;
  rclcpp::TimerBase::SharedPtr data_watchdog_timer_;

  std::unique_ptr<ElevationMap> ele_map_;
  std::unique_ptr<LidarCoverage> lidar_cov_;
  std::unique_ptr<TraversalCost> traversal_cost_;
  mutable std::mutex data_mutex_;

  float map_length_;
  float map_width_;
  float min_height_;
  float max_height_;
  float voxel_width_;
  int max_inpaint_pixels_;
  bool enable_center_padding_;
  float center_dist_thresh_;

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
  DownsampleParams downsample_params_;
  std::vector<LidarParams> lidar_params_;
  int sync_queue_size_;
  int perf_cloud_count_;
  int perf_odom_count_;
  int perf_synced_count_;
  double perf_total_ms_sum_;
  double perf_total_ms_max_;

private:
  void elevationInit();
  void lidarCoverInit();
  void observeCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void observeOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
  void syncedCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
                      const nav_msgs::msg::Odometry::ConstSharedPtr &odom);
  void bodyVisual();
  void publishPassableInfo();
  void publishGridMap();
  void publishTraversalCost();
  bool buildGravityTransformFromOdom(const nav_msgs::msg::Odometry &msg,
                                     Eigen::Affine3f &T_g2b) const;
  bool preprocessCloud(const sensor_msgs::msg::PointCloud2 &msg,
                       const Eigen::Affine3f &T_g2b,
                       sensor_msgs::msg::PointCloud2 &processed_msg);
  void startDataWatchdog();
  void checkDataHealth();
  void updateAndMaybeLogPerf();
  void loadBodyGeometry();
  void loadPassabilityParams();
  void loadElevationSolverParams();
  void loadTraversalCostParams();
  void loadRaycastParams();
  void loadDownsampleParams();
  void loadSyncParams();
  void loadLidarParams();
  std::string normalizeModelKey(const std::string &dog_model) const;
  BodyGeometry defaultBodyGeometry() const { return {0.9f, 0.4f, 0.5f}; }
};

#endif // PASSABLE_AREA_NODE_HPP
