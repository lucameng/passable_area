#ifndef LIDAR_COVERAGE_HPP
#define LIDAR_COVERAGE_HPP

#include <Eigen/Dense>
#include <cmath>
#include <grid_map_core/GridMap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.h>

struct LidarParam {
  std::string name;
  Eigen::Vector3f pos_body;     // body frame
  Eigen::Vector3f rpy_body_deg; // body frame
  Eigen::Vector3f rpy_body_rad; // body frame
  Eigen::Matrix3f R_mount;      // rotation

  double fov_up_deg = 52.0;   // deg
  double fov_down_deg = -7.0; // deg
  double fov_up_rad;          // rad
  double fov_down_rad;        // rad

  double min_range = 0.01; // m
  double max_range = 30.0; // m
};

class LidarCoverage {
public:
  LidarCoverage() = default;
  explicit LidarCoverage(const rclcpp::Node::SharedPtr &node);
  void initialize();
  void addLidar(const std::string &lidar_name);
  void processCoverage(grid_map::GridMap &ele_map,
                       const Eigen::Affine3f &T_g2b);

private:
  bool isCellCoveredByLidar(const Eigen::Vector3f &p_body,
                            const LidarParam &lidar) const;
  void computeCoverage(grid_map::GridMap &ele_map, const Eigen::Affine3f &T_g2b,
                       const std::string &layer_height = "elevation",
                       const std::string &layer_covered = "coverability") const;
  void dilateUncoveredArea(grid_map::GridMap &ele_map, int dilation_radius = 3,
                           const std::string &layer_covered = "coverability");

private:
  rclcpp::Node::SharedPtr node_;
  const float ground_height_;
  Eigen::Vector2f bound_max_;
  Eigen::Vector2f bound_min_;
  std::vector<LidarParam> lidars_;
};

#endif // LIDAR_COVERAGE_HPP