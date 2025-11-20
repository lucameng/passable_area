#ifndef LIDAR_COVERAGE_HPP
#define LIDAR_COVERAGE_HPP

#include "common.hpp"
#include "common_types.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <grid_map_core/GridMap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.h>
#include <string>
#include <vector>

class LidarCoverage {
public:
  LidarCoverage() = default;
  explicit LidarCoverage(const rclcpp::Node::SharedPtr &node);
  void initialize(const std::string &dog_model);
  void addLidars(const std::string &model,
                 const std::vector<std::string> &lidar_names);
  DogModel parseDogModel(const std::string &dog_model) const;
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
  std::string dog_model_;
  std::string active_model_;
  const std::vector<std::string> lidar_names_x30_;
  const std::vector<std::string> lidar_names_m20_;
};

#endif // LIDAR_COVERAGE_HPP
