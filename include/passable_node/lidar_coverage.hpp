#ifndef LIDAR_COVERAGE_HPP
#define LIDAR_COVERAGE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <grid_map_core/GridMap.hpp>
#include <Eigen/Dense>
#include <cmath>

struct LidarParam
{
    std::string name;
    Eigen::Vector3f pos_body;     // body frame
    Eigen::Vector3f rpy_body_deg; // body frame
    Eigen::Vector3f rpy_body_rad; // body frame
    Eigen::Matrix3f R_mount;      // rotation

    double fov_up_deg = 52.0;   // deg
    double fov_down_deg = -7.0; // deg
    double fov_up_rad;          // rad
    double fov_down_rad;        // rad

    double min_range = 0.1;  // m
    double max_range = 30.0; // m
};

class LidarCoverage
{
public:
    LidarCoverage() = default;
    explicit LidarCoverage(const ros::NodeHandle& nh_);
    void initialize();
    void addLidar(const std::string& lidar_name);
    void computeCoverage(grid_map::GridMap& map, const Eigen::Affine3f& T_g2b,
                         const std::string& layer_height = "elevation",
                         const std::string& layer_cover = "coverability") const;

private:
    bool isCellCoveredByLidar(const Eigen::Vector3f& p_body,
                              const LidarParam& lidar) const;

private:
    ros::NodeHandle nh_;
    const float ground_height_;
    Eigen::Vector2f bound_max_;
    Eigen::Vector2f bound_min_;
    std::vector<LidarParam> lidars_;
};

#endif // LIDAR_COVERAGE_HPP