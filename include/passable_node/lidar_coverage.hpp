#ifndef LIDAR_COVERAGE_HPP
#define LIDAR_COVERAGE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <grid_map_core/GridMap.hpp>
#include <Eigen/Dense>
#include <cmath>

enum CoverageStatus : uint8_t
{
    UNCOVERED = 0,
    COVERED = 1
};

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
    void computeCoverage(grid_map::GridMap& map, const Eigen::Affine3f& T_g2b) const;

private:
    // void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    bool isCellCoveredByLidar(const Eigen::Vector3f& p_body,
                              const LidarParam& lidar) const;

private:
    ros::NodeHandle nh_;
    // ros::Subscriber imu_sub_;
    const float ground_height_;
    Eigen::Vector2f bound_max_;
    Eigen::Vector2f bound_min_;
    // Eigen::Quaternionf q_g2b_;
    // Eigen::Matrix4f T_g2b_;
    // Eigen::Affine3f T_g2b_;
    std::vector<LidarParam> lidars_;
};

#endif // LIDAR_COVERAGE_HPP