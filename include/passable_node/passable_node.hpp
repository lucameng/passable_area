#ifndef PASSABLE_NODE_HPP
#define PASSABLE_NODE_HPP

#include "elevation_map.hpp"
#include "lidar_coverage.hpp"
#include "common.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <cmath>
#include <Eigen/Dense>
#include <unordered_set>
#include <memory>

class PassableNode
{
public:
    PassableNode() = default;
    explicit PassableNode(const ros::NodeHandle& nh);
    void initialize();

private:
    void elevationInit();
    void lidarCoverInit();
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void bodyVisual();
    void publishPassableInfo();
    void publishGridMap();
    Eigen::Affine3f getTransform() const;

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher passable_pub_;
    ros::Publisher impassable_pub_;
    ros::Publisher expanded_pub_;
    ros::Publisher body_vis_pub_;
    ros::Publisher grid_map_pub_;

    PointCloudXYZ passable_cloud_;
    PointCloudXYZ impassable_cloud_;
    PointCloudXYZ expanded_cloud_;

    std::string w_frame_;   // world frame
    std::string g_frame_;   // gravity frame
    std::string b_frame_;   // body frame
    std::string used_frame_;    // frame to be used
    ros::Time stamp_;

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
};

#endif // PASSABLE_NODE_HPP