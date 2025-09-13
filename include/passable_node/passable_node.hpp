#ifndef PASSABLE_NODE_HPP
#define PASSABLE_NODE_HPP

#include "elevation_map.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <Eigen/Dense>
#include <unordered_set>

class PassableNode : public rclcpp::Node
{
public:
    explicit PassableNode();
    void initialize();

private:
    void elevationInit();
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void setInputCloud(const sensor_msgs::msg::PointCloud2 &ros_cloud);
    void cloud2Elevation();
    void bodyVisual();
    void publishPassableInfo();
    void publishGridMap();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passable_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr impassable_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr expanded_passable_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr body_vis_pub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_;
    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
    pcl::PointCloud<pcl::PointXYZ> origin_cloud_;
    pcl::PointCloud<pcl::PointXYZ> passable_cloud_;
    pcl::PointCloud<pcl::PointXYZ> impassable_cloud_;
    pcl::PointCloud<pcl::PointXYZ> expanded_passable_cloud_;
    std::vector<int> expanded_indices_;

    std::string w_frame_;   // world frame
    std::string g_frame_;   // gravity frame
    std::string b_frame_;   // body frame
    elevationMap ele_map_;
    builtin_interfaces::msg::Time stamp_;

    float map_width_;
    float map_height_;
    float voxel_width_;
    float body_length_;
    float body_width_;
    float max_drop_;
    float rough_thres_;
    const float low_ratio_;
    const float high_ratio_;
    const float gap_thresh_;

    int body_l_;
    int body_w_;
    bool ele_init_;
};

#endif // PASSABLE_NODE_HPP