#include "passable_node.hpp"
#include <ros/ros.h>

PassableNode::PassableNode(const ros::NodeHandle& nh)
    : nh_(nh),
      ele_init_(false),
      lidar_init_(false),
      map_width_(nh.param("map_width", 7.0f)),
      map_height_(nh.param("map_height", 2.0f)),
      voxel_width_(nh.param("voxel_size", 0.1f)),
      body_length_(nh.param("body_length", 0.6f)),
      body_width_(nh.param("body_width", 0.4f)),
      max_drop_(nh.param("max_drop", 0.3f)),
      rough_thres_(nh.param("max_roughness", 0.1f)),
      w_frame_(nh.param<std::string>("world_frame", "camera_init")),
      g_frame_(nh.param<std::string>("gravity_frame", "base_gravity")),
      b_frame_(nh.param<std::string>("body_frame", "body")),
      body_l_(body_length_ / voxel_width_),
      body_w_(body_width_ / voxel_width_)
{
    // Passable node constructor
}

void PassableNode::initialize()
{
    ROS_INFO("Initializing passable node");
    elevationInit();
    lidarCoverInit();

    imu_sub_ = nh_.subscribe("imu", 50, &PassableNode::imuCallback, this,
                                ros::TransportHints().tcpNoDelay());
    cloud_sub_ = nh_.subscribe("cloud_topic", 1, &PassableNode::cloudCallback, this,
                               ros::TransportHints().tcpNoDelay());
    body_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("body_visual", 1);
    passable_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("passable_area", 1);
    impassable_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("impassable_area", 1);
    expanded_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("expanded", 1);
    grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
}

void PassableNode::elevationInit()
{
    ROS_INFO("Initializing elevation map");
    ele_map_ = std::make_unique<ElevationMap>(map_width_, map_height_, voxel_width_, g_frame_);
    ele_init_ = true;
}

void PassableNode::lidarCoverInit()
{
    ROS_INFO("Initializing lidar coverage");
    lidar_cov_ = std::make_unique<LidarCoverage>(nh_);
    lidar_cov_->initialize();
    lidar_init_ = true;
}

void PassableNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Quaternionf q(msg->orientation.w, msg->orientation.x,
                         msg->orientation.y, msg->orientation.z);
    q.normalize();

    Eigen::Matrix3f R_world2body = q.toRotationMatrix();
    // ROS_INFO_STREAM("R_world2body:\n" << R_world2body << "\n");

    float yaw = std::atan2(R_world2body(1, 0), R_world2body(0, 0));
    float pitch = std::asin(-R_world2body(2, 0));
    float roll = std::atan2(R_world2body(2, 1), R_world2body(2, 2));

    pitch = -pitch;
    Eigen::AngleAxisf Rx(roll,  Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf Ry(pitch, Eigen::Vector3f::UnitY());
    Eigen::Matrix3f R_gravity2body = (Ry * Rx).toRotationMatrix();
    
    // ROS_INFO_STREAM("R_baseGravity2body:\n" << R_baseGravity2body << "\n");

    std::lock_guard<std::mutex> lock(imu_mutex_);
    T_g2b_.setIdentity();
    T_g2b_.linear() = R_gravity2body;
    T_g2b_.translation() = Eigen::Vector3f::Zero();
}

void PassableNode::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!ele_init_) return;
    stamp_ = msg->header.stamp;

    ele_map_->processPointCloud(*msg, rough_thres_, max_drop_, getTransform());

    lidar_cov_->processCoverage(*ele_map_, getTransform());

    bodyVisual();
    publishPassableInfo();
    publishGridMap();
}

Eigen::Affine3f PassableNode::getTransform() const
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return T_g2b_;
}

void PassableNode::bodyVisual()
{
    visualization_msgs::Marker body;

    body.header.frame_id = b_frame_;
    body.header.stamp = stamp_;
    body.type = visualization_msgs::Marker::CUBE;
    body.action = visualization_msgs::Marker::ADD;

    body.pose.position.x = 0.0;
    body.pose.position.y = 0.0;
    body.pose.position.z = 0.0;

    body.pose.orientation.x = 0.0;
    body.pose.orientation.y = 0.0;
    body.pose.orientation.z = 0.0;
    body.pose.orientation.w = 1.0;

    body.scale.x = body_length_;
    body.scale.y = body_width_;
    body.scale.z = 0.3;

    body.color.r = 0.0;
    body.color.g = 0.0;
    body.color.b = 0.8;
    body.color.a = 0.7;

    body_vis_pub_.publish(body);
}

void PassableNode::publishPassableInfo()
{
    const auto& cloud = ele_map_->getWorkingCloud();
    
    if (cloud.empty()) return;
    
    const float half_w = map_width_ * 0.5f;
    const float half_h = map_height_ * 0.5f;

    passable_cloud_.clear();
    impassable_cloud_.clear();
    expanded_cloud_.clear();

    for (int i = 0; i < static_cast<int>(cloud.size()); ++i)
    {
        const auto& p = cloud[i];
        if (p.x <= -half_w || p.x >=  half_w || p.y <= -half_w || 
            p.y >=  half_w || p.z <= -half_h || p.z >=  half_h)
            continue;

        Eigen::Vector2d pos(p.x, p.y);
        uint8_t step = ele_map_->getPassability(pos);
        uint8_t cover = ele_map_->getCoverability(pos);

        if (step == PASSABLE)
        {
            passable_cloud_.push_back(p);
        }
        else if (step == IMPASSABLE && cover == COVERED)
        {
            impassable_cloud_.push_back(p);
        }
        else if (step == UNKNOWN)
        {
            int cnt = ele_map_->getPointCount(pos);
            if (cnt >= UNKNOWN_OBS_VALID_CNT)
            {
                impassable_cloud_.push_back(p);
            }
        }
    }

    auto finalize = [](pcl::PointCloud<pcl::PointXYZ>& c) {
        c.width = static_cast<uint32_t>(c.size());
        c.height = 1;
        c.is_dense = true;
    };

    sensor_msgs::PointCloud2 ros_passable, ros_impassable;
    finalize(passable_cloud_);
    finalize(impassable_cloud_);
    pcl::toROSMsg(passable_cloud_, ros_passable);
    pcl::toROSMsg(impassable_cloud_, ros_impassable);
    ros_passable.header.frame_id = ros_impassable.header.frame_id = g_frame_;
    ros_passable.header.stamp = ros_impassable.header.stamp = stamp_;

    passable_pub_.publish(ros_passable);
    impassable_pub_.publish(ros_impassable);

    /* visualize for debug
    expanded_cloud_ = cloud;
    sensor_msgs::PointCloud2 ros_expanded;
    finalize(expanded_cloud_);
    pcl::toROSMsg(expanded_cloud_, ros_expanded);
    ros_expanded.header.frame_id = g_frame_;
    ros_expanded.header.stamp = stamp_;
    expanded_pub_.publish(ros_expanded);
    */
}

void PassableNode::publishGridMap()
{
    grid_map_msgs::GridMap ros_map;
    grid_map::GridMapRosConverter::toMessage(*ele_map_, ros_map);
    ros_map.info.header.stamp = stamp_;
    ros_map.info.header.frame_id = g_frame_;
    grid_map_pub_.publish(ros_map);
}