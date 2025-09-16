#include "passable_node.hpp"
#include <rclcpp/rclcpp.hpp>

PassableNode::PassableNode()
    : Node("passable_node"),
      ele_init_(false),
      map_width_(declare_parameter("map_width", 7.0f)),
      map_height_(declare_parameter("map_height", 2.0f)),
      voxel_width_(declare_parameter("voxel_size", 0.1f)),
      body_length_(declare_parameter("body_length", 0.6f)),
      body_width_(declare_parameter("body_width", 0.4f)),
      max_drop_(declare_parameter("max_drop", 0.3f)),
      rough_thres_(declare_parameter("max_roughness", 0.1f)),
      low_ratio_(declare_parameter("low_ratio", 0.1f)),
      high_ratio_(declare_parameter("high_ratio", 0.8f)),
      gap_thresh_(declare_parameter("gap_thresh", 0.1f)),
      w_frame_(declare_parameter<std::string>("world_frame", "camera_init")),
      g_frame_(declare_parameter<std::string>("gravity_frame", "base_gravity")),
      b_frame_(declare_parameter<std::string>("body_frame", "body")),
      body_l_(body_length_ / voxel_width_),
      body_w_(body_width_ / voxel_width_)
{
    // Passable node constructor
}

void PassableNode::initialize()
{
    RCLCPP_INFO(get_logger(), "Initializing passable node");
    elevationInit();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_topic", 10, std::bind(&PassableNode::cloudCallback, this, std::placeholders::_1));
    body_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("body_visual", 10);
    passable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("passable", 10);
    impassable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("impassable", 10);
    expanded_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("expanded", 10);
    grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
}

void PassableNode::elevationInit()
{
    RCLCPP_INFO(get_logger(), "Initializing elevation map");
    ele_map_ = elevationMap(map_width_, voxel_width_, g_frame_);
    ele_init_ = true;
}

void PassableNode::setInputCloud(const sensor_msgs::msg::PointCloud2& ros_cloud)
{
    pcl::fromROSMsg(ros_cloud, origin_cloud_);
    if (origin_cloud_.empty()) return;
    // cloud_ptr_ = origin_cloud_.makeShared();
    // kdtree_.setInputCloud(cloud_ptr_);
}

void PassableNode::bodyVisual()
{
    visualization_msgs::msg::Marker body;

    body.header.frame_id = b_frame_;
    body.header.stamp = stamp_;
    body.type = visualization_msgs::msg::Marker::CUBE;
    body.action = visualization_msgs::msg::Marker::ADD;

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

    body_vis_pub_->publish(body);
}

void PassableNode::cloud2Elevation()
{
    ele_map_.clear("elevation");
    float half_width = map_width_ * 0.5;
    float half_height = map_height_ * 0.5;
    
    #pragma omp parallel for
    for (const auto& p : origin_cloud_)
    {
        if (p.x <= -half_width || p.x >= half_width ||
            p.y <= -half_width || p.y >= half_width ||
            p.z <= -half_height || p.z >= half_height) continue;

        float height = ele_map_.getAltitude(Eigen::Vector2d(p.x, p.y));
        if (std::isnan(height))
        {
            height = p.z;
        }
        else
        {
            height = std::max(height, p.z);
        }
        ele_map_.setAltitude(Eigen::Vector2d(p.x, p.y), height);
    }
}

void PassableNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!ele_init_) return;
    stamp_ = msg->header.stamp;
    setInputCloud(*msg);
    cloud2Elevation();
    ele_map_.inPainting("elevation", MINLIMIT);
    ele_map_.deNoise("elevation", MEDIAN, 3);
    ele_map_.judgePassability(rough_thres_, max_drop_, 3);

    bodyVisual();
    publishPassableInfo();
    publishGridMap();
}

void PassableNode::publishPassableInfo()
{
    if (origin_cloud_.empty()) return;

    const float half_w = map_width_ * 0.5f;
    const float half_h = map_height_ * 0.5f;

    passable_cloud_.clear();
    impassable_cloud_.clear();
    expanded_cloud_.clear();
    long expand_count = 0;

    for (int i = 0; i < static_cast<int>(origin_cloud_.size()); ++i)
    {
        const auto& p = origin_cloud_[i];
        if (p.x <= -half_w || p.x >= half_w || p.y <= -half_w || 
            p.y >= half_w || p.z <= -half_h || p.z >= half_h)
            continue;

        Eigen::Vector2d pos(p.x, p.y);
        uint8_t step = ele_map_.getPassability(pos);
        if (step == PASSABLE)
        {
            passable_cloud_.push_back(p);
        }
        else if (step == IMPASSABLE)
        {
            impassable_cloud_.push_back(p);
        }
        // else
        // {
        //     expanded_cloud_.push_back(p);
        // }
    }

    auto finalize = [](pcl::PointCloud<pcl::PointXYZ>& c) {
        c.width = static_cast<uint32_t>(c.size());
        c.height = 1;
        c.is_dense = true;
    };

    sensor_msgs::msg::PointCloud2 ros_passable, ros_impassable;
    finalize(passable_cloud_);
    finalize(impassable_cloud_);
    pcl::toROSMsg(passable_cloud_, ros_passable);
    pcl::toROSMsg(impassable_cloud_, ros_impassable);
    ros_passable.header.frame_id = ros_impassable.header.frame_id = g_frame_;
    ros_passable.header.stamp = ros_impassable.header.stamp = stamp_;

    passable_pub_->publish(ros_passable);
    impassable_pub_->publish(ros_impassable);

    // visualize for debug
    // sensor_msgs::msg::PointCloud2 ros_expanded;
    // finalize(expanded_cloud_);
    // pcl::toROSMsg(expanded_cloud_, ros_expanded);
    // ros_expanded.header.frame_id = g_frame_;
    // ros_expanded.header.stamp = stamp_;
    // expanded_pub_->publish(ros_expanded);
}

void PassableNode::publishGridMap()
{
    auto ros_map_ptr = grid_map::GridMapRosConverter::toMessage(ele_map_);

    ros_map_ptr->header.frame_id = g_frame_;
    ros_map_ptr->header.stamp = stamp_;

    grid_map_pub_->publish(*ros_map_ptr);
}