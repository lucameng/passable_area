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
        "cloud_topic", 1, std::bind(&PassableNode::cloudCallback, this, std::placeholders::_1));
    body_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("body_visual", 10);
    passable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("passable", 10);
    impassable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("impassable", 10);
    expanded_passable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("expanded", 10);
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
    ele_map_.clear("elevation_low");
    ele_map_.clear("elevation_high");

    float half_width = map_width_ * 0.5f;
    float half_height = map_height_ * 0.5f;
    int map_size_grid = ele_map_.getMapSizeGrid();

    std::vector<std::vector<float>> cell_bins(map_size_grid * map_size_grid);

#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(origin_cloud_.size()); ++i)
    {
        const auto& p = origin_cloud_[i];
        if (p.x <= -half_width || p.x >= half_width || 
            p.y <= -half_width || p.y >= half_width ||
            p.z <= -half_height || p.z >= half_height)
            continue;

        grid_map::Index index;
        if (!ele_map_.getIndex(Eigen::Vector2d(p.x, p.y), index)) continue;

        int id = index.x() + index.y() * map_size_grid;
#pragma omp critical
        cell_bins[id].push_back(p.z);
    }

#pragma omp parallel for collapse(2)
    for (int y = 0; y < map_size_grid; ++y)
    {
        for (int x = 0; x < map_size_grid; ++x)
        {
            int id = x + y * map_size_grid;
            auto& zs = cell_bins[id];
            if (zs.empty()) continue;

            float max_z = *std::max_element(zs.begin(), zs.end());

            float low_z = NAN, high_z = NAN;
            if (zs.size() > 3)
            {
                size_t k = static_cast<size_t>(zs.size() * low_ratio_);
                k = std::min(k, zs.size() - 1);
                std::nth_element(zs.begin(), zs.begin() + k, zs.end());
                low_z = zs[k];

                size_t j = static_cast<size_t>(zs.size() * high_ratio_);
                j = std::min(j, zs.size() - 1);
                std::nth_element(zs.begin(), zs.begin() + j, zs.end());
                high_z = zs[j];
            }
            
            ele_map_.at("elevation_low", grid_map::Index(x, y)) = low_z;
            ele_map_.at("elevation_high", grid_map::Index(x, y)) = high_z;
            ele_map_.at("elevation", grid_map::Index(x, y)) = max_z;
        }
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
    expanded_passable_cloud_.clear();
    long expand_count = 0;

    for (int i = 0; i < static_cast<int>(origin_cloud_.size()); ++i)
    {
        const auto& p = origin_cloud_[i];
        if (p.x <= -half_w || p.x >= half_w || p.y <= -half_w || 
            p.y >= half_w || p.z <= -half_h || p.z >= half_h)
            continue;

        Eigen::Vector2d pos(p.x, p.y);
        uint8_t step = ele_map_.getPassability(pos);
        if (step == STEPPABLE)
        {
            passable_cloud_.push_back(p);
        }
        else if (step == UNSTEPPABLE)
        {
            float low_z = ele_map_.atPosition("elevation_low", pos);
            float high_z = ele_map_.atPosition("elevation_high", pos);
            if (std::isfinite(high_z) && std::isfinite(low_z) && (p.z < low_z) &&
                (high_z - low_z >= gap_thresh_))
            {
                passable_cloud_.push_back(p);
                ++expand_count;
            }
            else
            {
                impassable_cloud_.push_back(p);
            }
        }
        else
        {
            impassable_cloud_.push_back(p);
        }
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
}

void PassableNode::publishGridMap()
{
    auto ros_map_ptr = grid_map::GridMapRosConverter::toMessage(ele_map_);

    ros_map_ptr->header.frame_id = g_frame_;
    ros_map_ptr->header.stamp = stamp_;

    grid_map_pub_->publish(*ros_map_ptr);
}