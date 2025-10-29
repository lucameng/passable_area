#include "passable_node.hpp"
#include <rclcpp/rclcpp.hpp>

PassableNode::PassableNode()
    : Node("passable_node"), ele_init_(false), lidar_init_(false),
      enable_blind_check_(declare_parameter("enable_blind_check", false)),
      map_width_(declare_parameter("map_width", 7.0f)),
      map_height_(declare_parameter("map_height", 2.0f)),
      voxel_width_(declare_parameter("voxel_size", 0.1f)),
      body_length_(declare_parameter("body_length", 0.6f)),
      body_width_(declare_parameter("body_width", 0.4f)),
      max_drop_(declare_parameter("max_drop", 0.3f)),
      rough_thres_(declare_parameter("max_roughness", 0.1f)),
      w_frame_(declare_parameter<std::string>("world_frame", "camera_init")),
      g_frame_(declare_parameter<std::string>("gravity_frame", "base_gravity")),
      b_frame_(declare_parameter<std::string>("body_frame", "body")),
      used_frame_(declare_parameter<std::string>("used_frame", "base_gravity")),
      body_l_(body_length_ / voxel_width_), body_w_(body_width_ / voxel_width_),
      T_g2b_(Eigen::Affine3f::Identity()) {
  // Passable node constructor
}

void PassableNode::initialize() {
  RCLCPP_INFO(get_logger(), "Initializing << passable node >>");
  RCLCPP_INFO(get_logger(), "used frame: %s", used_frame_.c_str());
  elevationInit();

  if (enable_blind_check_) {
    lidarCoverInit();
  }
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10,
      std::bind(&PassableNode::imuCallback, this, std::placeholders::_1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_topic", 10,
      std::bind(&PassableNode::cloudCallback, this, std::placeholders::_1));
  body_vis_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("body_visual", 10);
  passable_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("passable_area", 10);
  impassable_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("impassable_area", 10);
  // expanded_pub_ =
  // create_publisher<sensor_msgs::msg::PointCloud2>("expanded_area", 10);
  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
}

void PassableNode::elevationInit() {
  RCLCPP_INFO(get_logger(), "Initializing < elevation map >");
  ele_map_ = std::make_unique<ElevationMap>(map_width_, map_height_,
                                            voxel_width_, used_frame_);
  ele_init_ = true;
}

void PassableNode::lidarCoverInit() {
  RCLCPP_INFO(get_logger(), "Initializing < lidar coverage >");
  lidar_cov_ = std::make_unique<LidarCoverage>(shared_from_this());
  lidar_cov_->initialize();
  lidar_init_ = true;
}

void PassableNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (used_frame_ == b_frame_)
    return; // no need if it is body frame

  Eigen::Quaternionf q(msg->orientation.w, msg->orientation.x,
                       msg->orientation.y, msg->orientation.z);
  q.normalize();

  Eigen::Matrix3f R_world2body = q.toRotationMatrix();
  // RCLCPP_INFO_STREAM_INFO_STREAM("R_world2body:\n" << R_world2body << "\n");

  float yaw = std::atan2(R_world2body(1, 0), R_world2body(0, 0));
  float pitch = std::asin(-R_world2body(2, 0)); // N O T
  float roll = std::atan2(R_world2body(2, 1), R_world2body(2, 2));

  pitch = -pitch;
  Eigen::AngleAxisf Rx(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf Ry(pitch, Eigen::Vector3f::UnitY());
  Eigen::Matrix3f R_gravity2body = (Ry * Rx).toRotationMatrix();

  // RCLCPP_INFO_STREAM("R_baseGravity2body:\n" << R_baseGravity2body << "\n");

  std::lock_guard<std::mutex> lock(imu_mutex_);
  T_g2b_.setIdentity();
  T_g2b_.linear() = R_gravity2body;
  T_g2b_.translation() = Eigen::Vector3f::Zero();
}

void PassableNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (ele_init_) {
    ele_map_->processPointCloud(*msg, rough_thres_, max_drop_, getTransform(),
                                enable_blind_check_);
  }

  if (lidar_init_) {
    lidar_cov_->processCoverage(*ele_map_, getTransform());
  }

  stamp_ = msg->header.stamp;
  bodyVisual();
  publishPassableInfo();
  publishGridMap();
}

Eigen::Affine3f PassableNode::getTransform() const {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  return T_g2b_;
}

void PassableNode::bodyVisual() {
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

void PassableNode::publishPassableInfo() {
  const auto &cloud = ele_map_->getWorkingCloud();

  const float half_w = map_width_ * 0.5f;
  const float half_h = map_height_ * 0.5f;

  passable_cloud_.clear();
  impassable_cloud_.clear();
  expanded_cloud_.clear();

  for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {
    const auto &p = cloud[i];
    if (p.x <= -half_w || p.x >= half_w || p.y <= -half_w || p.y >= half_w ||
        p.z <= -half_h || p.z >= half_h)
      continue;

    Eigen::Vector2d pos(p.x, p.y);
    uint8_t step = ele_map_->getPassability(pos);
    uint8_t cover = ele_map_->getCoverability(pos);

    if (step == PASSABLE) {
      passable_cloud_.push_back(p);
    } else if (step == IMPASSABLE && cover == COVERED) {
      impassable_cloud_.push_back(p);
    } else if (step == UNKNOWN) {
      int cnt = ele_map_->getPointCount(pos);
      if (cnt >= UNKNOWN_OBS_VALID_CNT) {
        impassable_cloud_.push_back(p);
      }
    }
  }
  auto finalize = [](pcl::PointCloud<pcl::PointXYZ> &c) {
    c.width = static_cast<uint32_t>(c.size());
    c.height = 1;
    c.is_dense = true;
  };

  sensor_msgs::msg::PointCloud2 ros_passable, ros_impassable;
  finalize(passable_cloud_);
  finalize(impassable_cloud_);
  pcl::toROSMsg(passable_cloud_, ros_passable);
  pcl::toROSMsg(impassable_cloud_, ros_impassable);
  ros_passable.header.frame_id = ros_impassable.header.frame_id = used_frame_;
  ros_passable.header.stamp = ros_impassable.header.stamp = stamp_;

  passable_pub_->publish(ros_passable);
  impassable_pub_->publish(ros_impassable);

  // visualize for debug
  // sensor_msgs::msg::PointCloud2 ros_expanded;
  // finalize(expanded_cloud_);
  // pcl::toROSMsg(expanded_cloud_, ros_expanded);
  // ros_expanded.header.frame_id = used_frame_;
  // ros_expanded.header.stamp = stamp_;
  // expanded_pub_->publish(ros_expanded);
}

void PassableNode::publishGridMap() {
  auto ros_map_ptr = grid_map::GridMapRosConverter::toMessage(*ele_map_);

  ros_map_ptr->header.frame_id = used_frame_;
  ros_map_ptr->header.stamp = stamp_;

  grid_map_pub_->publish(*ros_map_ptr);
}