#include "passable_area_node.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

PassableAreaNode::PassableAreaNode()
    : Node("passable_area"), ele_init_(false), lidar_init_(false),
      enable_blind_check_(declare_parameter("enable_blind_check", false)),
      map_length_(declare_parameter("map_length", 10.0f)),
      map_width_(declare_parameter("map_width", 10.0f)),
      min_height_(declare_parameter("map_height_min", -1.5f)),
      max_height_(declare_parameter("map_height_max", 1.5f)),
      voxel_width_(declare_parameter("voxel_size", 0.1f)),
      max_drop_(declare_parameter("max_drop", 0.3f)),
      rough_thres_(declare_parameter("max_roughness", 0.1f)),
      clearance_threshold_(declare_parameter("clearance_threshold", 0.05f)),
      baseline_radius_(declare_parameter("baseline_radius", 0.5f)),
      use_legacy_elevation_(declare_parameter("use_legacy_elevation", false)),
      elevation_solver_bins_(declare_parameter<int>("elevation_solver_bins", 16)),
      solver_region_enabled_(declare_parameter("solver_region_enabled", false)),
      solver_region_min_x_(declare_parameter("solver_region_min_x", 0.0f)),
      solver_region_max_x_(declare_parameter("solver_region_max_x", 0.0f)),
      solver_region_min_y_(declare_parameter("solver_region_min_y", 0.0f)),
      solver_region_max_y_(declare_parameter("solver_region_max_y", 0.0f)),
      w_frame_(declare_parameter<std::string>("world_frame", "camera_init")),
      g_frame_(declare_parameter<std::string>("gravity_frame", "base_gravity")),
      b_frame_(declare_parameter<std::string>("body_frame", "body")),
      used_frame_(declare_parameter<std::string>("used_frame", "base_gravity")),
      dog_model_(declare_parameter<std::string>("dog_model", "m20")),
      body_length_(0.0f), body_width_(0.0f), body_height_(0.0f), body_l_(0),
      body_w_(0), T_g2b_(Eigen::Affine3f::Identity()) {
  if (min_height_ > max_height_) {
    std::swap(min_height_, max_height_);
  }
  loadBodyGeometry();
}

void PassableAreaNode::loadBodyGeometry() {
  std::string model_key = normalizeModelKey(dog_model_);
  if (model_key != "x30" && model_key != "m20") {
    RCLCPP_WARN(get_logger(),
                "Unknown dog_model '%s', defaulting body parameters to 'm20'",
                dog_model_.c_str());
    model_key = "m20";
  }

  const BodyGeometry defaults = defaultBodyGeometry();
  const std::string base_param = "body_params." + model_key + ".";

  body_length_ =
      declare_parameter<float>(base_param + "body_length", defaults.length);
  body_width_ =
      declare_parameter<float>(base_param + "body_width", defaults.width);
  body_height_ =
      declare_parameter<float>(base_param + "body_height", defaults.height);

  if (voxel_width_ <= 0.0f) {
    RCLCPP_WARN(get_logger(),
                "voxel_size must be positive; skipping body geometry scaling.");
    body_l_ = 0;
    body_w_ = 0;
  } else {
    body_l_ = static_cast<int>(body_length_ / voxel_width_);
    body_w_ = static_cast<int>(body_width_ / voxel_width_);
  }

  RCLCPP_INFO(get_logger(),
              "Body geometry for model '%s': length=%.3f m, width=%.3f m, "
              "height=%.3f m",
              model_key.c_str(), body_length_, body_width_, body_height_);
}

std::string
PassableAreaNode::normalizeModelKey(const std::string &dog_model) const {
  std::string normalized = dog_model;
  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return normalized;
}

void PassableAreaNode::initialize() {
  RCLCPP_INFO(get_logger(), "Initializing << passable area >>");
  RCLCPP_INFO(get_logger(), "used frame: %s", used_frame_.c_str());
  elevationInit();

  if (enable_blind_check_) {
    lidarCoverInit();
  }
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10,
      std::bind(&PassableAreaNode::imuCallback, this, std::placeholders::_1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_topic", 10,
      std::bind(&PassableAreaNode::cloudCallback, this, std::placeholders::_1));
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

void PassableAreaNode::elevationInit() {
  RCLCPP_INFO(get_logger(), "Initializing < elevation map >");
  ele_map_ = std::make_unique<ElevationMap>(
      map_length_, map_width_, min_height_, max_height_, voxel_width_,
      used_frame_, get_logger());
  ele_map_->setUseLegacyVertical(use_legacy_elevation_);
  ele_map_->setSolverBins(elevation_solver_bins_);
  ele_map_->setSolverRegion(solver_region_enabled_, solver_region_min_x_,
                            solver_region_max_x_, solver_region_min_y_,
                            solver_region_max_y_);
  ele_init_ = true;
}

void PassableAreaNode::lidarCoverInit() {
  RCLCPP_INFO(get_logger(), "Initializing < lidar coverage >");
  lidar_cov_ = std::make_unique<LidarCoverage>(shared_from_this());
  lidar_cov_->initialize(dog_model_);
  lidar_init_ = true;
}

void PassableAreaNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
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

void PassableAreaNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (ele_init_) {
    const auto start_time = std::chrono::steady_clock::now();

    ele_map_->processPointCloud(*msg, rough_thres_, max_drop_, getTransform(),
                                enable_blind_check_);

    const auto end_time = std::chrono::steady_clock::now();
    const auto duration_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                              start_time)
            .count();
    if (duration_ms > 30) {
      RCLCPP_WARN(get_logger(), "Elevation map updated in %ld ms",
                  static_cast<long>(duration_ms));
    }
  }

  if (lidar_init_) {
    lidar_cov_->processCoverage(*ele_map_, getTransform());
  }

  stamp_ = msg->header.stamp;
  bodyVisual();
  publishPassableInfo();
  publishGridMap();
}

Eigen::Affine3f PassableAreaNode::getTransform() const {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  return T_g2b_;
}

void PassableAreaNode::bodyVisual() {
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
  body.scale.z = body_height_;

  body.color.r = 0.0;
  body.color.g = 0.0;
  body.color.b = 0.8;
  body.color.a = 0.7;

  body_vis_pub_->publish(body);
}

void PassableAreaNode::publishPassableInfo() {
  const auto &cloud = ele_map_->getWorkingCloud();

  const float half_length = map_length_ * 0.5f;
  const float half_width = map_width_ * 0.5f;
  const float baseline_ground = ele_map_->getBaselineGround(baseline_radius_);
  const bool have_baseline = std::isfinite(baseline_ground);
  if (have_baseline) {
    RCLCPP_DEBUG(get_logger(), "Baseline ground height: %.3f m",
                 baseline_ground);
  } else {
    RCLCPP_DEBUG(get_logger(), "Baseline ground height unavailable");
  }

  passable_cloud_.clear();
  impassable_cloud_.clear();
  expanded_cloud_.clear();

  for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {
    const auto &p = cloud[i];
    if (p.x < -half_length || p.x > half_length || p.y < -half_width ||
        p.y > half_width || p.z < min_height_ || p.z > max_height_)
      continue;

    Eigen::Vector2d pos(p.x, p.y);
    Passability step = ele_map_->getPassability(pos);
    CoverageStatus cover = ele_map_->getCoverability(pos);
    const bool near_baseline =
        have_baseline &&
        std::fabs(p.z - baseline_ground) <= clearance_threshold_;

    if (step == Passability::Passable) {
      passable_cloud_.push_back(p);
    } else if (step == Passability::Impassable &&
               cover == CoverageStatus::Covered) {
      if (!near_baseline) {
        impassable_cloud_.push_back(p);
      }
    } else if (step == Passability::Unknown) {
      int cnt = ele_map_->getPointCount(pos);
      if (cnt >= UNKNOWN_OBS_VALID_CNT && !near_baseline) {
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

void PassableAreaNode::publishGridMap() {
  auto ros_map_ptr = grid_map::GridMapRosConverter::toMessage(*ele_map_);

  ros_map_ptr->header.frame_id = used_frame_;
  ros_map_ptr->header.stamp = stamp_;

  grid_map_pub_->publish(*ros_map_ptr);
}
