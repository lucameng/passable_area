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
      max_inpaint_pixels_(declare_parameter<int>("max_inpaint_pixels", 200)),
      enable_center_padding_(declare_parameter("enable_center_padding", true)),
      center_dist_thresh_(declare_parameter("center_dist_thresh", 0.8f)),
      w_frame_(declare_parameter<std::string>("world_frame", "camera_init")),
      g_frame_(declare_parameter<std::string>("gravity_frame", "base_gravity")),
      b_frame_(declare_parameter<std::string>("body_frame", "body")),
      used_frame_(declare_parameter<std::string>("used_frame", "base_gravity")),
      dog_model_(declare_parameter<std::string>("dog_model", "m20")),
      accumulate_cloud_topic_(declare_parameter<std::string>(
          "accumulate_cloud_topic", "cloud_topic")),
      imu_topic_(declare_parameter<std::string>("imu_topic", "imu")),
      passable_cloud_topic_(declare_parameter<std::string>(
          "passable_cloud_topic", "passable_area")),
      impassable_cloud_topic_(declare_parameter<std::string>(
          "impassable_cloud_topic", "impassable_area")),
      grid_map_topic_(
          declare_parameter<std::string>("grid_map_topic", "grid_map")),
      traversal_cost_topic_(declare_parameter<std::string>(
          "traversal_cost_topic", "traversal_cost")),
      body_length_(0.0f), body_width_(0.0f), body_height_(0.0f),
      T_g2b_(Eigen::Affine3f::Identity()) {
  if (min_height_ > max_height_) {
    std::swap(min_height_, max_height_);
  }
  loadBodyGeometry();
  loadLidarParams();
  loadPassabilityParams();
  loadElevationSolverParams();
  loadTraversalCostParams();
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

  RCLCPP_INFO(get_logger(),
              "Body geometry for model '%s': length=%.3f m, width=%.3f m, "
              "height=%.3f m",
              model_key.c_str(), body_length_, body_width_, body_height_);
}

void PassableAreaNode::loadElevationSolverParams() {
  const std::string base = "elevation_solver.";
  solver_params_.use_histogram_solver =
      declare_parameter<bool>(base + "use_histogram_solver", true);
  solver_params_.histogram_bins =
      declare_parameter<int>(base + "histogram_bins", 16);
  solver_params_.region.enabled =
      declare_parameter<bool>(base + "region_enabled", false);
  solver_params_.region.min_x =
      declare_parameter<float>(base + "region_min_x", 0.0f);
  solver_params_.region.max_x =
      declare_parameter<float>(base + "region_max_x", 0.0f);
  solver_params_.region.min_y =
      declare_parameter<float>(base + "region_min_y", 0.0f);
  solver_params_.region.max_y =
      declare_parameter<float>(base + "region_max_y", 0.0f);
}

void PassableAreaNode::loadTraversalCostParams() {
  traversal_cost_params_.enabled =
      declare_parameter("traversal_cost.enable", true);
  traversal_cost_params_.slope_free_deg =
      declare_parameter("traversal_cost.slope_free_deg", 5.0f);
  traversal_cost_params_.slope_block_deg =
      declare_parameter("traversal_cost.slope_block_deg", 30.0f);
  traversal_cost_params_.rough_free =
      declare_parameter("traversal_cost.rough_free", 0.02f);
  traversal_cost_params_.rough_block =
      declare_parameter("traversal_cost.rough_block", 0.08f);
  traversal_cost_params_.step_free =
      declare_parameter("traversal_cost.step_free", 0.05f);
  traversal_cost_params_.step_block =
      declare_parameter("traversal_cost.step_block", 0.18f);
  traversal_cost_params_.slope_weight =
      declare_parameter("traversal_cost.slope_weight", 0.4f);
  traversal_cost_params_.roughness_weight =
      declare_parameter("traversal_cost.roughness_weight", 0.3f);
  traversal_cost_params_.step_weight =
      declare_parameter("traversal_cost.step_weight", 0.3f);
  traversal_cost_params_.easy_cost =
      declare_parameter("traversal_cost.easy_cost", 1.0f);
  traversal_cost_params_.hard_cost =
      declare_parameter("traversal_cost.hard_cost", 60.0f);
  traversal_cost_params_.max_cost =
      declare_parameter("traversal_cost.max_cost", 100.0f);
  traversal_cost_params_.curve_power =
      declare_parameter("traversal_cost.curve_power", 3.0f);
  traversal_cost_params_.terrain_sample_window =
      declare_parameter<int>("traversal_cost.terrain_sample_window", 1);
  traversal_cost_params_.safe_zone_side_length =
      declare_parameter("traversal_cost.safe_zone_side_length", 1.0f);
}

void PassableAreaNode::loadRaycastParams() {
  raycast_params_.enable = declare_parameter("raycast.enable", true);
  raycast_params_.max_ray_distance =
      declare_parameter("raycast.max_ray_distance", 4.0f);
  raycast_params_.max_nan_gap = declare_parameter("raycast.max_nan_gap", 1.0f);
}

void PassableAreaNode::loadLidarParams() {
  lidar_params_.clear();

  std::string model_key = normalizeModelKey(dog_model_);
  std::vector<std::string> lidar_names;
  if (model_key == "x30") {
    lidar_names = {"lidar_front_up", "lidar_front_down", "lidar_rear_up",
                   "lidar_rear_down"};
  } else {
    if (model_key != "m20") {
      RCLCPP_WARN(get_logger(),
                  "Unknown dog_model '%s', falling back to 'm20' lidar params",
                  dog_model_.c_str());
      model_key = "m20";
    }
    lidar_names = {"lidar_front", "lidar_rear"};
  }

  for (const auto &lidar_name : lidar_names) {
    LidarParams param;
    param.name = lidar_name;
    const std::string prefix = "lidar_params." + model_key + "." + lidar_name;

    std::vector<double> pos_vec;
    declare_parameter(prefix + ".pos_body",
                      std::vector<double>{param.pos_body.x(),
                                          param.pos_body.y(),
                                          param.pos_body.z()});
    get_parameter(prefix + ".pos_body", pos_vec);
    if (pos_vec.size() == 3) {
      param.pos_body = Eigen::Vector3f(pos_vec[0], pos_vec[1], pos_vec[2]);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid pos_body for %s, using defaults",
                  lidar_name.c_str());
    }

    std::vector<double> rpy_vec;
    declare_parameter(prefix + ".rpy_body",
                      std::vector<double>{param.rpy_body_deg.x(),
                                          param.rpy_body_deg.y(),
                                          param.rpy_body_deg.z()});
    get_parameter(prefix + ".rpy_body", rpy_vec);
    if (rpy_vec.size() == 3) {
      param.rpy_body_deg = Eigen::Vector3f(rpy_vec[0], rpy_vec[1], rpy_vec[2]);
      param.rpy_body_rad = Eigen::Vector3f(dr::degreeToRadian(rpy_vec[0]),
                                           dr::degreeToRadian(rpy_vec[1]),
                                           dr::degreeToRadian(rpy_vec[2]));
      param.R_mount = dr::rotationFromYPRrad(param.rpy_body_rad);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid rpy_body for %s, using defaults",
                  lidar_name.c_str());
    }

    declare_parameter(prefix + ".fov_up_deg", param.fov_up_deg);
    declare_parameter(prefix + ".fov_down_deg", param.fov_down_deg);
    get_parameter(prefix + ".fov_up_deg", param.fov_up_deg);
    get_parameter(prefix + ".fov_down_deg", param.fov_down_deg);
    param.fov_up_rad = dr::degreeToRadian(param.fov_up_deg);
    param.fov_down_rad = dr::degreeToRadian(param.fov_down_deg);

    declare_parameter(prefix + ".min_range", param.min_range);
    declare_parameter(prefix + ".max_range", param.max_range);
    get_parameter(prefix + ".min_range", param.min_range);
    get_parameter(prefix + ".max_range", param.max_range);

    lidar_params_.push_back(param);
  }
  RCLCPP_INFO(
      get_logger(),
      "Using lidar params for dog model '%s' (%zu lidars)",
      model_key.c_str(), lidar_params_.size());
}

void PassableAreaNode::loadPassabilityParams() {
  passability_params_.drop_threshold = 
      declare_parameter("max_drop", 0.3f);
  passability_params_.roughness_threshold =
      declare_parameter("max_roughness", 0.1f);
  passability_params_.max_slope_deg = 
      declare_parameter("max_slope_deg", 45.0f);
  passability_params_.treat_nan_as_stiff =
      declare_parameter("treat_nan_as_stiff", true);
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
  RCLCPP_INFO(get_logger(),
              "Topic configuration:\n"
              "  cloud: %s\n"
              "  imu: %s\n"
              "  passable: %s\n"
              "  impassable: %s\n"
              "  grid_map: %s",
              accumulate_cloud_topic_.c_str(), imu_topic_.c_str(),
              passable_cloud_topic_.c_str(), impassable_cloud_topic_.c_str(),
              grid_map_topic_.c_str());
  elevationInit();

  if (enable_blind_check_) {
    lidarCoverInit();
  }
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&PassableAreaNode::imuCallback, this, std::placeholders::_1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      accumulate_cloud_topic_, 10,
      std::bind(&PassableAreaNode::cloudCallback, this, std::placeholders::_1));
  body_vis_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("body_visual", 10);
  passable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      passable_cloud_topic_, 10);
  impassable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      impassable_cloud_topic_, 10);
  grid_map_pub_ =
      create_publisher<grid_map_msgs::msg::GridMap>(grid_map_topic_, 10);
  traversal_cost_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>(traversal_cost_topic_, 10);
}

void PassableAreaNode::elevationInit() {
  RCLCPP_INFO(get_logger(), "Initializing < elevation map >");
  ele_map_ = std::make_unique<ElevationMap>(
      map_length_, map_width_, min_height_, max_height_, voxel_width_,
      used_frame_, get_logger());
  ele_map_->setMaxInpaintPixels(max_inpaint_pixels_);
  ele_map_->setCenterPaddingParams(enable_center_padding_, center_dist_thresh_);
  ele_map_->setElevationSolverParams(solver_params_);
  ele_map_->setTraversalCostParams(traversal_cost_params_);
  loadRaycastParams();
  ele_map_->setRaycastParams(raycast_params_);
  ele_map_->setLidarParams(lidar_params_);
  traversal_cost_ = std::make_unique<TraversalCost>(
      *ele_map_, ele_map_->getTraversalCostParams(), get_logger());
  ele_init_ = true;
}

void PassableAreaNode::lidarCoverInit() {
  RCLCPP_INFO(get_logger(), "Initializing < lidar coverage >");
  lidar_cov_ = std::make_unique<LidarCoverage>(shared_from_this());
  lidar_cov_->initialize(dog_model_, lidar_params_);
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

    ele_map_->processPointCloud(*msg, passability_params_, getTransform());
    if (traversal_cost_) {
      traversal_cost_->updateCostLayer();
    }

    const auto end_time = std::chrono::steady_clock::now();
    const auto duration_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                              start_time)
            .count();
    if (duration_ms > 50) {
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
  publishTraversalCost();
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

    if (step == Passability::Passable) {
      passable_cloud_.push_back(p);
    } else if (step == Passability::Impassable &&
               cover == CoverageStatus::Covered) {
        impassable_cloud_.push_back(p);
    } else if (step == Passability::Unknown) {
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

void PassableAreaNode::publishGridMap() {
  auto ros_map_ptr = grid_map::GridMapRosConverter::toMessage(*ele_map_);

  ros_map_ptr->header.frame_id = used_frame_;
  ros_map_ptr->header.stamp = stamp_;

  grid_map_pub_->publish(*ros_map_ptr);
}

void PassableAreaNode::publishTraversalCost() {
  if (!traversal_cost_ || !traversal_cost_pub_)
    return;
  traversal_cost_->publish(traversal_cost_pub_, used_frame_, stamp_);
}
