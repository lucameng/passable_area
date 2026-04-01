#include "passable_area/interfaces/ros/nodes/passable_area_node.hpp"

#include "passable_area/core/utils/math_utils.hpp"

#include <chrono>

namespace passable_area::interfaces::ros {
namespace {

Eigen::Quaternionf YawOnlyQuaternion(const Eigen::Quaternionf &orientation) {
  return Eigen::Quaternionf(
      Eigen::AngleAxisf(passable_area::core::YawFromQuaternion(orientation),
                        Eigen::Vector3f::UnitZ()));
}

} // namespace

PassableAreaNode::PassableAreaNode(const rclcpp::NodeOptions &options)
    : Node("passable_area", options), node_params_(RosParamLoader{}.load(*this)),
      config_(node_params_.config), processor_(config_), topic_config_(node_params_.topics) {
  result_publishers_.initialize(*this, topic_config_);
  debug_publishers_.initialize(*this, topic_config_);
  watchdog_.initialize(*this, topic_config_.input_cloud_topic, topic_config_.odom_topic);
  perf_stats_.initialize(*this);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  cloud_sub_.subscribe(this, topic_config_.input_cloud_topic, sensor_qos.get_rmw_qos_profile());
  odom_sub_.subscribe(this, topic_config_.odom_topic, sensor_qos.get_rmw_qos_profile());
  cloud_sub_.registerCallback(
      std::bind(&PassableAreaNode::onCloudObserved, this, std::placeholders::_1));
  odom_sub_.registerCallback(
      std::bind(&PassableAreaNode::onOdomObserved, this, std::placeholders::_1));
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(topic_config_.sync_queue_size), cloud_sub_, odom_sub_);
  sync_->registerCallback(std::bind(&PassableAreaNode::onSynced, this, std::placeholders::_1,
                                    std::placeholders::_2));
}

void PassableAreaNode::onCloudObserved(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  (void)msg;
  watchdog_.markCloud(now());
  perf_stats_.observeCloud();
}

void PassableAreaNode::onOdomObserved(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  (void)msg;
  watchdog_.markOdom(now());
  perf_stats_.observeOdom();
}

void PassableAreaNode::onSynced(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                                const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg) {
  watchdog_.markSynced(now());
  auto start = std::chrono::steady_clock::now();

  passable_area::core::PointCloud cloud;
  passable_area::core::Pose3D pose;
  if (!point_converter_.fromRos(*cloud_msg, cloud)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "PointCloud2 conversion failed");
    return;
  }
  if (!odom_converter_.fromRos(*odom_msg, pose)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Odometry conversion failed");
    return;
  }

  passable_area::core::FrameInput input;
  input.stamp = rclcpp::Time(cloud_msg->header.stamp).nanoseconds();
  input.base_pose_in_odom = pose;
  input.input_cloud_in_base = std::move(cloud);

  const auto output = processor_.update(input);
  if (!output.valid) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Processor dropped frame");
    return;
  }

  std_msgs::msg::Header odom_header;
  odom_header.stamp = cloud_msg->header.stamp;
  odom_header.frame_id = config_.odom_frame;
  result_publishers_.publish(output, odom_header);

  std_msgs::msg::Header base_gravity_header;
  base_gravity_header.stamp = cloud_msg->header.stamp;
  base_gravity_header.frame_id = config_.base_gravity_frame;
  debug_publishers_.publish(output, base_gravity_header);
  publishBaseGravityTransform(pose, cloud_msg->header.stamp);

  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                        .count();
  perf_stats_.record(ms);
}

void PassableAreaNode::publishBaseGravityTransform(
    const passable_area::core::Pose3D &base_pose_in_odom,
    const builtin_interfaces::msg::Time &stamp) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = config_.odom_frame;
  transform.child_frame_id = config_.base_gravity_frame;
  transform.transform.translation.x = base_pose_in_odom.position.x();
  transform.transform.translation.y = base_pose_in_odom.position.y();
  transform.transform.translation.z = base_pose_in_odom.position.z();

  const Eigen::Quaternionf yaw_only = YawOnlyQuaternion(base_pose_in_odom.orientation);
  transform.transform.rotation.x = yaw_only.x();
  transform.transform.rotation.y = yaw_only.y();
  transform.transform.rotation.z = yaw_only.z();
  transform.transform.rotation.w = yaw_only.w();
  tf_broadcaster_->sendTransform(transform);
}

} // namespace passable_area::interfaces::ros
