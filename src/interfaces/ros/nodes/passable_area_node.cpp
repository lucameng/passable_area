#include "passable_area/interfaces/ros/nodes/passable_area_node.hpp"

#include <chrono>

namespace passable_area::interfaces::ros {

PassableAreaNode::PassableAreaNode(const rclcpp::NodeOptions &options)
    : Node("passable_area", options), config_(param_loader_.load(*this)), processor_(config_) {
  input_cloud_topic_ = declare_parameter("input_cloud_topic", std::string("/LOC_BODY_POINTS"));
  odom_topic_ = declare_parameter("odom_topic", std::string("/ODOM"));
  sync_queue_size_ = declare_parameter("sync.queue_size", 10);

  result_publishers_.initialize(*this);
  debug_publishers_.initialize(*this);
  watchdog_.initialize(*this, input_cloud_topic_, odom_topic_);
  perf_stats_.initialize(*this);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  cloud_sub_.subscribe(this, input_cloud_topic_, sensor_qos.get_rmw_qos_profile());
  odom_sub_.subscribe(this, odom_topic_, sensor_qos.get_rmw_qos_profile());
  cloud_sub_.registerCallback(
      std::bind(&PassableAreaNode::onCloudObserved, this, std::placeholders::_1));
  odom_sub_.registerCallback(
      std::bind(&PassableAreaNode::onOdomObserved, this, std::placeholders::_1));
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size_), cloud_sub_, odom_sub_);
  sync_->registerCallback(std::bind(&PassableAreaNode::onSynced, this, std::placeholders::_1,
                                    std::placeholders::_2));
}

void PassableAreaNode::onCloudObserved(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  (void)msg;
  watchdog_.markCloud(now());
}

void PassableAreaNode::onOdomObserved(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  (void)msg;
  watchdog_.markOdom(now());
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
  input.base_pose_in_local = pose;
  input.input_cloud_in_base = std::move(cloud);

  const auto output = processor_.update(input);
  if (!output.valid) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Processor dropped frame");
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = cloud_msg->header.stamp;
  header.frame_id = config_.gravity_frame;
  result_publishers_.publish(output, header);
  debug_publishers_.publish(output, header);

  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                        .count();
  perf_stats_.record(ms);
}

} // namespace passable_area::interfaces::ros
