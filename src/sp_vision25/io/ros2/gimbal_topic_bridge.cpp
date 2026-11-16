#include "gimbal_topic_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace io
{

GimbalTopicBridge::GimbalTopicBridge(float bullet_speed_fallback, bool require_attack_mode)
: require_attack_mode_(require_attack_mode), attack_mode_(!require_attack_mode)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  node_ = std::make_shared<rclcpp::Node>("vision_auto_aim");
  if (const char * speed = std::getenv("SP_VISION_BULLET_SPEED")) {
    try {
      bullet_speed_fallback = std::stof(speed);
    } catch (const std::exception &) {
      RCLCPP_WARN(node_->get_logger(), "Invalid SP_VISION_BULLET_SPEED='%s'", speed);
    }
  }
  state_.bullet_speed = std::max(1.0F, bullet_speed_fallback);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "serial/imu", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "serial/gimbal_joint_state", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCallback(msg); });
  sim_joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCallback(msg); });
  robot_mode_sub_ = node_->create_subscription<example_interfaces::msg::UInt8>(
    "decision/robot_mode", 10,
    [this](example_interfaces::msg::UInt8::SharedPtr msg) { robotModeCallback(msg); });
  gimbal_pub_ = node_->create_publisher<pb_rm_interfaces::msg::GimbalCmd>("cmd_gimbal", 10);
  shoot_pub_ = node_->create_publisher<example_interfaces::msg::UInt8>("cmd_shoot", 10);
  target_pub_ = node_->create_publisher<sp_msgs::msg::VisionTargetMsg>("vision/target", 10);

  spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
  tf_buffer_->setUsingDedicatedThread(true);
  tools::logger()->info(
    "[GimbalTopicBridge] require_attack_mode={} ({})", require_attack_mode_,
    require_attack_mode_ ? "follow decision/robot_mode" : "independent vision control");
  tools::logger()->info(
    "[GimbalTopicBridge] input serial/imu + serial/gimbal_joint_state; output cmd_gimbal, "
    "cmd_shoot, vision/target");
}

GimbalTopicBridge::~GimbalTopicBridge()
{
  publishSafePose();
  quit_ = true;
  sample_cv_.notify_all();
  if (rclcpp::ok()) rclcpp::shutdown();
  if (spin_thread_.joinable()) spin_thread_.join();
}

GimbalMode GimbalTopicBridge::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState GimbalTopicBridge::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string GimbalTopicBridge::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

void GimbalTopicBridge::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  Eigen::Quaterniond q(
    msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-6) return;
  q.normalize();
  const auto t = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    if (latest_sample_) prev_sample_ = latest_sample_;
    latest_sample_ = TimedQuaternion{q, t};
    sample_new_ = true;
  }
  sample_cv_.notify_all();
}

void GimbalTopicBridge::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.size() != msg->position.size()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "gimbal_yaw_joint") state_.yaw = static_cast<float>(msg->position[i]);
    if (msg->name[i] == "gimbal_pitch_joint") state_.pitch = static_cast<float>(msg->position[i]);
    if (i < msg->velocity.size()) {
      if (msg->name[i] == "gimbal_yaw_joint") {
        state_.yaw_vel = static_cast<float>(msg->velocity[i]);
      }
      if (msg->name[i] == "gimbal_pitch_joint") {
        state_.pitch_vel = static_cast<float>(msg->velocity[i]);
      }
    }
  }
}

void GimbalTopicBridge::robotModeCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
{
  if (!require_attack_mode_) return;

  constexpr uint8_t kAttackMode = 1;
  constexpr uint8_t kDefendMode = 2;
  constexpr uint8_t kMoveMode = 3;
  const bool next_attack_mode = msg->data == kAttackMode;
  bool exited_attack_mode = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    exited_attack_mode = previous_attack_mode_ && !next_attack_mode;
    pending_safe_pose_ = exited_attack_mode;
    previous_attack_mode_ = next_attack_mode;
    attack_mode_ = next_attack_mode;
    mode_ = attack_mode_ ? GimbalMode::AUTO_AIM : GimbalMode::IDLE;
    if (exited_attack_mode) publishSafePoseLocked();
  }
  if (msg->data != kAttackMode && msg->data != kDefendMode && msg->data != kMoveMode) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "Unsupported decision/robot_mode=%u; vision gimbal/fire output is disabled", msg->data);
  }
}

Eigen::Quaterniond GimbalTopicBridge::q(std::chrono::steady_clock::time_point t)
{
  constexpr auto kWaitTimeout = std::chrono::milliseconds(100);
  std::unique_lock<std::mutex> lock(sample_mutex_);
  if (prev_sample_ && latest_sample_) {
    const auto & [q_a, t_a] = *prev_sample_;
    const auto & [q_b, t_b] = *latest_sample_;
    if (t_a < t && t <= t_b) {
      const double span = tools::delta_time(t_a, t_b);
      if (std::abs(span) > 1e-6) {
        const double offset = std::clamp(tools::delta_time(t_a, t) / span, 0.0, 1.0);
        last_returned_q_ = q_a.slerp(offset, q_b).normalized();
      } else {
        last_returned_q_ = q_b;
      }
      return last_returned_q_;
    }
  }
  if (latest_sample_) {
    last_returned_q_ = std::get<0>(*latest_sample_);
    return last_returned_q_;
  }
  lock.unlock();
  try {
    const auto transform = tf_buffer_->lookupTransform(
      "map", "gimbal_pitch", tf2::TimePointZero, std::chrono::milliseconds(20));
    const auto & rotation = transform.transform.rotation;
    return Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z).normalized();
  } catch (const tf2::TransformException &) {
  }
  lock.lock();
  sample_cv_.wait_for(lock, kWaitTimeout, [this] { return sample_new_ || quit_.load(); });
  sample_new_ = false;
  if (latest_sample_) last_returned_q_ = std::get<0>(*latest_sample_);
  return last_returned_q_;
}

void GimbalTopicBridge::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_safe_pose_) {
    publishSafePoseLocked();
  } else if (attack_mode_ && control) {
    pb_rm_interfaces::msg::GimbalCmd cmd;
    cmd.header.stamp = node_->now();
    cmd.yaw_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
    cmd.pitch_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
    cmd.source = pb_rm_interfaces::msg::GimbalCmd::SOURCE_VISION;
    cmd.position.yaw = yaw;
    cmd.position.pitch = pitch;
    cmd.velocity.yaw = yaw_vel;
    cmd.velocity.pitch = pitch_vel;
    cmd.yaw_acceleration = yaw_acc;
    cmd.pitch_acceleration = pitch_acc;
    gimbal_pub_->publish(cmd);
  }

  example_interfaces::msg::UInt8 shoot;
  shoot.data = static_cast<uint8_t>(attack_mode_ && control && fire);
  shoot_pub_->publish(shoot);
}

void GimbalTopicBridge::publishSafePose()
{
  std::lock_guard<std::mutex> lock(mutex_);
  publishSafePoseLocked();
}

void GimbalTopicBridge::publishSafePoseLocked()
{
  const bool should_publish = attack_mode_ || pending_safe_pose_;
  attack_mode_ = false;
  pending_safe_pose_ = false;
  previous_attack_mode_ = false;
  if (!should_publish || !gimbal_pub_ || !shoot_pub_) return;

  pb_rm_interfaces::msg::GimbalCmd cmd;
  cmd.header.stamp = node_->now();
  cmd.yaw_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  cmd.pitch_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  cmd.source = pb_rm_interfaces::msg::GimbalCmd::SOURCE_VISION;
  cmd.position.yaw = 0.0f;
  cmd.position.pitch = 0.0f;
  cmd.velocity.yaw = 0.0f;
  cmd.velocity.pitch = 0.0f;
  cmd.yaw_acceleration = 0.0f;
  cmd.pitch_acceleration = 0.0f;
  gimbal_pub_->publish(cmd);

  example_interfaces::msg::UInt8 shoot;
  shoot.data = 0;
  shoot_pub_->publish(shoot);
}

void GimbalTopicBridge::send(io::VisionToGimbal cmd)
{
  send(
    cmd.mode != 0, cmd.mode == 2, cmd.yaw, cmd.yaw_vel, cmd.yaw_acc, cmd.pitch, cmd.pitch_vel,
    cmd.pitch_acc);
}

void GimbalTopicBridge::publishTarget(const VisionTargetState & target)
{
  sp_msgs::msg::VisionTargetMsg msg;
  msg.timestamp = node_->now();
  msg.tracking = target.tracking;
  msg.nav_hold = false;
  msg.fire_permitted = target.fire_permitted;
  msg.target_id = target.target_id;
  msg.suggested_goal_index = target.suggested_goal_index;
  msg.target_type = target.target_type;
  msg.confidence = static_cast<float>(target.confidence);
  msg.target_distance = static_cast<float>(target.target_distance);
  msg.target_yaw = static_cast<float>(target.target_yaw);
  msg.target_pitch = static_cast<float>(target.target_pitch);
  msg.target_position_gimbal.x = target.target_position_gimbal.x();
  msg.target_position_gimbal.y = target.target_position_gimbal.y();
  msg.target_position_gimbal.z = target.target_position_gimbal.z();

  if (target.tracking) {
    geometry_msgs::msg::PointStamped source;
    source.header.stamp = msg.timestamp;
    source.header.frame_id = "gimbal_pitch";
    source.point = msg.target_position_gimbal;
    try {
      const auto transform = tf_buffer_->lookupTransform(
        "map", source.header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(20));
      geometry_msgs::msg::PointStamped mapped;
      tf2::doTransform(source, mapped, transform);
      msg.target_position_map = mapped.point;
      msg.has_target_position_map = true;
      msg.target_position_map_frame = "map";
      msg.nav_hold = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 3000,
        "Cannot transform vision target from gimbal_pitch to map: %s", ex.what());
    }
  }
  target_pub_->publish(msg);
}

}  // namespace io
