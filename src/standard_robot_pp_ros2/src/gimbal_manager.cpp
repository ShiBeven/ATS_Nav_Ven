 

#include "standard_robot_pp_ros2/gimbal_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace standard_robot_pp_ros2
{

GimbalManagerNode::GimbalManagerNode(const rclcpp::NodeOptions & options)
: Node("gimbal_manager", options)
{
  RCLCPP_INFO(get_logger(), "Start GimbalManagerNode!");

  const auto timeout_ms = declare_parameter<int>("command_timeout_ms", 200);
  command_timeout_ = std::chrono::milliseconds(std::max<int64_t>(1, timeout_ms));

  cmd_sub_ = this->create_subscription<pb_rm_interfaces::msg::GimbalCmd>(
    "cmd_gimbal", 10,
    std::bind(&GimbalManagerNode::gimbalCmdCallback, this, std::placeholders::_1));

  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("cmd_gimbal_joint", 10);

  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
    const auto current_time = now();
    double delta_time = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_.last_update.nanoseconds() == 0) {
        state_.last_update = current_time;
        return;
      }
      delta_time = (current_time - state_.last_update).seconds();
      state_.last_update = current_time;
    }
    updateState(delta_time);
  });
}

void GimbalManagerNode::gimbalCmdCallback(const pb_rm_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  const auto current_time = now();
  std::lock_guard<std::mutex> lock(state_mutex_);
  const bool lease_expired = commandLeaseExpired(current_time);
  if (
    !lease_expired &&
    sourcePriority(msg->source) < sourcePriority(active_source_)) {
    RCLCPP_DEBUG(
      get_logger(), "Ignore gimbal command from lower-priority source %u while source %u is active",
      static_cast<unsigned int>(msg->source), static_cast<unsigned int>(active_source_));
    return;
  }

  active_source_ = msg->source;
  last_command_time_ = current_time;

  if (msg->pitch_type == pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE) {
    state_.pitch = msg->position.pitch;
    state_.pitch_velocity = msg->velocity.pitch;
    state_.pitch_acceleration = msg->pitch_acceleration;
    state_.pitch_ctrl.mode = ControlMode::POSITION;
  } else if (msg->pitch_type == pb_rm_interfaces::msg::GimbalCmd::VELOCITY) {
    state_.pitch_ctrl = {
      .mode = ControlMode::VELOCITY,
      .velocity = msg->velocity.pitch,
      .min_range = msg->velocity.pitch_min_range,
      .max_range = msg->velocity.pitch_max_range,
      .is_continuous =
        std::abs((msg->velocity.pitch_max_range - msg->velocity.pitch_min_range) - 2 * M_PI) <
        0.01};
    state_.pitch_velocity = msg->velocity.pitch;
    state_.pitch_acceleration = msg->pitch_acceleration;
  }

  if (msg->yaw_type == pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE) {
    state_.yaw = msg->position.yaw;
    state_.yaw_velocity = msg->velocity.yaw;
    state_.yaw_acceleration = msg->yaw_acceleration;
    state_.yaw_ctrl.mode = ControlMode::POSITION;
  } else if (msg->yaw_type == pb_rm_interfaces::msg::GimbalCmd::VELOCITY) {
    state_.yaw_ctrl = {
      .mode = ControlMode::VELOCITY,
      .velocity = msg->velocity.yaw,
      .min_range = msg->velocity.yaw_min_range,
      .max_range = msg->velocity.yaw_max_range,
      .is_continuous =
        std::abs((msg->velocity.yaw_max_range - msg->velocity.yaw_min_range) - 2 * M_PI) < 0.01};
    state_.yaw_velocity = msg->velocity.yaw;
    state_.yaw_acceleration = msg->yaw_acceleration;
  }
}

int GimbalManagerNode::sourcePriority(const uint8_t source)
{
  switch (source) {
    case pb_rm_interfaces::msg::GimbalCmd::SOURCE_VISION:
      return 3;
    case pb_rm_interfaces::msg::GimbalCmd::SOURCE_BEHAVIOR:
      return 2;
    case pb_rm_interfaces::msg::GimbalCmd::SOURCE_TELEOP:
      return 1;
    default:
      return 0;
  }
}

bool GimbalManagerNode::commandLeaseExpired(const rclcpp::Time & current_time) const
{
  return last_command_time_.nanoseconds() == 0 ||
         (current_time - last_command_time_).nanoseconds() >=
           std::chrono::duration_cast<std::chrono::nanoseconds>(command_timeout_).count();
}

void GimbalManagerNode::updateState(double delta_time)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_.pitch_ctrl.mode == ControlMode::VELOCITY) {
    state_.pitch = updateAxisPosition(state_.pitch, state_.pitch_ctrl, delta_time);
  }
  if (state_.yaw_ctrl.mode == ControlMode::VELOCITY) {
    state_.yaw = updateAxisPosition(state_.yaw, state_.yaw_ctrl, delta_time);
  }
  publishJointState();
}

double GimbalManagerNode::updateAxisPosition(double current, AxisControl & ctrl, double delta)
{
  const double displacement = ctrl.velocity * delta;
  double new_pos = current + displacement;

  if (ctrl.is_continuous) {
    const double range = ctrl.max_range - ctrl.min_range;
    new_pos = ctrl.min_range + std::fmod(new_pos - ctrl.min_range, range);
    return (new_pos < ctrl.min_range) ? new_pos + range : new_pos;
  }

  if (new_pos > ctrl.max_range) {
    ctrl.velocity = -ctrl.velocity;
    return ctrl.max_range;
  }
  if (new_pos < ctrl.min_range) {
    ctrl.velocity = -ctrl.velocity;
    return ctrl.min_range;
  }
  return new_pos;
}

void GimbalManagerNode::publishJointState()
{
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = now();
  msg.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
  msg.position = {state_.pitch, state_.yaw};
  msg.velocity = {state_.pitch_velocity, state_.yaw_velocity};
  msg.effort = {state_.pitch_acceleration, state_.yaw_acceleration};
  joint_pub_->publish(msg);
}
}  // namespace standard_robot_pp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::GimbalManagerNode)
