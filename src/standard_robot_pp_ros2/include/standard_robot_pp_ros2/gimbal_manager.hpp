 

#ifndef STANDARD_ROBOT_PP_ROS2__GIMBAL_MANAGER_HPP_
#define STANDARD_ROBOT_PP_ROS2__GIMBAL_MANAGER_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>

#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace standard_robot_pp_ros2
{

enum class ControlMode { POSITION, VELOCITY };

struct AxisControl
{
  ControlMode mode = ControlMode::POSITION;
  float velocity = 0.0f;       // Desired velocity (rad/s)
  float min_range = 0.0f;      // Minimum angle limit (rad)
  float max_range = 0.0f;      // Maximum angle limit (rad)
  bool is_continuous = false;  // Continuous rotation mode
};

class GimbalManagerNode : public rclcpp::Node
{
public:
  explicit GimbalManagerNode(const rclcpp::NodeOptions & options);
  ~GimbalManagerNode() override = default;

private:
  void gimbalCmdCallback(const pb_rm_interfaces::msg::GimbalCmd::SharedPtr msg);

  void updateState(double delta_time);

  void publishJointState();

  double updateAxisPosition(double current, AxisControl & ctrl, double delta);

  static int sourcePriority(uint8_t source);
  bool commandLeaseExpired(const rclcpp::Time & current_time) const;

  struct
  {
    double pitch = 0.0;
    double yaw = 0.0;
    double pitch_velocity = 0.0;
    double yaw_velocity = 0.0;
    double pitch_acceleration = 0.0;
    double yaw_acceleration = 0.0;
    AxisControl pitch_ctrl;
    AxisControl yaw_ctrl;
    rclcpp::Time last_update;
  } state_;

  uint8_t active_source_ = pb_rm_interfaces::msg::GimbalCmd::SOURCE_UNKNOWN;
  rclcpp::Time last_command_time_;
  std::chrono::milliseconds command_timeout_{200};
  mutable std::mutex state_mutex_;

  rclcpp::Subscription<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__GIMBAL_MANAGER_HPP_
