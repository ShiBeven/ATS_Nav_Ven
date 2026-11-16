#ifndef IO__GIMBAL_TOPIC_BRIDGE_HPP
#define IO__GIMBAL_TOPIC_BRIDGE_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <tuple>

#include "example_interfaces/msg/u_int8.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sp_msgs/msg/vision_target_msg.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "io/gimbal/gimbal.hpp"
#include "io/ros2/vision_target.hpp"

namespace io
{

class GimbalTopicBridge
{
public:
  explicit GimbalTopicBridge(
    float bullet_speed_fallback = 24.0F, bool require_attack_mode = true);
  ~GimbalTopicBridge();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);
  void send(io::VisionToGimbal cmd);
  void publishTarget(const VisionTargetState & target);

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void robotModeCallback(const example_interfaces::msg::UInt8::SharedPtr msg);
  void publishSafePose();
  void publishSafePoseLocked();

  mutable std::mutex mutex_;
  mutable std::mutex sample_mutex_;
  GimbalMode mode_ = GimbalMode::AUTO_AIM;
  GimbalState state_{};
  const bool require_attack_mode_;
  bool attack_mode_ = false;
  bool previous_attack_mode_ = false;
  bool pending_safe_pose_ = false;
  using TimedQuaternion = std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>;
  Eigen::Quaterniond last_returned_q_ = Eigen::Quaterniond::Identity();
  std::optional<TimedQuaternion> prev_sample_;
  std::optional<TimedQuaternion> latest_sample_;
  std::condition_variable sample_cv_;
  bool sample_new_ = false;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_joint_state_sub_;
  rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr robot_mode_sub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_pub_;
  rclcpp::Publisher<sp_msgs::msg::VisionTargetMsg>::SharedPtr target_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::thread spin_thread_;
  std::atomic<bool> quit_{false};
};

}  // namespace io

#endif  // IO__GIMBAL_TOPIC_BRIDGE_HPP
