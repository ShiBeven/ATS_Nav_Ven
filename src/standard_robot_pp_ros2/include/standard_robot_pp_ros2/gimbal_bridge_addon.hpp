#ifndef STANDARD_ROBOT_PP_ROS2__GIMBAL_BRIDGE_ADDON_HPP_
#define STANDARD_ROBOT_PP_ROS2__GIMBAL_BRIDGE_ADDON_HPP_

// 涓插彛铻嶅悎鏀归€狅細瀵艰埅鑺傜偣鏂板鐨勮瑙夋ˉ鎺ラ€昏緫銆?// 浣跨敤鏂瑰紡锛氭湰鏂囦欢瀹氫箟鐙珛绫?GimbalBridgeAddon锛屽湪
// standard_robot_pp_ros2.hpp 涓互鎴愬憳褰㈠紡鎸佹湁锛堣 nav_bridge/README.md 姝ラ 2锛夛紝
// 涓嶈鎶婃湰鏂囦欢鍐呭澶嶅埗杩?StandardRobotPpRos2Node 绫诲畾涔夊唴閮ㄣ€?
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef_addon.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace standard_robot_pp_ros2
{

// ============================================================
// 鏂板璇濋瀹氫箟
// ============================================================
// /serial/gimbal_state : std_msgs/Float64MultiArray锛堢數鎺?>瑙嗚锛?//   绱㈠紩: [0]=mode [1..4]=q_w,q_x,q_y,q_z [5]=yaw [6]=yaw_vel
//          [7]=pitch [8]=pitch_vel [9]=bullet_speed [10]=bullet_count
//   缁村害: data.size()==11锛涙棤鏈夋晥甯ф椂涓嶅彂甯冦€?//   璇存槑锛歨eader 鐢?ROS now()锛堝埌杈炬椂鍒伙級锛岃瑙?slerp 渚濊禆鍒拌揪鏃跺簭锛?//         涓嶉€忎紶涓嬩綅鏈?time_stamp锛堝崗璁瓧娈典粛鎸夊師鏍蜂繚瀛橈級銆?//
// /serial/gimbal_cmd : std_msgs/Float64MultiArray锛堣瑙?>鐢垫帶锛?//   绱㈠紩: [0]=mode [1]=yaw [2]=yaw_vel [3]=yaw_acc
//          [4]=pitch [5]=pitch_vel [6]=pitch_acc
//   缁村害: data.size()==7锛涚敱鐙珛绾跨▼鎸夋渶鏂板€肩粍鍖呭彂閫併€?//   璇濋鍐呮暟鍊间竴寰嬪姬搴?寮у害姣忕/寮у害姣忎簩娆℃柟绉掋€?//
// 璇濋绫诲瀷閫夌敤 Float64MultiArray 鐨勫師鍥狅細瑙嗚浠撳簱宸叉湁 sp_msgs锛?// 浣嗗鑸晶涓哄噺灏戞帴鍙ｅ寘鑰﹀悎锛屽厛鐢ㄦ爣鍑嗘秷鎭壙杞斤紱鍚庣画濡傞渶寮虹被鍨?// 鍙湪 sp_msgs 涓畾涔?GimbalStateMsg/GimbalCmdMsg 鏇挎崲锛屽瓧娈典笉鍙樸€?
class GimbalBridgeAddon
{
public:
  // 视觉指令失联判定窗口，与协议规格书 3.2 电控端 >=100ms 超时语义对齐。
  // 超过该时间无新话题帧则停发（电控随之进入失联处理），绝不无限重发旧帧。
  static constexpr int kGimbalCmdTimeoutMs = 100;

  using SerialSendFn = std::function<void(const std::vector<uint8_t> &)>;

  // node: 瀹夸富 StandardRobotPpRos2Node锛坱his锛夈€?  // serial_send_fn 蹇呴』鍦ㄦ瀯閫犳椂娉ㄥ叆锛氬彂閫佺嚎绋嬪湪鏋勯€犲嚱鏁颁腑宸插惎鍔紝
  // serial_send_fn 必须在构造时注入：发送线程在构造函数中已启动，
  // 事后调用 setSerialSendFn 会对 std::function 产生并发读写（UB）。
  GimbalBridgeAddon(rclcpp::Node & node, const SerialSendFn & serial_send_fn)
  : node_(node),
    serial_send_fn_(serial_send_fn),
    gimbal_state_pub_(
      node.create_publisher<std_msgs::msg::Float64MultiArray>("serial/gimbal_state", 10)),
    gimbal_cmd_sub_(node.create_subscription<std_msgs::msg::Float64MultiArray>(
      "serial/gimbal_cmd", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(gimbal_cmd_mutex_);
        latest_gimbal_cmd_ = *msg;
        latest_gimbal_cmd_time_ = std::chrono::steady_clock::now();
      }))
  {
    // 发送线程：独立于现有 sendData 的 5ms 周期，按视觉节拍转发。
    // 不与 SendRobotCmdData 复用线程，避免导航发送节拍影响视觉指令延迟。
    gimbal_cmd_send_thread_ = std::thread([this]() { gimbalCmdSendLoop(); });
  }

  ~GimbalBridgeAddon()
  {
    gimbal_cmd_running_ = false;
    if (gimbal_cmd_send_thread_.joinable()) gimbal_cmd_send_thread_.join();
  }

  // receiveData() 的 ID_GIMBAL_STATE 分支调用 publishGimbalState()。
  void publishGimbalState(const ReceiveGimbalStateData & d)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(11);
    msg.data.push_back(static_cast<double>(d.data.mode));
    for (int i = 0; i < 4; ++i) msg.data.push_back(static_cast<double>(d.data.q[i]));
    msg.data.push_back(static_cast<double>(d.data.yaw));
    msg.data.push_back(static_cast<double>(d.data.yaw_vel));
    msg.data.push_back(static_cast<double>(d.data.pitch));
    msg.data.push_back(static_cast<double>(d.data.pitch_vel));
    msg.data.push_back(static_cast<double>(d.data.bullet_speed));
    msg.data.push_back(static_cast<double>(d.data.bullet_count));
    gimbal_state_pub_->publish(msg);
  }

private:
  void gimbalCmdSendLoop()
  {
    // 帧头只需初始化一次；time_stamp 每包刷新；CRC16 每包重算。
    send_cmd_.frame_header.sof = SOF_SEND;
    send_cmd_.frame_header.id = ID_GIMBAL_CMD;
    send_cmd_.frame_header.len = sizeof(SendGimbalCmdData) - 6;

    const auto period = std::chrono::milliseconds(5);  // 涓婇檺 200Hz 杞彂
    auto next_tick = std::chrono::steady_clock::now();
    while (rclcpp::ok() && gimbal_cmd_running_) {
      next_tick += period;
      bool has_cmd = false;
      {
        std::lock_guard<std::mutex> lock(gimbal_cmd_mutex_);
        const bool fresh = latest_gimbal_cmd_time_ != std::chrono::steady_clock::time_point{} &&
                           (std::chrono::steady_clock::now() - latest_gimbal_cmd_time_) <
                             std::chrono::milliseconds(kGimbalCmdTimeoutMs);
        if (latest_gimbal_cmd_.data.size() == 7 && fresh) {
          has_cmd = true;
          send_cmd_.data.mode = static_cast<uint8_t>(latest_gimbal_cmd_.data[0]);
          send_cmd_.data.yaw = static_cast<float>(latest_gimbal_cmd_.data[1]);
          send_cmd_.data.yaw_vel = static_cast<float>(latest_gimbal_cmd_.data[2]);
          send_cmd_.data.yaw_acc = static_cast<float>(latest_gimbal_cmd_.data[3]);
          send_cmd_.data.pitch = static_cast<float>(latest_gimbal_cmd_.data[4]);
          send_cmd_.data.pitch_vel = static_cast<float>(latest_gimbal_cmd_.data[5]);
          send_cmd_.data.pitch_acc = static_cast<float>(latest_gimbal_cmd_.data[6]);
        }
      }
      if (has_cmd && serial_send_fn_) {
        send_cmd_.time_stamp = static_cast<uint32_t>(node_.now().nanoseconds() / 1000000ULL);
        // 甯уご CRC8锛堝唴瀹瑰浐瀹氾紝浣嗕繚鎸佷笌鐜版湁 sendData 鐩稿悓鐨勬瘡鍖呰绠楁柟寮忥紝
        // 闃叉缁撴瀯浣撳唴瀛樿鎰忓鏀瑰姩鍚庢牎楠屽け閰嶏級
        crc8::append_CRC8_check_sum(reinterpret_cast<uint8_t *>(&send_cmd_), sizeof(HeaderFrame));
        // 鏁村寘 CRC16
        crc16::append_CRC16_check_sum(
          reinterpret_cast<uint8_t *>(&send_cmd_), sizeof(SendGimbalCmdData));
        serial_send_fn_(toVector(send_cmd_));
      }
      std::this_thread::sleep_until(next_tick);
    }
  }

private:
  rclcpp::Node & node_;
  std::function<void(const std::vector<uint8_t> &)> serial_send_fn_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_cmd_sub_;

  std::mutex gimbal_cmd_mutex_;
  std_msgs::msg::Float64MultiArray latest_gimbal_cmd_;
  // 话题最后一帧到达时刻：用于失联检测（对应电控端 >=100ms 超时语义）。
  std::chrono::steady_clock::time_point latest_gimbal_cmd_time_{};
  SendGimbalCmdData send_cmd_{};
  std::thread gimbal_cmd_send_thread_;
  std::atomic<bool> gimbal_cmd_running_{true};
};

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__GIMBAL_BRIDGE_ADDON_HPP_
