#include "camera.hpp"

#include <stdexcept>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <cstdint>

#include "tools/yaml.hpp"

#ifdef SP_VISION_HAS_HIKROBOT
#include "hikrobot/hikrobot.hpp"
#endif

#ifdef SP_VISION_HAS_ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#endif

namespace io
{
#ifdef SP_VISION_HAS_ROS2
namespace
{
class RosImageCamera final : public CameraBase
{
public:
  explicit RosImageCamera(const std::string & topic)
  : node_(std::make_shared<rclcpp::Node>("vision_camera")), executor_()
  {
    subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        cv::Mat converted;
        const auto encoding = message->encoding;
        if (encoding == "bgr8" || encoding == "rgb8") {
          cv::Mat view(
            static_cast<int>(message->height), static_cast<int>(message->width), CV_8UC3,
            const_cast<uint8_t *>(message->data.data()), message->step);
          if (encoding == "rgb8") {
            cv::cvtColor(view, converted, cv::COLOR_RGB2BGR);
          } else {
            converted = view.clone();
          }
        } else if (encoding == "bgra8" || encoding == "rgba8") {
          cv::Mat view(
            static_cast<int>(message->height), static_cast<int>(message->width), CV_8UC4,
            const_cast<uint8_t *>(message->data.data()), message->step);
          cv::cvtColor(
            view, converted, encoding == "rgba8" ? cv::COLOR_RGBA2BGR : cv::COLOR_BGRA2BGR);
        } else {
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "Unsupported Gazebo camera image encoding: %s", encoding.c_str());
          return;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          image_ = std::move(converted);
          image_ready_ = true;
          ++frame_count_;
        }
        cv_.notify_one();
      });
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }

  ~RosImageCamera() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
  }

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!image_ready_ && rclcpp::ok()) cv_.wait_for(lock, std::chrono::milliseconds(100));
    img = image_.clone();
    image_ready_ = false;
    timestamp = std::chrono::steady_clock::now();
  }

  double camera_fps() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started_at_).count();
    return elapsed > 0.0 ? static_cast<double>(frame_count_) / elapsed : 0.0;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  cv::Mat image_;
  bool image_ready_ = false;
  uint64_t frame_count_ = 0;
  const std::chrono::steady_clock::time_point started_at_ = std::chrono::steady_clock::now();
};
}  // namespace
#endif

Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  if (const char * camera_mode = std::getenv("SP_VISION_CAMERA_MODE")) {
    camera_name = camera_mode;
  }

  if (camera_name == "hikrobot") {
#ifdef SP_VISION_HAS_HIKROBOT
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
#else
    throw std::runtime_error(
      "HikRobot camera backend is disabled; rebuild with SP_VISION_ENABLE_HIKROBOT=ON.");
#endif
  }
#ifdef SP_VISION_HAS_ROS2
  else if (camera_name == "ros_image") {
    const char * topic = std::getenv("SP_VISION_CAMERA_TOPIC");
    camera_ = std::make_unique<RosImageCamera>(topic ? topic : "front_industrial_camera/image");
  }
#endif

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

double Camera::camera_fps() const
{
  return camera_->camera_fps();
}

}  // namespace io
