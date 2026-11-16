#include <fmt/core.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "io/camera.hpp"
#include "io/ros2/gimbal_topic_bridge.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/path.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/standard3.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder, const cv::Size & pattern_size)
{
  // 实车串口由 standard_robot_pp_ros2 统一打开并解析融合大包；
  // 标定程序通过 ROS 话题读取 IMU 姿态，避免再次以旧版协议独占串口。
  io::GimbalTopicBridge gimbal(24.0F, false);
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;
  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = gimbal.q(timestamp);

    // 在图像上显示欧拉角，用来判断imuabs系的xyz正方向，同时判断imu是否存在零漂
    auto img_with_ypr = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(img_with_ypr, fmt::format("Yaw {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Pitch {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Roll {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);
    cv::drawChessboardCorners(img_with_ypr, pattern_size, centers_2d, success);  // 显示识别结果
    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

    // 按“s”保存图片和对应四元数，按“q”退出程序
    cv::imshow("Press s to save, q to quit", img_with_ypr);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 保存图片和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, q);
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }

  // 离开该作用域时，camera和cboard会自动关闭
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  // 标定输出是用户生成数据，不适合写回安装空间。
  // 因此这里把相对输出目录显式落到“当前执行目录”下，绝对路径则保持不变。
  const auto output_folder_path =
    std::filesystem::path(output_folder).is_absolute() ?
    std::filesystem::path(output_folder) :
    std::filesystem::absolute(output_folder);
  output_folder = output_folder_path.string();

  // 新建输出文件夹
  std::filesystem::create_directories(output_folder_path);

  const auto yaml = tools::load(config_path);
  const cv::Size pattern_size(
    tools::read<int>(yaml, "pattern_cols"), tools::read<int>(yaml, "pattern_rows"));
  tools::logger()->info(
    "标定板尺寸为{}列{}行", pattern_size.width, pattern_size.height);
  // 主循环，保存图片和对应四元数
  capture_loop(config_path, output_folder, pattern_size);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
