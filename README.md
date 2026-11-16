# ATS 一体化工作区使用指南

> 最后核对日期：2026-10-26。本 README 已按新导航迁移后的目录、话题和启动入口逐条核对，可直接复制执行。
> 所有命令均在 **Ubuntu 22.04 / ROS 2 Humble（Bash）** 下编写；Windows 侧仅作为代码编辑环境。

---

## 目录

1. [项目简介](#1-项目简介)
2. [目录结构与包职责](#2-目录结构与包职责)
3. [运行环境与依赖要求](#3-运行环境与依赖要求)
4. [首次部署与编译](#4-首次部署与编译)
5. [启动与运行](#5-启动与运行)
6. [配置文件总表](#6-配置文件总表)
7. [调试与验证](#7-调试与验证)
8. [占位符与路径替换规则](#8-占位符与路径替换规则)
9. [常见问题与回滚](#9-常见问题与回滚)
10. [延伸文档索引](#10-延伸文档索引)

---

## 1. 项目简介

ATS 是 RoboMaster 哨兵机器人**视觉 + 导航一体化** ROS 2 工作区。2026-09-19 起，导航（原 `ATS_2026_snetry_test-main`）与视觉（原 `sp_vision25-debug`）合并为一个 colcon 工作区：一次 `colcon build` 同时编译两侧，运行时通过 ROS 2 话题桥接，一条 launch 同时拉起。

### 1.1 核心功能

| 功能 | 实现包 | 说明 |
|---|---|---|
| 串口通信 | `src/standard_robot_pp_ros2` | 按参数文件独占一条 USB 串口（当前默认 `/dev/ttyACM1`、`115200`，上车前需确认）；与下位机交换 13 种收包 + 1 种发包 |
| 自瞄 | `src/sp_vision25` | OpenVINO 推理 + EKF 跟踪 + PnP 解算 + 弹道预测 |
| 导航与定位 | `src/pb2025_sentry_nav` + `src/interfaces/sp_msgs` | Livox Mid360 + point_lio 里程计 + small_gicp 重定位 + Nav2 全局/局部规划 |
| 行为决策 | `src/pb2025_sentry_behavior` | 行为树，根据血量/时间/视觉目标选择巡逻、补给、跟踪等目标点 |
| 统一入口 | `src/ats_bringup` | `ats_full.launch.py` 拉起串口节点、云台管理器和视觉主程序 |
| 完整导航入口 | `src/pb2025_sentry_bringup` | `bringup.launch.py` 拉起串口 + Nav2 + 行为树 + rosbag 记录 |
| Gazebo 仿真 | `src/rmu_gazebo_simulator` + `src/dependencies/rmoss_gazebo` | `gazebo_navigation.launch.py` 启动 Gazebo Fortress、Nav2、行为树和 ROS Image 视觉输入 |

### 1.2 当前数据流（核心架构）

```text
[下位机电控]
   ↕ 唯一 USB 串口（设备名和波特率以 `standard_robot_pp_ros2.yaml` 为准，帧格式 HeaderFrame+time_stamp+data+CRC16）
[串口节点 standard_robot_pp_ros2]   ← 13 种收包 + 1 种发包协议
   ├─ 发布 /serial/gimbal_joint_state（sensor_msgs/JointState）
   ├─ 订阅 /cmd_vel                 （底盘速度）
   └─ 订阅 cmd_gimbal_joint         （云台 JointState 指令）
       ▲
   gimbal_manager <- cmd_gimbal（行为树和视觉共用）
       ▲                     ▲ cmd_shoot
   auto_aim_debug_mpc -> vision/target -> 行为树视觉接管
   ↕ ROS 2 (DDS)
[导航速度链]
   Nav2 MPPI -> velocity_smoother -> fake_vel_transform
   -> cmd_vel_gimbal_yaw_odom -> sentry_chassis_vel_transform -> /cmd_vel
```

要点：
- **视觉主程序不自行打开串口**；实车从 `/serial/imu`、`/serial/gimbal_joint_state` 取状态，通过 `cmd_gimbal`、`cmd_shoot` 发控制。`gimbal_manager` 将 `cmd_gimbal` 转成串口节点接收的 `cmd_gimbal_joint`。
- **导航速度由 `sentry_chassis_vel_transform` 输出到绝对话题 `/cmd_vel`**，串口节点直接订阅该话题。
- `auto_aim_debug_mpc` 主循环持续发布 `sp_msgs/msg/VisionTargetMsg` 到 `vision/target`；仅当目标 TF 能转换到 `map` 时才设置 `nav_hold=true`。

### 1.3 适用场景

- RoboMaster 哨兵机器人实车部署（HikRobot 相机 + Livox Mid360 + C 板串口）；
- WSL2 / Ubuntu 22.04 下的仿真、loopback 无硬件回归调试；
- 视觉算法离线调试（WebDebugger 浏览器调参、录包回放）。

---

## 2. 目录结构与包职责

```text
ATS/
├── README.md                        # 本使用指南
├── build_release.sh                 # 全工作区 Release 编译入口
├── AGENTS.md                        # 代码助手工作区约定（搜索策略/验证策略）
├── NAV2.sh                          # 实机导航守护脚本（自动重启 rm_navigation_reality_launch.py）
├── mapping.sh                       # 建图守护脚本（slam:=True）
├── pp_ros2.sh                       # 仅串口节点守护脚本
├── rosbag.sh                        # rosbag 记录守护脚本
├── ATS_融合方案.md                   # 融合方案总览
├── 通信协议梳理报告.md                # 通信协议梳理报告
├── src/
│   ├── ats_bringup/                 # 统一启动入口（ats_full.launch.py）
│   ├── sp_vision25/                 # 视觉包（自瞄、MPC、相机和 ROS 话题桥接）
│   ├── standard_robot_pp_ros2/      # 底盘串口节点（/cmd_vel 与 JointState 接口）
│   ├── pb2025_sentry_nav/           # 导航功能包集合（pb2025_nav_bringup、trajectory_optimizer 等）
│   ├── pb2025_sentry_behavior/      # 行为树
│   ├── pb2025_sentry_bringup/       # 完整导航启动（bringup.launch.py、地图、PCD、RViz）
│   ├── pb2025_robot_description/    # 机器人 xmacro 模型
│   ├── interfaces/pb_rm_interfaces/ # 自定义 ROS 接口
│   ├── dependencies/                # 第三方依赖（rmoss_core、BehaviorTree.ROS2 等）
│   ├── loopback_sim/                # nav2_loopback_sim 无硬件仿真
│   └── tools/                       # pcd2pgm、rosbag2_composable_recorder、键盘控制等工具
├── scripts/
│   └── codex_brief.sh               # 生成工作区/包级上下文摘要（供代码助手）
├── docs/                            # 历史 README 与迁移、融合文档存档
├── patterns/                        # 代码助手提示词模板
└── Start/                           # 串口融合方案与协议规格书存档（决策记录/桥接补丁）
```

`src/pb2025_sentry_nav` 内含的主要子目录（均为独立包）：

```text
src/pb2025_sentry_nav/
├── fake_vel_transform/              # 虚拟速度参考坐标系（云台扫描自旋）
├── ign_sim_pointcloud_tool/         # 仿真器点云处理工具
├── livox_ros_driver2/               # Livox 雷达驱动
├── loam_interface/                  # point_lio 里程计接口
├── pb2025_nav_bringup/              # 导航启动/参数/地图/PCD/RViz
├── pb2025_sentry_nav/               # 元包
├── pb_nav2_plugins/                 # Nav2 插件
├── trajectory_optimizer/            # B 样条平滑、轨迹 profile 与速度 governor
├── pb_teleop_twist_joy/             # 手柄控制
├── pointcloud_to_laserscan/         # terrain_map → LaserScan（仅 SLAM 模式）
├── point_lio/                       # 里程计
├── sensor_scan_generation/          # 点云坐标系变换
├── small_gicp_relocalization/       # 重定位
├── terrain_analysis/                # 车体 4m 内地形分析
├── terrain_analysis_ext/            # 车体 4m 外地形分析
└── （sp_msgs 已迁移到 src/interfaces/sp_msgs）
```

---

## 3. 运行环境与依赖要求

| 依赖 | 版本/路径要求 | 说明 |
|---|---|---|
| 操作系统 | Ubuntu 22.04 或 WSL2 | 命令均为 Bash |
| ROS 2 | Humble | `source /opt/ros/humble/setup.bash` |
| HikRobot SDK | 实车构建需 `HIKROBOT_SDK_ROOT`；Gazebo-only 可关闭后端 | CMake 搜索顺序见下 |
| OpenVINO | 2025.4 | `OpenVINO_DIR` 可指定到安装目录下的 `runtime/cmake`；默认尝试实验室归档路径 |
| OpenCV / fmt / Eigen3 / spdlog / yaml-cpp / nlohmann_json | 系统安装 | `find_package` 必需 |
| small_gicp | 源码安装 | 重定位依赖 |
| 硬件 | C 板 USB 串口（当前参数默认 `/dev/ttyACM1`）、HikRobot 相机、Livox Mid360 | 仅实机需要 |

**HikRobot SDK 搜索路径**（`SP_VISION_ENABLE_HIKROBOT=ON` 时，见 `src/sp_vision25/io/CMakeLists.txt`）：

1. 环境变量 `$HIKROBOT_SDK_ROOT`
2. `/opt/MVS`
3. `/usr/local/MVS`
4. `/opt/hikrobot/MVS`
5. `/opt/HikRobot/MVS`
6. `src/dependencies/hik_camera_ros2_driver/hikSDK`
7. `src/sp_vision25/io/hikrobot`

**OpenVINO 默认路径**（见 `src/sp_vision25/CMakeLists.txt`）：

```text
$HOME/openvino_toolkit_ubuntu22_2025.4.1.20426.82bbf0292c5_x86_64/runtime/cmake
```

路径不同时，设置 `OpenVINO_DIR` 指向包含 `OpenVINOConfig.cmake` 的 `runtime/cmake` 目录；无需修改源码。

---

## 4. 首次部署与编译

### 4.1 安装 ROS 2 与系统依赖（一次性）

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

```bash
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

用途：安装工作区声明的全部系统依赖。

### 4.2 安装 small_gicp（一次性，导航重定位依赖）

```bash
sudo apt install -y libeigen3-dev libomp-dev
git clone https://github.com/koide3/small_gicp.git /tmp/small_gicp
cd /tmp/small_gicp
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

### 4.3 配置 HikRobot SDK（一次性）

安装 HikRobot MVS SDK 到 `/opt/MVS`（或 `/usr/local/MVS`），然后：

```bash
export HIKROBOT_SDK_ROOT=/opt/MVS
```

建议把该行写入 `~/.bashrc`，避免每次编译重复输入。

### 4.4 配置串口 udev 规则（一次性，实机）

```bash
cd /path/to/ATS/src/standard_robot_pp_ros2
./script/create_udev_rules.sh
```

用途：把 RoboMaster C 板 USB 串口固定映射为稳定设备名并加入 `dialout` 组。当前脚本仍生成 `/dev/ttyACM0`，若采用参数文件默认的 `/dev/ttyACM1`，请同步调整脚本或参数后再启动。执行后**重新插拔串口或注销重登**使组权限生效。

> ⚠️ `/path/to/ATS` 为占位符，见[第 8 节](#8-占位符与路径替换规则)。

### 4.5 全量编译（统一 Release）

```bash
cd ~/ATS
bash ./build_release.sh
```

脚本会加载 ROS 2 Humble、运行 `rosdep install` 并以 Release 模式编译整个 `src/` 工作区。首次运行可能需要输入 sudo 密码。默认使用 2 个并行 worker；内存较少时可设 `BUILD_WORKERS=1`。

OpenVINO 使用默认路径时无需额外参数。若安装在其他位置，显式传入 CMake 配置目录：

```bash
cd ~/ATS
OpenVINO_DIR="$HOME/intel/openvino_2025/runtime/cmake" BUILD_WORKERS=2 bash ./build_release.sh
```

脚本会在找到 HikRobot SDK 时编译相机驱动；找不到时自动关闭该驱动，视觉算法和 Gazebo ROS Image 相机仍会编译。`small_gicp` 需要按 4.2 节预先安装。完成后执行 `source install/setup.bash`。

### 4.6 只重编改动包（推荐日常使用）

```bash
cd /path/to/ATS
source install/setup.bash
colcon build --packages-select sp_vision25 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```bash
cd /path/to/ATS
source install/setup.bash
colcon build --packages-select standard_robot_pp_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

用途：分别重编视觉包 / 串口节点。其他包同理替换包名（见 8.4 节可选值表）。

### 4.7 每次打开新终端加载工作区环境

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ATS
source install/setup.bash
```

> 本 README 后续命令均默认已执行此三行。

---

## 5. 启动与运行

### 5.1 一键拉起：串口节点 + 视觉（融合主入口）

```bash
cd /path/to/ATS
source install/setup.bash
ros2 launch ats_bringup ats_full.launch.py
```

用途：同时启动 `standard_robot_pp_ros2`、`gimbal_manager` 和 `auto_aim_debug_mpc`。对应文件 `src/ats_bringup/launch/ats_full.launch.py`，其中视觉配置固定取 share 目录下的 `configs/standard3.yaml`。该入口不会启动 Nav2 或行为树；完整实车需使用 `pb2025_sentry_bringup bringup.launch.py`，再单独启动视觉。

### 5.2 完整导航（含 Nav2 + 行为树）

```bash
cd /path/to/ATS
source install/setup.bash
ros2 launch pb2025_sentry_bringup bringup.launch.py world:=rmul slam:=False use_rviz:=True
```

用途：拉起串口驱动、Nav2、行为树、rosbag 记录。对应文件 `src/pb2025_sentry_bringup/launch/bringup.launch.py`。

常用 launch 参数（来自源码声明）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `world` | 空字符串 | 地图/PCD 文件名与该参数同名；为空时路径无效，**实跑必须显式传 `world:=rmul`** |
| `slam` | `False` | `True` 时禁用 small_gicp 并发静态 `map->odom` TF，用于建图 |
| `launch_small_gicp_relocalization` | `False` | `False` 时使用 `map->odom` 零位静态 TF；需要先验 PCD 重定位时设为 `True` |
| `use_rviz` | `False` | 是否启动 RViz |
| `use_sim_time` | `False` | 仿真时钟 |
| `namespace` | 空 | 顶级命名空间；视觉节点继承 ROS launch 命名空间，串口话题需与配置一致 |
| `params_file` | `src/pb2025_sentry_bringup/params/node_params.yaml` | ROS 2 参数文件 |
| `behavior_params_file` | `src/pb2025_sentry_behavior/params/sentry_behavior.yaml` | 行为树参数 |
| `rviz_config_file` | `src/pb2025_sentry_bringup/rviz/sentry_default_view.rviz` | RViz 配置 |
| `use_robot_state_pub` | `False` | 是否启动 robot_state_publisher（True 时串口节点不启动，改发静态 TF） |
| `use_composition` | `True` | 是否组合节点启动 |
| `log_level` | `info` | 日志级别 |

### 5.3 仅导航定位栈（实机入口）

```bash
cd /path/to/ATS
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=rmul slam:=False use_rviz:=True
```

用途：Livox 驱动 + 定位 + Nav2 + 手柄。该入口不会启动串口节点和底盘速度转换；实机单独运行时还需启动 `standard_robot_pp_ros2` 与 `sentry_chassis_vel_transform`。对应文件 `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/rm_navigation_reality_launch.py`。默认参数文件 `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`，地图取 `src/pb2025_sentry_bringup/map/rmul.yaml`，PCD 取 `src/pb2025_sentry_bringup/pcd/rmul.pcd`。

复用原地图且不启动重定位：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:=rmul slam:=False launch_small_gicp_relocalization:=False use_rviz:=True
```

建图模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=rmul slam:=True
```

仿真模式（需配套 rmu_gazebo_simulator）：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py
```

### 5.4 仅串口节点

```bash
cd /path/to/ATS
source install/setup.bash
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
```

用途：仅启动上下位机串口通信。对应文件 `src/standard_robot_pp_ros2/launch/standard_robot_pp_ros2.launch.py`，参数文件 `src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml`。

### 5.5 行为树单独启动

```bash
cd /path/to/ATS
source install/setup.bash
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py
```

对应文件 `src/pb2025_sentry_behavior/launch/pb2025_sentry_behavior_launch.py`。

### 5.6 根目录守护脚本（历史脚本，gnome-terminal 环境可用）

| 脚本 | 命令 | 启动的 launch |
|---|---|---|
| `NAV2.sh` | `./NAV2.sh` | `ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=rmuc slam:=False use_robot_state_pub:=False` |
| `mapping.sh` | `./mapping.sh` | `ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=False` |
| `pp_ros2.sh` | `./pp_ros2.sh` | `ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py` |
| `rosbag.sh` | `./rosbag.sh` | `ros2 launch pb2025_sentry_bringup record_rosbag_launch.py` |

使用前需要执行权限：

```bash
chmod +x NAV2.sh mapping.sh pp_ros2.sh rosbag.sh
./NAV2.sh
```

用途：在独立 gnome-terminal 中启动并周期巡检，进程退出自动重启。**依赖 gnome-terminal 与图形桌面**，远程 SSH 无 GUI 环境请直接用 5.1–5.5 的 `ros2 launch` 命令。

> 注意：`NAV2.sh` 内置 `world:=rmuc`，而仓库当前 bringup 目录只有 `rmul` 地图/PCD；实跑请先改为 `world:=rmul`，或自行补齐 `src/pb2025_sentry_bringup/map/rmuc.yaml`、`pcd/rmuc.pcd`（仿真地图 `rmuc_2024`/`rmuc_2025` 在 `src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/`）。

### 5.7 视觉调试看门狗（WebDebugger）

```bash
cd /path/to/ATS/src/sp_vision25
./debug_mpc_watchdog.sh auto_aim_debug_mpc ../install/sp_vision25/share/sp_vision25/configs/standard3.yaml
```

用途：在新终端启动调试程序并自动重启；若启用 WebDebugger 会等待 `http://127.0.0.1:8090/healthz` 就绪后自动打开浏览器。第二个参数可省略（默认 `standard3.yaml`，本项目唯一配置）。

比赛主程序看门狗（无浏览器逻辑）：

```bash
cd /path/to/ATS/src/sp_vision25
./run_with_watchdog.sh auto_aim_debug_mpc ../install/sp_vision25/share/sp_vision25/configs/standard3.yaml
```

### 5.8 无硬件 loopback 仿真

决策回环：

```bash
cd /path/to/ATS
source install/setup.bash
ros2 launch pb2025_sentry_bringup loopback_decision_sim.launch.py
```

视觉输入注入回环：

```bash
ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py \
  publish_vision_target:=True \
  vision_tracking:=True \
  vision_nav_hold:=True \
  vision_target_type:=1 \
  vision_has_target_position_map:=True \
  vision_target_position_map_frame:=map \
  vision_target_position_map_x:=2.0 \
  vision_target_position_map_y:=1.0
```

Nav2 导航回环（`params_file` 无默认值，必须显式传入）：

```bash
ros2 launch pb2025_sentry_bringup loopback_navigation.launch.py params_file:=src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml
```

用途：不接雷达/串口/相机验证决策树与 Nav2 行为。`loopback_decision_sim.launch.py` 默认地图 `src/pb2025_sentry_bringup/map/rmul.yaml`、行为参数 `src/pb2025_sentry_behavior/params/sentry_behavior.yaml`。

### 5.9 rosbag 记录

```bash
cd /path/to/ATS
source install/setup.bash
ros2 launch pb2025_sentry_bringup record_rosbag_launch.py
```

用途：记录 `/serial/gimbal_joint_state`、`/livox/imu`、`/livox/lidar`，zstd 压缩，时长 30 秒，输出到 `rosbags/sentry_<时间戳>`。对应文件 `src/pb2025_sentry_bringup/launch/record_rosbag_launch.py`。

---

## 6. 配置文件总表

| 用途 | 仓库相对路径 | 说明 |
|---|---|---|
| 视觉主配置（唯一配置） | `src/sp_vision25/configs/standard3.yaml` | `ats_full.launch.py` 传入；含模型路径、相机、标定、WebDebugger、标定参数 |
| 视觉模型文件 | `src/sp_vision25/assets/yolov5.xml`、`tiny_resnet.onnx` | 配置中写相对路径 `assets/...`，运行时由 `src/sp_vision25/tools/path.cpp` 解析 |
| 串口/决策参数 | `src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml` | 串口设备、波特率、决策点位、血量/时间阈值 |
| 共享消息接口 | `src/interfaces/sp_msgs` | `VisionTargetMsg`、`TrajectoryProfileMsg` 等视觉/导航接口 |
| Nav2 实机参数 | `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml` | 实机定位+导航 |
| Nav2 仿真参数 | `src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml` | 仿真 |
| 轨迹优化器参数 | `src/pb2025_sentry_bringup/params/node_params.yaml`、Nav2 `smoother_server` 参数 | B 样条 smoother、trajectory profile、速度 governor |
| 完整 bringup 参数 | `src/pb2025_sentry_bringup/params/node_params.yaml` | bringup.launch.py 默认 |
| Mid360 用户配置 | `src/pb2025_sentry_bringup/params/mid360_user_config.json` | 雷达（另存档一份在 `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/mid360_user_config.json`） |
| 行为树参数 | `src/pb2025_sentry_behavior/params/sentry_behavior.yaml` | 常规 |
| 行为树回环参数 | `src/pb2025_sentry_behavior/params/sentry_behavior_loopback.yaml`、`sentry_behavior_vision_test.yaml` | loopback 用 |
| 地图（bringup 用） | `src/pb2025_sentry_bringup/map/rmul.yaml`、`rmul.pgm` | world:=rmul |
| 先验点云 | `src/pb2025_sentry_bringup/pcd/rmul.pcd` | world:=rmul |
| 实机地图（导航包自带） | `src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/rmul.yaml`、`3x15.yaml` | 实机场地 |
| 仿真地图（导航包自带） | `src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmuc_2024.yaml`、`rmuc_2025.yaml`、`rmul_2024.yaml`、`rmul_2025.yaml` | 仿真场地 |
| RViz 配置 | `src/pb2025_sentry_bringup/rviz/sentry_default_view.rviz`、`src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/nav2_default_view.rviz` | 两个入口各一份 |
| 协议规格书 | `Start/doc/协议规格书.md` | 交给电控的协议规格 |
| udev 规则脚本 | `src/standard_robot_pp_ros2/script/create_udev_rules.sh` | 脚本仍生成 ttyACM0；当前参数默认 ttyACM1，使用前需统一 |

---

## 7. 调试与验证

### 7.1 串口与导航通信验证

```bash
ros2 topic hz /serial/gimbal_joint_state
```

用途：检查电控→串口节点的云台关节状态，消息类型为 `sensor_msgs/msg/JointState`。

```bash
ros2 topic hz /cmd_vel
```

用途：检查导航速度是否已到达串口节点。导航运行时应由 `sentry_chassis_vel_transform` 输出。

```bash
ros2 topic list
```

用途：确认 `/serial/gimbal_joint_state`、`/cmd_vel` 和 `cmd_gimbal_joint` 是否出现。

确认视觉控制和目标输出：

```bash
ros2 topic type /cmd_gimbal
ros2 topic echo /cmd_gimbal_joint --once
ros2 topic echo /vision/target --once
```

视觉云台输出要求 `decision/robot_mode=1`。视觉消息中的 `has_target_position_map` 只有在 `map -> gimbal_pitch` TF 可查询时才为 true。

### 7.2 话题发布手动测试

```bash
ros2 launch pb2025_sentry_bringup gazebo_navigation.launch.py
```

用途：启动 Gazebo Fortress 与导航/行为/视觉闭环。Gazebo GUI 初始暂停，点击开始后检查 `/red_standard_robot1/front_industrial_camera/image`、`/red_standard_robot1/vision/target` 和导航 action。仿真机器人血量与弹量由 `gazebo_referee_adapter.py` 转成行为树消息；比赛阶段仍由仿真输入节点提供。适配器也接收 RFID 状态，但 `rmuc_2025` 场景是否产生该状态需在目标环境确认。只测导航/行为时加 `launch_vision:=False`；该参数只跳过视觉进程，构建 `sp_vision25` 仍需 OpenVINO。

### 7.3 TF 树检查

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

用途：查看整车 TF（base_footprint/base_link/gimbal/chassis 等）。若使用非空 namespace（多机扩展），需要：

```bash
ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=/red_standard_robot1
```

### 7.4 WebDebugger

视觉配置中（`src/sp_vision25/configs/standard3.yaml` 的 `web_debugger` 段）：

```yaml
disable_web: false
web_host: "0.0.0.0"
web_port: 8090
web_fps: 8.0
web_scale: 0.7
web_jpeg_quality: 70
web_client_ttl_ms: 2000
```

启动调试程序后浏览器访问：

```text
http://127.0.0.1:8090/
```

### 7.5 录包回放

```bash
ros2 bag play /path/to/your/rosbag_dir
```

用途：回放 `rosbags/sentry_<时间戳>` 目录（路径替换见 8.9）。

### 7.6 编译期静态检查提示

- HikRobot SDK 未找到时 CMake 会直接报错，并提示设置 `HIKROBOT_SDK_ROOT` 或把 SDK 放到 `sp_vision25/io/hikrobot` / `src/dependencies/hik_camera_ros2_driver/hikSDK`；
- OpenVINO 找不到时检查 `$HOME` 下解压目录版本号是否与 `src/sp_vision25/CMakeLists.txt` 的 `OpenVINO_DIR` 一致（见 8.12）。

---

## 8. 占位符与路径替换规则

本文档所有命令可复制，但含占位符处必须先替换。逐项说明：

### 8.1 `/path/to/ATS`（工作区根目录）

- **替换对象**：本仓库在你 Linux/WSL 机器上的实际克隆目录。
- **替换规则**：替换为不含中文与空格的绝对路径，例如 `~/ATS` 或 `/home/<用户名>/ATS`。
- **示例**：`cd /path/to/ATS` → `cd ~/ATS`。
- **注意事项**：不要替换成 Windows 盘符路径（如 `D:\ATS`）——`colcon`/`ros2` 命令必须在 Linux 文件系统下执行；Windows 侧只作为编辑器工作副本。
- **适配场景**：所有 `cd`、`colcon build`、`ros2 launch` 命令。

### 8.2 `/tmp/small_gicp`（small_gicp 克隆目录）

- **替换对象**：你想克隆并编译 small_gicp 的位置。
- **替换规则**：任意可写临时目录；若已有源码，替换为已有目录后进入其 `build` 目录执行 `cmake`/`make`。
- **注意事项**：安装阶段 `sudo make install` 会写入系统路径，与克隆位置无关。
- **适配场景**：仅第 4.2 节首次安装时使用。

### 8.3 `HIKROBOT_SDK_ROOT=/opt/MVS`

- **替换对象**：HikRobot MVS SDK 实际安装根目录（该目录下应能找到头文件与 `lib`）。
- **替换规则**：按第 3 节的搜索顺序选择实际存在的目录；若使用仓库内置 SDK，可指向 `src/dependencies/hik_camera_ros2_driver/hikSDK`（写绝对路径）。
- **注意事项**：只在环境变量指定的目录找不到时 CMake 才会继续探测固定路径；`export` 只对当前终端有效。
- **适配场景**：第 4.3、4.5 节编译命令。

### 8.4 `--packages-select <包名>`

- **替换对象**：要重编的 colcon 包名。
- **可选值与对应目录**：
  - `sp_vision25` → `src/sp_vision25`
  - `standard_robot_pp_ros2` → `src/standard_robot_pp_ros2`
  - `pb2025_sentry_bringup` → `src/pb2025_sentry_bringup`
  - `pb2025_sentry_behavior` → `src/pb2025_sentry_behavior`
  - `pb2025_nav_bringup` → `src/pb2025_sentry_nav/pb2025_nav_bringup`
  - `trajectory_optimizer` → `src/pb2025_sentry_nav/trajectory_optimizer`
  - `sentry_chassis_vel_transform` → `src/sentry_chassis_vel_transform`
  - `sp_msgs` → `src/interfaces/sp_msgs`
  - `pb_rm_interfaces` → `src/interfaces/pb_rm_interfaces`
- **替换规则**：填包名（`package.xml` 中的 name 字段），不是目录名；可空格分隔多个包。
- **适配场景**：第 4.6 节增量编译。

### 8.5 `world:=rmul`

- **替换对象**：场地名，同时决定地图与 PCD 文件名。
- **对应文件**：
  - 地图：`src/pb2025_sentry_bringup/map/<world>.yaml` 与 `.pgm`（现有 `rmul`）
  - PCD：`src/pb2025_sentry_bringup/pcd/<world>.pcd`（现有 `rmul.pcd`）
  - 导航包自带实机地图：`src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/rmul.yaml`、`3x15.yaml`；仿真地图：`.../map/simulation/rmuc_2024.yaml`、`rmuc_2025.yaml`、`rmul_2024.yaml`、`rmul_2025.yaml`
- **替换规则**：`<world>` 必须与上述文件主名完全一致（区分大小写）。
- **注意事项**：`NAV2.sh` 默认 `world:=rmuc`，仓库当前 bringup 目录只有 `rmul`，直接运行会找不到地图/PCD。
- **适配场景**：5.2、5.3、5.6 节命令。

### 8.6 `../install/sp_vision25/share/sp_vision25/configs/standard3.yaml`（看门狗第二个参数）

- **替换对象**：视觉程序要加载的配置文件。
- **可选值**：本项目仅保留 `configs/standard3.yaml` 一个配置，无需替换。
- **替换规则**：路径相对脚本所在目录 `src/sp_vision25` 书写，也可直接写绝对路径。
- **注意事项**：`--symlink-install` 下 share 内文件与源码同步；未带 `--symlink-install` 编译时，改源码 yaml 后需重新 `colcon build` 该包才会进入 install。
- **适配场景**：5.7 节看门狗命令。

### 8.7 `debug_mpc_watchdog.sh auto_aim_debug_mpc` / `run_with_watchdog.sh auto_aim_debug_mpc`（第一个参数）

- **替换对象**：要启动的可执行程序名。
- **可选值**（均在 `install/sp_vision25/lib/sp_vision25/` 下）：
  - 主入口：`auto_aim_debug_mpc`（自瞄 + MPC + 话题桥接版，唯一生产入口）
  - 标定工具：`capture`、`calibrate_camera`、`calibrate_handeye`、`calibrate_robotworld_handeye`、`split_video`
- **替换规则**：填可执行文件名，不含路径。
- **注意事项**：两个看门狗脚本默认均为 `auto_aim_debug_mpc` + `standard3.yaml`，通常无需传参。
- **适配场景**：5.7 节。

### 8.8 `params_file:=src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`

- **替换对象**：loopback_navigation 显式传入的 Nav2 参数文件（该 launch 无默认值，**必须提供**）。
- **替换规则**：替换为真实存在的参数文件路径；可用值见第 6 节总表。
- **注意事项**：5.8 示例中是源码相对路径，工作目录必须是 ATS 根目录；也可换成 `install/pb2025_nav_bringup/share/pb2025_nav_bringup/config/reality/nav2_params.yaml`。
- **适配场景**：5.8 节 Nav2 回环。

### 8.9 `ros2 bag play /path/to/your/rosbag_dir`

- **替换对象**：录制的 rosbag 目录，形如 `rosbags/sentry_2026-09-20_12-00-00`。
- **替换规则**：填录制输出的目录路径（`ros2 bag record -o` 指定的目录），不是单个文件。
- **适配场景**：7.5 节回放。

### 8.10 namespace 占位（多机扩展）

- **替换对象**：`ros2 launch ... namespace:=<ns>` 中的 `<ns>`，如 `red_standard_robot1`。
- **替换规则**：仅多机器人部署需要。
- **注意事项（重要）**：导航速度 `/cmd_vel` 是绝对话题；视觉和云台管理器话题为相对名，并由 ROS namespace 一起解析。单机默认空 namespace；Gazebo 入口将视觉、云台、导航和行为节点放在同一机器人 namespace。多机实车仍需核对绝对速度话题和串口节点 namespace。
- **适配场景**：5.2、5.3、7.3。

### 8.11 WebDebugger 地址中的主机与端口

- **替换对象**：`http://127.0.0.1:8090/` 中的 `127.0.0.1` 与 `8090`。
- **对应文件**：`src/sp_vision25/configs/standard3.yaml` 的 `web_host` / `web_port` 字段。
- **替换规则**：
  - 本机访问：保持 `127.0.0.1`；
  - 远程/WSL 访问实机服务：替换为机器人主机 IP；WSL2 访问 Windows 浏览器可用 WSL 的 `localhost` 转发（默认 `web_host: 0.0.0.0` 已允许外部访问）；
  - 端口冲突时改 `web_port` 后同步改 URL。
- **适配场景**：7.4 节。

### 8.12 OpenVINO 安装路径

- **设置方式**：把 `OpenVINO_DIR` 指向包含 `OpenVINOConfig.cmake` 的 `runtime/cmake` 目录；Release 脚本会检查实验室默认目录，也接受环境变量覆盖。
- **示例**：`OpenVINO_DIR="$HOME/intel/openvino_2025/runtime/cmake" bash ./build_release.sh`
- **注意事项**：更换 OpenVINO 版本或安装目录后，CMake 会通过 `-DOpenVINO_DIR` 刷新构建配置；版本仍需与项目模型运行环境匹配。
- **适配场景**：OpenVINO `find_package` 失败时。

### 8.13 串口设备名与波特率

- **替换对象**：`src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml` 中的 `device_name` 与 `baud_rate`（当前快照默认 `/dev/ttyACM1`、`115200`）。
- **替换规则**：运行 `ls /dev/ttyACM*` 确认实际设备；多串口设备时依赖 udev 规则固定别名。
- **注意事项**：波特率必须与电控固件一致；若需要 200Hz 导航+自瞄数据，应由电控确认更高波特率是否可用。`debug: false` 必须保持，防止 160B debug 帧超过下位机 USB CDC 64B 单帧限制。
- **适配场景**：串口打不开/数据错乱时排查。

---

## 9. 常见问题与回滚

### 9.1 桥接无数据

1. 确认两侧话题存在：`ros2 topic list | grep serial`
2. 检查串口状态：`ros2 topic hz /serial/gimbal_joint_state`
3. 检查导航速度：`ros2 topic hz /cmd_vel`
4. 检查串口：按参数文件中的设备名执行 `ls -l /dev/ttyACM*`，并确认 `groups` 含 `dialout`

如果排查视觉云台桥接，检查 `decision/robot_mode`、`cmd_gimbal`、`cmd_gimbal_joint` 和 `gimbal_manager`。视觉坐标未转为地图坐标时，检查 `map -> gimbal_pitch` TF。

### 9.2 关闭视觉控制

停止 `sp_vision25` 进程即可关闭视觉输出；视觉桥接不会直接打开串口。需要运行导航但不启动视觉时，使用 `pb2025_sentry_bringup bringup.launch.py`，不要启动 `ats_full.launch.py`。

### 9.3 回滚：导航速度链排查

逐层检查 `cmd_vel_controller_governed`、`cmd_vel_nav2_result`、`cmd_vel_gimbal_yaw_odom` 和 `/cmd_vel`。

### 9.4 频率不足/丢帧

- 确认电控波特率与参数文件一致（需电控确认，见 8.13）；
- WSL2 下 USB 透传稳定性有限，实机优先直连；
- 编译用 `Release`（默认已带）。

---

## 10. 延伸文档索引

| 文档 | 路径 |
|---|---|
| 串口融合决策记录 | `Start/README.md` |
| 协议规格书（给电控） | `Start/doc/协议规格书.md` |
| 导航侧桥接补丁说明 | `Start/nav_bridge/README.md` |
| 视觉侧桥接说明与 mode 映射 | `Start/vision_bridge/README.md` |
| 视觉包说明 | `src/sp_vision25/README.md`（精简后自瞄链路说明） |
| 导航迁移分析 | `docs/navigation_migration_analysis.md` |
| 原地图选点导航 | `docs/复用原地图选点导航.md` |
| 视觉链路测试 | `docs/视觉链路测试.md` |
| 串口节点原始 README | `src/standard_robot_pp_ros2/README.md` |
| 导航原始 README | `src/pb2025_sentry_nav/README.md`、`src/pb2025_sentry_nav/README_en.md` |
| 机器人描述 README | `src/pb2025_robot_description/de_README.md` |
| 原 4 份 README 存档 | `docs/ws_README.md`、`docs/nav_README.md`、`docs/de_README.md` |
| 融合过程记录 | `docs/融合.md`、`ATS_融合方案.md` |
| 通信协议梳理 | `通信协议梳理报告.md` |
| 移植说明 | `docs/移植.md` |
| 视觉跟随仿真调试 | `docs/视觉跟随仿真调试.md` |
| loopback 重构记录 | `docs/slim_loopback_refactor.md` |
| 行为树决策检查清单 | `docs/sentry_bt_decision_checklist.md` |
| navigate_through_poses 迁移清单 | `docs/navigate_through_poses_migration_checklist.md` |
| 代码助手工作流 | `docs/codex_workflow.md`、`docs/codex_usage_tutorial.md`、`AGENTS.md` |

---

## 附录：快速命令速查

```bash
# 全工作区 Release 编译
cd ~/ATS && bash ./build_release.sh

# 环境加载
source install/setup.bash

# 融合主入口
ros2 launch ats_bringup ats_full.launch.py

# 完整导航
ros2 launch pb2025_sentry_bringup bringup.launch.py world:=rmul slam:=False use_rviz:=True

# 导航速度链验证
ros2 topic hz /serial/gimbal_joint_state
ros2 topic hz /cmd_vel
```
