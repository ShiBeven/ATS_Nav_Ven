# ATS 一体化工作区使用指南

> 最后核对日期：2026-09-20。本 README 以仓库当前目录结构为准逐条核对，可直接复制执行。
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
| 串口通信 | `src/standard_robot_pp_ros2` | **独占 `/dev/ttyACM0`**，与下位机交换 13 种收包 + 1 种发包；内置云台桥接 |
| 自瞄 | `src/sp_vision25` | OpenVINO 推理 + EKF 跟踪 + PnP 解算 + 弹道预测 |
| 导航与定位 | `src/pb2025_sentry_nav`（含 `sp_msgs`） | Livox Mid360 + point_lio 里程计 + small_gicp 重定位 + Nav2 全局/局部规划 |
| 行为决策 | `src/pb2025_sentry_behavior` | 行为树，根据血量/时间/视觉目标选择巡逻、补给、跟踪等目标点 |
| 统一入口 | `src/ats_bringup` | `ats_full.launch.py` 一键拉起串口节点 + 视觉主程序 |
| 完整导航入口 | `src/pb2025_sentry_bringup` | `bringup.launch.py` 拉起串口 + Nav2 + 行为树 + rosbag 记录 |

### 1.2 融合后的数据流（核心架构）

```text
[下位机电控]
   ↕ 唯一 USB 串口（建议 921600, 8N1，帧格式 HeaderFrame+time_stamp+data+CRC16）
[导航串口节点 standard_robot_pp_ros2]   ← 13 收包 + 1 发包协议保持不变
   ├─ 发布 /serial/gimbal_state   （ID_GIMBAL_STATE 0x0E，约 200Hz）
   └─ 订阅 /serial/gimbal_cmd     （组 ID_GIMBAL_CMD 0x02 发出）
   ↕ ROS 2 (DDS)
[视觉主程序 auto_aim_debug_mpc（话题桥接版）]
   ├─ 订阅 /serial/gimbal_state   → 注入 io::Gimbal（时间队列 slerp / 状态）
   └─ 发布 /serial/gimbal_cmd     → 原 gimbal.send(...) 全部改为发话题
```

要点：
- **视觉不再自行打开串口**，由 `standard_robot_pp_ros2` 统一接管串口；
- 仲裁规则为**导航优先**：mode 4=导航接管（NAV_OVERRIDE）、5=冲突（CONFLICT）时视觉桥接层直接丢弃输出；
- 话题名是相对名 `/serial/gimbal_state`、`/serial/gimbal_cmd`，**部署时导航节点必须以空 namespace 启动**，否则两侧无法互通。

### 1.3 适用场景

- RoboMaster 哨兵机器人实车部署（HikRobot 相机 + Livox Mid360 + C 板串口）；
- WSL2 / Ubuntu 22.04 下的仿真、loopback 无硬件回归调试；
- 视觉算法离线调试（WebDebugger 浏览器调参、录包回放）。

---

## 2. 目录结构与包职责

```text
ATS/
├── README.md                        # 本使用指南
├── AGENTS.md                        # 代码助手工作区约定（搜索策略/验证策略）
├── NAV2.sh                          # 实机导航守护脚本（自动重启 rm_navigation_reality_launch.py）
├── mapping.sh                       # 建图守护脚本（slam:=True）
├── pp_ros2.sh                       # 仅串口节点守护脚本
├── rosbag.sh                        # rosbag 记录守护脚本
├── ATS_融合方案.md                   # 融合方案总览
├── 通信协议梳理报告.md                # 通信协议梳理报告
├── src/
│   ├── ats_bringup/                 # 统一启动入口（ats_full.launch.py）
│   ├── sp_vision25/                 # 视觉包（自瞄/打符/全向感知，含话题桥接）
│   ├── standard_robot_pp_ros2/      # 导航串口节点（独占 /dev/ttyACM0，含视觉桥接）
│   ├── pb2025_sentry_nav/           # 导航功能包集合（含 sp_msgs、pb2025_nav_bringup 等）
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
├── pb_omni_pid_pursuit_controller/  # 路径跟踪控制器
├── pb_teleop_twist_joy/             # 手柄控制
├── pointcloud_to_laserscan/         # terrain_map → LaserScan（仅 SLAM 模式）
├── point_lio/                       # 里程计
├── sensor_scan_generation/          # 点云坐标系变换
├── small_gicp_relocalization/       # 重定位
├── terrain_analysis/                # 车体 4m 内地形分析
├── terrain_analysis_ext/            # 车体 4m 外地形分析
└── sp_msgs/                         # 视觉相关消息（视觉包依赖）
```

---

## 3. 运行环境与依赖要求

| 依赖 | 版本/路径要求 | 说明 |
|---|---|---|
| 操作系统 | Ubuntu 22.04 或 WSL2 | 命令均为 Bash |
| ROS 2 | Humble | `source /opt/ros/humble/setup.bash` |
| HikRobot SDK | `HIKROBOT_SDK_ROOT` 指向 SDK 根目录 | CMake 搜索顺序见下 |
| OpenVINO | 2025.4 | CMake 中写死为 `$HOME/openvino_toolkit_ubuntu22_2025.4.1.20426.82bbf0292c5_x86_64` |
| OpenCV / fmt / Eigen3 / spdlog / yaml-cpp / nlohmann_json | 系统安装 | `find_package` 必需 |
| small_gicp | 源码安装 | 重定位依赖 |
| 硬件 | `/dev/ttyACM0`（C 板）、HikRobot 相机、Livox Mid360 | 仅实机需要 |

**HikRobot SDK 搜索路径**（见 `src/sp_vision25/io/CMakeLists.txt`，按顺序探测）：

1. 环境变量 `$HIKROBOT_SDK_ROOT`
2. `/opt/MVS`
3. `/usr/local/MVS`
4. `/opt/hikrobot/MVS`
5. `/opt/HikRobot/MVS`
6. `src/dependencies/hik_camera_ros2_driver/hikSDK`
7. `src/sp_vision25/io/hikrobot`

**OpenVINO 路径**（见 `src/sp_vision25/CMakeLists.txt` 第 17 行）：

```text
$HOME/openvino_toolkit_ubuntu22_2025.4.1.20426.82bbf0292c5_x86_64/runtime/cmake
```

若你的 OpenVINO 压缩包版本号不同，需要同步修改 `src/sp_vision25/CMakeLists.txt` 中的 `OpenVINO_DIR`（见[第 8 节](#8-占位符与路径替换规则)）。

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

用途：把 RoboMaster C 板 USB 串口固定映射为 `/dev/ttyACM0` 并加入 `dialout` 组。执行后**重新插拔串口或注销重登**使组权限生效。

> ⚠️ `/path/to/ATS` 为占位符，见[第 8 节](#8-占位符与路径替换规则)。

### 4.5 全量编译（统一 Release）

```bash
source /opt/ros/humble/setup.bash
export HIKROBOT_SDK_ROOT=/opt/MVS
cd /path/to/ATS
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

用途：全工作区编译。`--parallel-workers 1` 为 WSL 内存紧张时的保守设置；实机可去掉该参数或调大。

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

用途：同时启动 `standard_robot_pp_ros2`（串口 + 桥接）与 `auto_aim_debug_mpc`（视觉主程序，话题桥接版）。对应文件 `src/ats_bringup/launch/ats_full.launch.py`，其中视觉配置固定取 share 目录下的 `configs/standard3.yaml`。

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
| `use_rviz` | `False` | 是否启动 RViz |
| `use_sim_time` | `False` | 仿真时钟 |
| `namespace` | 空 | 顶级命名空间；**视觉桥接要求保持空**（见 8.10） |
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

用途：Livox 驱动 + 定位 + Nav2 + 手柄。对应文件 `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/rm_navigation_reality_launch.py`。默认参数文件 `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`，地图取 `src/pb2025_sentry_bringup/map/rmul.yaml`，PCD 取 `src/pb2025_sentry_bringup/pcd/rmul.pcd`。

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
ros2 launch pb2025_sentry_bringup loopback_vision_test.launch.py
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
| Nav2 实机参数 | `src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml` | 实机定位+导航 |
| Nav2 仿真参数 | `src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml` | 仿真 |
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
| udev 规则脚本 | `src/standard_robot_pp_ros2/script/create_udev_rules.sh` | C 板串口固定为 ttyACM0 |

---

## 7. 调试与验证

### 7.1 桥接通信验证（融合后必测）

```bash
ros2 topic hz /serial/gimbal_state
```

用途：电控→视觉状态话题频率，**应 ≥200Hz**。

```bash
ros2 topic echo /serial/gimbal_cmd
```

用途：查看视觉决策帧（`std_msgs/msg/Float64MultiArray`，固定 7 元素：`mode yaw yaw_vel yaw_acc pitch pitch_vel pitch_acc`）。

```bash
ros2 topic list
```

用途：确认 `/serial/gimbal_state`、`/serial/gimbal_cmd` 已出现。

### 7.2 话题发布手动测试

```bash
ros2 topic pub /serial/gimbal_state std_msgs/msg/Float64MultiArray "{data: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0, 0.0]}" -r 200
```

用途：无下位机时模拟 200Hz 云台状态注入视觉程序。话题类型为 `std_msgs/msg/Float64MultiArray`（非自定义消息），数组固定 **11 元素**，顺序如下（见 `Start/vision_bridge/gimbal_topic_bridge.hpp` 第 38 行）：

```text
gimbal_state[11] = mode q_w q_x q_y q_z yaw yaw_vel pitch pitch_vel bullet_speed bullet_count
```

上例含义：mode=0（空闲）、单位四元数（q_w=1）、角度与角速度全 0、弹速 15、余弹 0。

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
- **注意事项（重要）**：视觉桥接话题是相对名 `/serial/gimbal_state`、`/serial/gimbal_cmd`。一旦导航节点以非空 namespace 启动，话题变为 `<ns>/serial/...`，而视觉节点无 namespace，**两侧将无法互通**。使用 `ats_full.launch.py` 时必须保持空 namespace，或为视觉节点加相同 namespace（需要自行改 launch）。
- **适配场景**：5.2、5.3、7.3。

### 8.11 WebDebugger 地址中的主机与端口

- **替换对象**：`http://127.0.0.1:8090/` 中的 `127.0.0.1` 与 `8090`。
- **对应文件**：`src/sp_vision25/configs/standard3.yaml` 的 `web_host` / `web_port` 字段。
- **替换规则**：
  - 本机访问：保持 `127.0.0.1`；
  - 远程/WSL 访问实机服务：替换为机器人主机 IP；WSL2 访问 Windows 浏览器可用 WSL 的 `localhost` 转发（默认 `web_host: 0.0.0.0` 已允许外部访问）；
  - 端口冲突时改 `web_port` 后同步改 URL。
- **适配场景**：7.4 节。

### 8.12 OpenVINO 版本号（修改源码时）

- **替换对象**：`src/sp_vision25/CMakeLists.txt` 中的：
  ```cmake
  set(OpenVINO_DIR "$ENV{HOME}/openvino_toolkit_ubuntu22_2025.4.1.20426.82bbf0292c5_x86_64/runtime/cmake")
  ```
- **替换规则**：版本号必须与 `$HOME` 下实际解压目录名**逐字符一致**。
- **注意事项**：修改后需删除 `build/sp_vision25`（或整仓 `build/`）再重新编译，CMake 缓存不会自动刷新该路径。
- **适配场景**：OpenVINO `find_package` 失败时。

### 8.13 串口设备名与波特率

- **替换对象**：`src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml` 中的 `device_name: /dev/ttyACM0` 与 `baud_rate: 921600`。
- **替换规则**：运行 `ls /dev/ttyACM*` 确认实际设备；多串口设备时依赖 udev 规则固定别名。
- **注意事项**：融合方案建议波特率 921600（115200 不足以承载 200Hz 导航+自瞄数据），**需电控确认**；`debug: false` 必须保持，防止 160B debug 帧超过下位机 USB CDC 64B 单帧限制。
- **适配场景**：串口打不开/数据错乱时排查。

---

## 9. 常见问题与回滚

### 9.1 桥接无数据

1. 确认两侧话题存在：`ros2 topic list | grep serial`
2. 检查频率：`ros2 topic hz /serial/gimbal_state`
3. 检查 namespace 是否为空（见 8.10）
4. 检查串口：`ls -l /dev/ttyACM0`、`groups`（应含 `dialout`）

### 9.2 回滚：视觉恢复直连串口

编辑 `src/sp_vision25/src/auto_aim_debug_mpc.cpp`，把构造行改回：

```cpp
io::Gimbal gimbal(config_path);
```

（含对应 include），然后重编 `sp_vision25`。详见 `Start/vision_bridge/auto_aim_debug_mpc.diff` 与 `Start/vision_bridge/README.md`。

### 9.3 回滚：导航去掉桥接

删除 `src/standard_robot_pp_ros2/include/standard_robot_pp_ros2/standard_robot_pp_ros2.hpp` 与 `src/standard_robot_pp_ros2/src/standard_robot_pp_ros2.cpp` 中 `gimbal_bridge_` 相关行，重编 `standard_robot_pp_ros2`。

### 9.4 频率不足/丢帧

- 确认电控波特率是否已升到 921600（需电控确认，见 8.13）；
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
# 编译
source /opt/ros/humble/setup.bash && export HIKROBOT_SDK_ROOT=/opt/MVS
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1

# 环境加载
source install/setup.bash

# 融合主入口
ros2 launch ats_bringup ats_full.launch.py

# 完整导航
ros2 launch pb2025_sentry_bringup bringup.launch.py world:=rmul slam:=False use_rviz:=True

# 桥接验证
ros2 topic hz /serial/gimbal_state
ros2 topic echo /serial/gimbal_cmd
```
