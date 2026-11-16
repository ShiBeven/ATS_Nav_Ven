# sp_vision25 — 哨兵视觉包（精简版）

> 最后更新：2026-09-20（精简改造：仅保留自瞄链路，删除 buff / omniperception / 多余入口）。
> 本包当前唯一生产入口为 `auto_aim_debug_mpc`，唯一配置文件为 `configs/standard3.yaml`。

## 1. 包定位

RoboMaster 哨兵机器人视觉包，运行在 `ats_full.launch.py` 融合入口下：

- **自瞄**：YOLOv5 检测 + 传统方法 + EKF 跟踪 + PnP 解算 + MPC（TinyMPC）规划
- **云台桥接**：实车读取 `/serial/imu`、`/serial/gimbal_joint_state`；Gazebo 读取 TF、`/joint_states`。控制统一发布 `cmd_gimbal`、`cmd_shoot`。
- **目标发布**：主循环持续发布 `sp_msgs/msg/VisionTargetMsg` 到 `vision/target`，并通过 TF 将目标位置转换到 `map`。
- **控制仲裁**：当前配置 `require_attack_mode: false`，有效 MPC 结果可直接发布云台命令和射击请求，不依赖行为树模式；改为 `true` 恢复原有 Attack 门控。缺失 map TF 时 `nav_hold=false`。修改范围与恢复方法见 [视觉独立控制与恢复说明](docs/视觉独立控制与恢复说明.md)。

## 2. 代码结构（精简后）

```text
sp_vision25/
├── src/
│   └── auto_aim_debug_mpc.cpp   # 唯一生产入口（自瞄 + MPC + 话题桥接）
├── tasks/
│   └── auto_aim/                # 自瞄算法库
│       ├── armor / classifier / detector / solver / aimer / target
│       ├── tracker / voter / shooter / yolo
│       ├── yolos/yolov5.cpp     # 唯一 YOLO 后端
│       └── planner/             # MPC 规划器（含 tinympc）
├── io/
│   ├── camera.cpp / hikrobot/   # 相机抽象 + HikRobot 驱动
│   ├── gimbal/                  # 串口 Gimbal 实现（结构体 GimbalToVision/VisionToGimbal 在此，勿动）
│   ├── ros2/                    # gimbal_topic_bridge（核心）+ publish2nav/subscribe2nav（备用）
│   └── serial/                  # 串口库（vendored）
├── tools/                       # 通用工具（EKF、弹道、日志、Web 调试器等）
├── calibration/                 # 标定工具源码（capture / calibrate_camera / calibrate_handeye / ...）
├── configs/
│   └── standard3.yaml           # 唯一配置（含标定参数，勿删）
├── assets/
│   ├── yolov5.xml + yolov5.bin  # 装甲板检测模型
│   ├── tiny_resnet.onnx         # 数字分类模型
│   └── web_debugger/            # Web 调试器前端
├── tests/                       # 自瞄链路回归测试
└── docs/                        # 自瞄/标定/MPC 调参文档
```

## 3. 运行

```bash
# 编译
source /opt/ros/humble/setup.bash
export HIKROBOT_SDK_ROOT=/opt/MVS
colcon build --packages-select sp_vision25 --cmake-args -DCMAKE_BUILD_TYPE=Release

# 一键启动（导航串口 + 视觉）
source install/setup.bash
ros2 launch ats_bringup ats_full.launch.py
```

Gazebo-only 工作站没有 HikRobot MVS SDK 时，构建参数加 `-DSP_VISION_ENABLE_HIKROBOT=OFF`；运行视觉算法仍需 OpenVINO。

### 3.1 桥接验证

```bash
ros2 topic echo /cmd_gimbal --once
ros2 topic echo /cmd_gimbal_joint --once
ros2 topic echo /cmd_shoot --once
ros2 topic echo /vision/target --once
```

话题类型分别为 `pb_rm_interfaces/msg/GimbalCmd`、`sensor_msgs/msg/JointState`、`example_interfaces/msg/UInt8` 和 `sp_msgs/msg/VisionTargetMsg`。完整验收步骤见 [视觉链路测试](../../docs/视觉链路测试.md)。

### 3.2 Gazebo 仿真

```bash
ros2 launch pb2025_sentry_bringup gazebo_navigation.launch.py
```

该入口启动 Gazebo Fortress、Nav2、行为树、云台管理器和 Gazebo 相机输入的视觉节点。Gazebo 初始暂停时，在 GUI 中开始仿真。没有 OpenVINO 运行环境时可用 `launch_vision:=False` 只运行导航与决策；构建工作区本身仍需 OpenVINO。

### 3.3 Web 调试器

启动后浏览器访问 `http://127.0.0.1:8090/`（参数在 `configs/standard3.yaml` 的 `web_debugger` 段）。

## 4. 通信结构（不可改动）

```text
GimbalToVision (packed, ≤64B):
  head(1) mode(1) q[4](16) yaw(4) yaw_vel(4) pitch(4) pitch_vel(4)
  bullet_speed(4) bullet_count(2) crc16(2)

VisionToGimbal (packed, ≤64B):
  head(1) mode(1) yaw(4) yaw_vel(4) yaw_acc(4) pitch(4) pitch_vel(4) pitch_acc(4) crc16(2)
```

定义位于 `io/gimbal/gimbal.hpp`，带 `static_assert(sizeof(...) <= 64)` 编译期断言。
**任何改动都会破坏与下位机/导航串口节点的协议兼容，禁止修改。**

## 5. 标定流程

完整流程见 `docs/标定流程与参数回填清单.md`。要点：

1. `capture` 采样（默认读 `configs/standard3.yaml` 的相机参数）
2. `calibrate_camera` / `calibrate_handeye` / `calibrate_robotworld_handeye` 解算
3. 结果回填到 `configs/standard3.yaml` 的 `camera_matrix / distort_coeffs /
   R_camera2gimbal / t_camera2gimbal` 字段

标定棋盘格参数（`pattern_cols / pattern_rows / center_distance_mm`）已合并进
`standard3.yaml` 的「标定参数」段。

## 6. 历史说明

- 打符（auto_buff）与全向感知（omniperception）链路已于 2026-09-20 精简删除；
  原 buff 模块工程文档已随模块一并移除，历史记录见仓库外部备份。
- MindVision 相机支持已删除，仅保留 HikRobot。
- `left_yaw_offset / right_yaw_offset` 为可选键（`IsDefined` 保护），缺省回退到
  `yaw_offset`。
