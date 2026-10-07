# sp_vision25 精简改造设计方案（仅视觉侧，不动通信）

> 目标：仅保留 `auto_aim_debug_mpc` 自瞄链路，删除 buff / omniperception / 其他入口与无用资源。
> **硬性约束（用户明确要求）**：不改动任何通信相关代码，`GimbalToVision`、`VisionToGimbal` 结构体的**字节数、字段、顺序、CRC 逻辑、话题格式（11 元素 state / 7 元素 cmd）完全不变**，不影响电控与导航串口节点。

---

## 一、现状盘点（实测数据）

### 1.1 体量
| 项目 | 大小 |
|---|---|
| `src/sp_vision25` 总计 | **186.9 MB** |
| `assets/demo/`（打符录像） | 111.6 MB |
| `io/mindvision/`（MindVision SDK，未使用） | 27.4 MB |
| buff/多余模型文件 | 29.0 MB |
| **可削减合计** | **≈168.0 MB** |
| **精简后预计** | **≈18.9 MB** |

### 1.2 入口程序（`src/sp_vision25/src/`，13 个）
| 程序 | 链路 | 处置 |
|---|---|---|
| `auto_aim_debug_mpc.cpp` | **自瞄+MPC（唯一保留）** | ✅ 保留 |
| `auto_debug.cpp` | 自瞄调试（无 MPC） | ❌ 删 |
| `auto_buff_debug.cpp` | 打符 | ❌ 删 |
| `auto_buff_debug_mpc.cpp` | 打符+MPC | ❌ 删 |
| `mt_standard.cpp` | 多线程标准机 | ❌ 删 |
| `sentry.cpp` / `sentry_bp.cpp` / `sentry_debug.cpp` / `sentry_multithread.cpp` | 全向感知哨兵（omniperception） | ❌ 删 |
| `standard.cpp` / `standard_mpc.cpp` | 标准机直连串口 | ❌ 删 |
| `uav.cpp` / `uav_debug.cpp` | 无人机 | ❌ 删 |

### 1.3 任务库（`tasks/`）
| 目录 | 大小 | 处置 |
|---|---|---|
| `tasks/auto_aim/` | 262 KB / 47 文件 | ✅ 保留（含 planner/tinympc） |
| `tasks/auto_buff/` | 155 KB / 14 文件 | ❌ 删（Ceres 依赖一并解除） |
| `tasks/omniperception/` | 22 KB / 6 文件 | ❌ 删（但注意 tracker.hpp 有一处耦合，见 4.2） |

### 1.4 io 子系统
| 目录 | 大小 | 处置 | 理由 |
|---|---|---|---|
| `io/gimbal/` | 18 KB | ⚠️ 保留但**零改动** | 定义通信结构体 `GimbalToVision`/`VisionToGimbal`/`GimbalMode`/`GimbalState`，话题桥接 `#include "io/gimbal/gimbal.hpp"` 依赖它 |
| `io/ros2/gimbal_topic_bridge.*` | — | ✅ 保留零改动 | 融合核心，话题收发 |
| `io/ros2/publish2nav.*` `subscribe2nav.*` `ros2.*` | — | ❌ 删（CMake 中一并移除） | 只有 `sentry*.cpp` 使用；`vision/target` 话题由 loopback 仿真或后续视觉融合节点提供，自瞄主链路不依赖 |
| `io/hikrobot/` | 13.7 MB | ✅ 保留 | 相机驱动（`camera_name: hikrobot`） |
| `io/mindvision/` | 27.4 MB | ❌ 删 | `camera.cpp` 中已有 `#ifdef` 保护，删除后 CMake 检测不到 MindVision SDK 会自动禁用该分支，**需同步删除 `camera.cpp` 中 mindvision 分支与 `#ifdef`**（见 4.4） |
| `io/usbcamera/` | 6 KB | ❌ 删 | 仅 `sentry*.cpp` 使用 |
| `io/serial/` | 102 KB | ✅ 保留零改动 | `gimbal.hpp` include 了 `serial/serial.h`；串口结构体定义属于通信侧 |
| `io/cboard.cpp/.hpp` | — | ❌ 删（源文件），但 `ShootMode` 枚举需要迁移（见 4.3） | CAN 通信已废弃，但 `aimer.hpp` include 它取 `io::ShootMode` |
| `io/dm_imu/` | 7 KB | ❌ 删 | 仅 `mt_standard`/`uav*` 使用 |
| `io/socketcan.hpp` | — | ❌ 删 | 仅 `cboard` 使用 |

### 1.5 tools 子系统（16 个 .cpp，保留 15 删 2）
| 模块 | 处置 | 理由 |
|---|---|---|
| `ransac_sine_fitter` | ❌ 删 | 仅 `buff_target.hpp` 使用 |
| `pid` | ❌ 删 | 全仓库无引用 |
| `thread_pool` | ⚠️ 保留但可后续移除 | 仅 `perceptron.hpp` include；omniperception 删除后无实际使用者（见 4.2） |
| 其余 14 个（debug/ekf/img_tools/logger/math/path/plotter/recorder/runtime_params/thread_safe_queue/trajectory/web_debugger/yaml/crc/debug_visualization） | ✅ 保留 | 保留链路的依赖闭包 |

### 1.6 资产文件（`assets/`）
| 文件 | 大小 | 处置 | 理由 |
|---|---|---|---|
| `demo/`（符.avi 等 5 个） | 111.6 MB | ❌ 删 | 打符调试录像 |
| `yolov5.xml` + `yolov5.bin` | 3.1 MB | ✅ 保留 | `sentry.yaml` 的 `yolo_name: yolov5` |
| `tiny_resnet.onnx` | 1.1 MB | ✅ 保留 | `classifier.cpp` 数字识别 |
| `yolo11.xml/.bin` | 3.6 MB | ❌ 删 | 未被选中（可留作可选，见 8.1） |
| `yolov8.xml/.bin` | 4.3 MB | ❌ 删 | 未被选中 |
| `yolo11_buff_int8.xml/.bin` | 3.5 MB | ❌ 删 | 打符模型 |
| `Rune-v8n-*.xml/.bin` | 6.0 MB | ❌ 删 | 打符模型 |
| `best2-sim.onnx` | 12.3 MB | ❌ 删 | 无任何引用 |
| `standard_fanblade.jpg` | 44 KB | ❌ 删 | 仅 demo.yaml/mvs.yaml（buff 相关配置）引用 |
| `web_debugger/` | 0.1 MB | ✅ 保留 | WebDebugger 前端 |

### 1.7 配置文件（`configs/`）
保留：`sentry.yaml`（主配置）
删除：`standard3.yaml` `standard4.yaml` `ascento.yaml` `uav.yaml` `demo.yaml` `example.yaml`
（`camera.yaml` `calibration.yaml` `handeye.yaml` `mvs.yaml` 为标定辅助，可保留或归档，默认保留——体积仅 KB 级）

### 1.8 测试程序（`tests/`，24 个）
| 测试 | 处置 | 理由 |
|---|---|---|
| `auto_aim_test.cpp` `planner_test*.cpp` `detector_video_test.cpp` `gimbal_response_test.cpp` `fire_test.cpp` `gimbal_test.cpp` | ✅ 保留 | 自瞄链路回归（`gimbal_test` 用直连串口 `io::Gimbal`，可作为回滚验证工具） |
| `auto_aim_test_web.cpp` | ⚠️ 删或改 | include 了 buff 头（见 4.5） |
| `auto_buff_test.cpp` `buff_detect_video_test.cpp` `buff_record.cpp` | ❌ 删 | buff 专用 |
| `camera_test.cpp` `camera_detect_test.cpp` `camera_thread_test.cpp` `usbcamera*_test.cpp` `multi_usbcamera_test.cpp` | ⚠️ 部分删 | usbcamera 系删除；camera 系保留（hikrobot 验证） |
| `cboard_test.cpp` `dm_test.cpp` | ❌ 删 | cboard/dm_imu 删除后无法编译 |
| `handeye_test.cpp` | ✅ 保留 | 标定流程 |
| `minimum_vision_system.cpp` | ⚠️ 需改 | include 了 `mt_detector`/`dm_imu`，需剥离多线程部分或删除 |
| `publish_test.cpp` `subscribe_test.cpp` `topic_loop_test.cpp` | ⚠️ 需改或删 | 依赖 `io/ros2/ros2.hpp`（publish2nav/subscribe2nav）；若删除这些工具则测试同步删 |

### 1.9 calibration/（5 个标定工具源码）
保留（独立可执行，链接 tools，不依赖被删模块；体积 KB 级，现场标定必需）。

### 1.10 docs/（10 个文档，168 KB）
- 保留：自瞄相关（`autoaim_analysis.md` `MPC控制与调参指南.md` `自瞄代码阅读指南.md` `自瞄模块解析.md` `调用时序与YAML参数映射.md` `运行入口与文件关系.md` `网页热调参数链路说明.md` `标定流程与参数回填清单.md` `文档导航.md`）
- 删除：`outpost_analysis.md`（前哨站，属自瞄但为独立优化记录，可归档不删）
- 实际建议：**全部保留**（总 168 KB，删除收益可忽略，且是团队知识资产）

---

## 二、保留链路完整依赖闭包（自动追踪结果，33 个文件）

```
src/auto_aim_debug_mpc.cpp
io/camera.hpp/cpp、io/hikrobot/*、io/command.hpp
io/gimbal/gimbal.hpp/cpp（零改动：结构体+枚举定义）
io/ros2/gimbal_topic_bridge.hpp/cpp
io/serial/*（零改动）
tasks/auto_aim/：aimer armor classifier detector solver target tracker voter shooter yolo
tasks/auto_aim/yolos/：yolov5.cpp（yolov8/yolo11 按需删或留）
tasks/auto_aim/planner/：planner.cpp + tinympc/*
tools/：debug debug_visualization exiter extended_kalman_filter img_tools logger math_tools
       openvino_utils path plotter recorder runtime_params thread_safe_queue trajectory web_debugger yaml
```

注意闭包追踪发现的三处**跨界 include（需要在方案中处理）**：

1. `tracker.hpp` → `#include "tasks/omniperception/perceptron.hpp"`（用 `omniperception::DetectionResult` 作为多相机融合 track 重载的参数类型）
2. `aimer.hpp` → `#include "io/cboard.hpp"`（实际只用了 `io::ShootMode` 枚举）
3. `debug_visualization.hpp` → `#include "tasks/auto_aim/aimer.hpp"`（连带来 cboard）

---

## 三、改动清单（按执行顺序）

### 阶段 0：备份与安全网
- 建议先 `git init` + 全量提交（当前 `D:\ATS` **不是 git 仓库**，删除不可恢复！）
- 或至少压缩备份 `src/sp_vision25` 到仓库外
- 在 Linux/WSL 侧先跑通一次 `colcon build --packages-select sp_vision25` 作为基线

### 阶段 1：删资源（最大收益，零代码风险）
| 操作 | 对象 |
|---|---|
| 删目录 | `assets/demo/`（111.6 MB） |
| 删目录 | `io/mindvision/`（27.4 MB） |
| 删文件 | `assets/` 下 9 个无用模型（见 1.6，共 29.0 MB） |

### 阶段 2：解耦合（3 处小改动，先于删代码）
1. **`tasks/auto_aim/tracker.hpp/.cpp`**：删除 `track(detection_queue...)` 多相机重载与 `#include "tasks/omniperception/perceptron.hpp"`（主链路只调用 `track(armors, t)`，已核实）
2. **`io/command.hpp` 或新文件**：把 `io::ShootMode` 枚举从 `cboard.hpp` 移入 `io/command.hpp`（通信结构体不涉及，仅头文件归属调整；`aimer.hpp` 改 include `io/command.hpp`）
3. **`tools/thread_pool.hpp`**：omniperception 删除后无使用者，可留可删（建议删，CMake 同步）

### 阶段 3：删源码目录
| 删除对象 | 说明 |
|---|---|
| `tasks/auto_buff/` | 整目录 |
| `tasks/omniperception/` | 整目录 |
| `src/` 下 12 个入口 .cpp（除 auto_aim_debug_mpc） | 见 1.2 |
| `tests/` 下 buff/cboard/dm/usbcamera 系测试 | 见 1.8 |
| `io/usbcamera/` `io/dm_imu/` `io/cboard.cpp/.hpp` `io/socketcan.hpp` | 见 1.4 |
| `io/ros2/publish2nav.*` `subscribe2nav.*` `ros2.*` `vision_target.hpp` | 见 1.4（注意 4.1） |

### 阶段 4：改 CMake
`src/sp_vision25/CMakeLists.txt`：
- 删除 12 个 `add_executable`（仅留 `auto_aim_debug_mpc` + 保留的测试 + 标定 5 个）
- 删除 `add_subdirectory(tasks/auto_buff)` `add_subdirectory(tasks/omniperception)`
- `auto_aim_debug_mpc` 的 `target_link_libraries` 中去掉 `auto_buff`
- 删除所有 `auto_buff_test/buff_*/camera_*` 等测试 target 与 `SP_VISION_RUNTIME_TARGETS` 对应项
- 删除 `find_package(Ceres ...)`（在 tasks/auto_buff/CMakeLists.txt 内，随目录删）

`src/sp_vision25/io/CMakeLists.txt`：
- `IO_SOURCES` 中删 `usbcamera/usbcamera.cpp` `cboard.cpp` `dm_imu/dm_imu.cpp`
- 删除 MindVision 查找整段（`MINDVISION_SDK_HINT_PATHS` 到 `SP_VISION_HAS_MINDVISION_SDK`）
- ROS 段 `target_sources` 只留 `ros2/gimbal_topic_bridge.cpp`（删 `publish2nav.cpp ros2.cpp subscribe2nav.cpp`）
- **若要保持"无 ROS 也能编译"的可移植性**（原设计），`sp_msgs` 条件保持不变——桥接本身不依赖 sp_msgs，但 CMake 条件链里含 `sp_msgs_FOUND`，可一并简化为只查 `rclcpp/std_msgs`（见 4.6）

`src/sp_vision25/tools/CMakeLists.txt`：
- 删 `ransac_sine_fitter.cpp` `pid.cpp`（`thread_pool.hpp` 若删源则无需改，它是纯头文件）

`src/sp_vision25/tasks/auto_aim/CMakeLists.txt`：
- 删 `multithread/commandgener.cpp` `multithread/mt_detector.cpp`（仅 `mt_standard`/`minimum_vision_system` 使用，主链路不用）
- `yolos/yolov8.cpp` `yolos/yolo11.cpp` 按模型清理决定同步删或留

### 阶段 5：清配置
- `configs/` 删 6 个非 sentry 配置（见 1.7）
- `sentry.yaml` **不改**（`vision_fusion` 段是给 omniperception decider 读的，主程序不读该段，留着无害；`can` 相关键同样无人读取，不动——**避免任何可能影响行为的配置变更**）

### 阶段 6：外层引用同步
| 文件 | 改动 |
|---|---|
| `src/sp_vision25/run_with_watchdog.sh` | 默认 BIN/CONFIG 改为 `auto_aim_debug_mpc` + `sentry.yaml` |
| `src/sp_vision25/debug_mpc_watchdog.sh` | 同上；删 `sentry_debug` 分支 |
| `src/ats_bringup/launch/ats_full.launch.py` | **无需改**（已指向 auto_aim_debug_mpc + sentry.yaml） |
| `README.md` / `src/sp_vision25/README.md` | 更新目录说明与命令（README 5.7 节看门狗命令、8.7 可执行程序清单） |
| `package.xml` | 可移除 `sp_msgs` 依赖（若按 4.6 简化 CMake）；描述文字更新 |

---

## 四、风险点与对策

### 4.1 `vision/target` 话题断供 ⚠️ 最重要的架构风险
`pb2025_sentry_behavior` 订阅 `vision/target`（`sp_msgs/VisionTargetMsg`）用于视觉跟随决策。该话题当前**只有被删的 `sentry*.cpp`（经 publish2nav）发布**。
- **事实**：`auto_aim_debug_mpc` 本身不发布该话题（已核实主循环无此发布）。
- **影响**：删码后行为树的 `IsVisionTargetValid` 永远 false，机器人退化为纯导航巡逻（不会报错，但视觉跟随功能不可用）。
- **决策点（需用户确认）**：
  - A. 接受现状：仅自瞄，无视觉引导导航（当前 ats_full 链路本来就没发这个话题，**行为与现状一致，无回归**）
  - B. 保留 `io/ros2/publish2nav.*` + `vision_target.hpp`，未来在主程序补一个最小发布器（保留约 3 个文件，体积可忽略）
  - **建议 B**，保留代价极低且为视觉-行为树联动留接口。
- **注意**：`sentry.yaml` 的 `vision_fusion` 段与 behavior 的 goal 索引联动文档在 `docs/sentry_bt_decision_checklist.md`，删除配置前先交叉核对。

### 4.2 tracker 与 omniperception 的耦合
删除 `track(detection_queue...)` 重载是纯删法（主链路只用 `track(armors, t)`，两处调用点已核实），不影响自瞄逻辑。`thread_pool.hpp` 随之无人使用。

### 4.3 `io::ShootMode` 枚举迁移
从 `cboard.hpp` 移到 `io/command.hpp` 是**纯移动**，枚举值不变（`left_shoot=0, right_shoot=1, both_shoot=2`），不触碰任何通信结构体。

### 4.4 camera.cpp 的 MindVision 分支
`io/mindvision/` 删除后，CMake 找不到 SDK → `SP_VISION_HAS_MINDVISION_SDK=OFF` → `#ifdef` 分支本来就不会编。但为了整洁与避免误配 `camera_name: mindvision` 时报模糊错误，建议把 `camera.cpp` 中 mindvision 分支与 `#ifdef` 一并删净，只留 hikrobot。**`sentry.yaml` 中 `camera_name: "hikrobot"` 不变。**

### 4.5 `auto_aim_test_web.cpp`
该测试同时 include 自瞄与 buff 头（`auto_buff_test.cpp` 注释提到与 buff 录像联动）。若保留需先剥离 buff include；最简单是删除该测试（自瞄 Web 调试能力已由主程序 `auto_aim_debug_mpc` 的 WebDebugger 覆盖）。

### 4.6 CMake 条件链简化（可选）
当前 ROS 条件包含 `sp_msgs_FOUND`，而保留链路（gimbal_topic_bridge）只用 `rclcpp + std_msgs`。若采用 4.1-B 保留 publish2nav，则 `sp_msgs` 依赖必须保留；若选 A 可移除。**建议保守起见保留现状条件**，避免牵动 `package.xml`/行为树构建顺序。

### 4.7 编译产物
`build/sp_vision25` `install/sp_vision25` 中会残留旧文件；改造完成后执行一次全清重编，并确认 `install/.../lib/sp_vision25/` 只剩预期可执行。

### 4.8 Git 缺失
工作区当前**没有 git 仓库**（`.git` 不存在），所有删除**不可逆**。阶段 0 的备份是硬前提。

### 4.9 AGENTS.md 工作区指南
指南假定存在 `src/sp_vision25` 完整结构。改造后无需改 AGENTS.md（包级路径仍有效），但 `docs/` 下若删文件需同步 README 索引。

---

## 五、通信不变性保证清单（逐项核对）

| 项 | 位置 | 改造后状态 |
|---|---|---|
| `GimbalToVision` 结构体（head/mode/q[4]/yaw/yaw_vel/pitch/pitch_vel/bullet_speed/bullet_count/crc16，`static_assert <= 64`） | `io/gimbal/gimbal.hpp` | ✅ 零改动 |
| `VisionToGimbal` 结构体（head/mode/yaw/yaw_vel/yaw_acc/pitch/pitch_vel/pitch_acc/crc16） | `io/gimbal/gimbal.hpp` | ✅ 零改动 |
| `GimbalMode` 枚举（IDLE/AUTO_AIM/SMALL_BUFF/BIG_BUFF） | `io/gimbal/gimbal.hpp` | ✅ 零改动（SMALL_BUFF/BIG_BUFF 枚举值保留——它们是协议一部分，即使自瞄不用 buff） |
| `io::Gimbal` 类（串口实现） | `io/gimbal/gimbal.cpp` | ✅ 保留零改动（gimbal_test 回归工具需要；bridge 也 include 此头） |
| 话题 `/serial/gimbal_state`（11 元素 Float64MultiArray） | `io/ros2/gimbal_topic_bridge.cpp` | ✅ 零改动 |
| 话题 `/serial/gimbal_cmd`（7 元素 Float64MultiArray） | `io/ros2/gimbal_topic_bridge.cpp` | ✅ 零改动 |
| mode 4/5 仲裁丢包逻辑 | bridge `send()` | ✅ 零改动 |
| `io/serial/` 库 | `io/serial/` | ✅ 保留零改动 |
| 导航侧 `standard_robot_pp_ros2` | 不在本方案范围 | ✅ 完全不动 |
| `Start/` 协议规格与补丁存档 | 不动 | ✅ 完全不动 |

---

## 六、预期收益

| 指标 | 改造前 | 改造后 |
|---|---|---|
| `src/sp_vision25` 体积 | 186.9 MB | ≈18.9 MB（**-90%**） |
| 入口可执行 | 13 + 24 测试 | 1 + ≈12 保留测试 + 5 标定 |
| 构建目标 | 40+ | ≈18 |
| 编译时间 | 长（Ceres + 全部入口） | 显著缩短（去 Ceres、OpenVINO 模型解析减少） |
| 依赖 | Ceres / MindVision SDK / sp_msgs(条件) | 去除 Ceres、MindVision |

## 七、验证方案（改造后必做）

1. `colcon build --packages-select sp_vision25 --cmake-args -DCMAKE_BUILD_TYPE=Release` 成功
2. `install/sp_vision25/lib/sp_vision25/` 中存在 `auto_aim_debug_mpc`（及预期测试）
3. `ros2 launch ats_bringup ats_full.launch.py` 正常启动
4. 桥接验证（README 7.1）：
   ```bash
   ros2 topic hz /serial/gimbal_state    # ≥200Hz
   ros2 topic echo /serial/gimbal_cmd    # 7 元素
   ```
5. 手动注入验证（README 7.2）11 元素 state 话题，确认主程序无 crash
6. （若有硬件）实机自瞄一轮，确认弹道与删前一致
7. 通信结构体字节断言仍在编译期生效（`static_assert(sizeof(GimbalToVision) <= 64)` 在 gimbal.hpp，文件未动即自动保证）

## 八、待用户决策项

1. **`vision/target` 话题**：方案 A（纯自瞄，无视觉引导）还是 B（保留 publish2nav 备用）？→ 建议 B
2. **`yolo11/yolov8` 模型与代码**：当前 sentry.yaml 用 yolov5。删掉另外两个 YOLO 变体可再省 ≈8 MB 与 2 个源文件；若想保留换模型灵活性则留。→ 建议删（git 已有历史）
3. **`calibration/` 与标定测试**：保留（体积小、现场必需）
4. **`configs/camera.yaml calibration.yaml handeye.yaml mvs.yaml`**：保留（KB 级）
5. **`docs/outpost_analysis.md`**：保留（团队记录）

---

*本方案未做任何文件修改，等待确认后执行。*