# ATS 一体化融合方案（评审稿）

> 日期：2026-09-19
> 目标：将 `ATS_2026_snetry_test-main`（导航）与 `sp_vision25-debug`（视觉）融合为一个整体工作区 **ATS**（即 `D:\ATS`），实现：
> 1. 一次 `colcon build` 同时编译导航与视觉（统一 Release）；
> 2. 运行时通过既有 ROS2 桥接话题通信（沿用 `Start/` 方案设计）；
> 3. 一条 launch 指令同时启动两模块。
>
> **本文件只是方案，未做任何改动。**

---

## 一、现状检查结论

### 1.1 视觉包 `sp_vision25-debug`

| 检查项 | 结论 |
|---|---|
| 包身份 | **已是合法 ament_cmake 包**（`package.xml` 声明 `sp_vision25`，依赖 `rclcpp/std_msgs/sp_msgs`），可直接入工作区 |
| CMake | 顶层 `CMakeLists.txt` 已含 `find_package(ament_cmake)`、`ament_target_dependencies`、`ament_package()`、install 规则（可执行→`lib/sp_vision25`，configs/assets/docs→`share/sp_vision25`），colcon 化程度比预期高 |
| 主入口 | 哨兵实际运行程序为 `src/auto_aim_debug_mpc.cpp`（仅 4 处 `gimbal.*` 调用，无 `mode()` 调用） |
| 路径解析 | `tools/path.cpp` 通过编译期宏 `SP_VISION_SOURCE_DIR`（`tools/CMakeLists.txt` 注入 `PROJECT_SOURCE_DIR`）+ `share/sp_vision25` 反推双路定位 assets/configs，**迁移后无需改代码** |
| 硬编码路径 | ① `io/CMakeLists.txt` HikRobot SDK 提示路径 `${CMAKE_SOURCE_DIR}/../src/dependencies/hik_camera_ros2_driver/hikSDK`（基于“视觉包与 src/ 平级”的旧假设，迁移后失效需改）；② `run_with_watchdog.sh`/`debug_mpc_watchdog.sh` 写死 `build/<bin>`（colcon 后失效需改或删）；③ 顶层 `set(CMAKE_BUILD_TYPE Release)` 硬编码会覆盖外部传参 |
| 通信 | 现 `io::Gimbal` 直接开串口，与导航抢 `/dev/ttyACM0`——按 `Start/vision_bridge` 方案改为话题桥接 |

### 1.2 导航工作区 `ATS_2026_snetry_test-main`

| 检查项 | 结论 |
|---|---|
| 结构 | 本身就是一个 colcon 工作区（`src/` 下 30+ 包），顶层不是 ROS 包 |
| 串口节点 | `standard_robot_pp_ros2` 独占串口，含 `receiveData()` switch 分发、`is_usb_ok_`/`serial_driver_` 成员——与 `Start/nav_bridge` 并入步骤完全吻合 |
| yaml | `device_name: /dev/ttyACM0`、`baud_rate: 115200`、`debug: false`——需按协议规格书改 921600 |
| 顶层脚本 | `NAV2.sh`/`mapping.sh`/`pp_ros2.sh`/`rosbag.sh` 全部硬编码 `/home/ats/pd/pb2025_sentry_ws`，迁移后路径全部失效 |
| 文档 | 顶层 4 份 README（`README.md`/`de_README.md`/`nav_README.md`/`ws_README.md`）内容有重叠且均基于旧目录结构 |

### 1.3 `Start/` 桥接补丁状态

代码完整但**尚未并入任何仓库**：`nav_bridge/`（packet_typedef_addon.hpp + gimbal_bridge_addon.hpp 及接线说明）、`vision_bridge/`（gimbal_topic_bridge.hpp/.cpp + 3 行 diff）。`Start/` 无 package.xml，colcon 会无视它——**桥梁是 ROS2 话题，不是文件夹**，故补丁应拆开并入两侧包。

---

## 二、目标目录结构（融合后的 ATS 工作区）

```
D:\ATS\                          ← 工作区根（就是"整个 ATS"）
├── src/
│   ├── dependencies/            ← 导航原有依赖（rmoss_core、BehaviorTree.ROS2 等）
│   ├── interfaces/  loopback_sim/  pb2025_robot_description/
│   ├── pb2025_sentry_behavior/  pb2025_sentry_bringup/
│   ├── pb2025_sentry_nav/        ← 含 sp_msgs（视觉包的依赖，自动拓扑排序）
│   ├── standard_robot_pp_ros2/   ← 导航串口节点（并入 nav_bridge 补丁）
│   ├── tools/                    ← 导航辅助工具包
│   ├── sp_vision25/              ← 视觉包（自 sp_vision25-debug 迁入，改名）
│   └── ats_bringup/              ← 【新建】统一启动入口包
├── build/  install/  log/        ← colcon 统一产物（编译时生成）
├── scripts/                     ← 修订路径后的启动脚本（NAV2.sh 等）
├── README.md                    ← 合并后的总 README（编译/启动/联调一站式）
├── AGENTS.md  .gitignore        ← 工作区级配置
└── Start/                       ← 保留：桥接方案与协议规格书存档（不参与构建）
```

要点：`ATS_2026_snetry_test-main/src/*` **上提一级**到 `D:\ATS\src\`；原两个顶层目录融合后删除（用户已备份）。

---

## 三、删除清单（按处理方式分类）

### 3.1 直接删除——对构建完全无用（用户已备份）

**视觉侧（sp_vision25-debug）：**

| 项 | 数量/大小 | 理由 |
|---|---|---|
| 所有 `*.bak*` 文件 | **106 个**（tools/tasks/tests/src/configs 下，含 `CMakeLists.txt.bak-20260913-record`） | 历史快照，编译不引用 |
| `_github_upload/` 整目录 | 含独立 `.git`、assets、全源码副本 | 旧上传副本，纯冗余 |
| `.marscode/` | — | IDE 缓存 |
| `.git/`（迁移拷贝时排除） | — | 旧仓库历史，备份已在别处 |
| `.vscode/` | — | 个人 IDE 配置（可选保留） |

**导航侧（ATS_2026_snetry_test-main）：**

| 项 | 理由 |
|---|---|
| `.codex` | 零字节 AI 工具残留 |
| 顶层 `de_README.md`、`nav_README.md`、`ws_README.md` | 内容并入新总 README 后删除（信息不丢失，只是合并） |
| `docs/`、`patterns/`、`scripts/codex_brief.sh` | codex 工作流文档，与构建无关；建议**保留**为 `D:\ATS\docs\`（很小，是团队知识），也可按用户意思删 |

### 3.2 修订后保留——路径失效需改

| 文件 | 原路径假设 | 修订 |
|---|---|---|
| `NAV2.sh` `mapping.sh` `pp_ros2.sh` `rosbag.sh` | `/home/ats/pd/pb2025_sentry_ws` | 改为脚本自定位工作区根（`$(cd "$(dirname ...)" && pwd)`），不再写死绝对路径 |
| `run_with_watchdog.sh` `debug_mpc_watchdog.sh` | `${ROOT}/build/<bin>` | 改为 `install/sp_vision25/lib/sp_vision25/<bin>`，或改为 `ros2 run sp_vision25 ...` |
| `build.sh` | 独立 CMake 构建入口 | **删除**（colcon 统一构建后失去意义；独立调试方式写入总 README 命令即可） |

### 3.3 评估后保留

| 项 | 理由 |
|---|---|
| `assets/demo/`（含 93MB 视频） | 不参与编译，但 `auto_buff_test` 等离线测试入口依赖；**建议保留**，若确定只做实车联调可删（省约 115MB） |
| `assets/` 模型文件 | 运行时必需，install 到 share |
| `loopback_sim` 及仿真相关包 | 回环/仿真联调要用 |
| `CMakeLists.txt.bak` 之外的全部源码 | 均被编译引用 |

---

## 四、迁移与接线步骤（按顺序执行）

### 步骤 1：目录迁移（PowerShell，只 Move/复制，不破坏内容）

```
1. D:\ATS\src\  ← 新建
2. ATS_2026_snetry_test-main\src\*  → D:\ATS\src\   （上提）
3. sp_vision25-debug → D:\ATS\src\sp_vision25        （改名迁入，排除 .git/.marscode/_github_upload/*.bak*）
4. ATS_2026_snetry_test-main 顶层文件（README/AGENTS/脚本/文档）→ D:\ATS\
5. 删除两个旧顶层目录
```

### 步骤 2：视觉包 CMake 修订（3 处）

```cmake
# ① 顶层 CMakeLists.txt：硬编码 Release 改为保护式，允许外部统一传参
- set(CMAKE_BUILD_TYPE Release)
+ if(NOT CMAKE_BUILD_TYPE)
+   set(CMAKE_BUILD_TYPE Release)
+ endif()
```

```cmake
# ② io/CMakeLists.txt：SDK 搜索路径适配新目录关系
#    旧：${CMAKE_SOURCE_DIR}/../src/dependencies/...（假设视觉包与 src 平级）
#    新：${CMAKE_SOURCE_DIR}/../dependencies/hik_camera_ros2_driver/hikSDK
#    （sp_vision25 现在就在 src/ 下，dependencies 与它同级）
```

```cmake
# ③ io/CMakeLists.txt ROS2 块：接入话题桥接（Start/vision_bridge 方案）
  target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp
+                           ros2/gimbal_topic_bridge.cpp)
```

### 步骤 3：视觉主程序接入桥接（3 行 diff，来自 Start/vision_bridge）

```diff
+ #include "io/ros2/gimbal_topic_bridge.hpp"
- io::Gimbal gimbal(config_path);
+ io::GimbalTopicBridge gimbal;    // 其余 q()/state()/send() 零改动
```

同时新增文件：`io/ros2/gimbal_topic_bridge.hpp/.cpp`（自 `Start/vision_bridge/` 拷入）。

### 步骤 4：导航串口节点接入桥接（Start/nav_bridge 并入）

1. `packet_typedef_addon.hpp` 内容并入 `include/standard_robot_pp_ros2/packet_typedef.hpp`（新增 ID_GIMBAL_STATE=0x0E / ID_GIMBAL_CMD=0x02 + static_assert）；
2. `gimbal_bridge_addon.hpp` 新增至 `include/standard_robot_pp_ros2/`，CMakeLists 无需改（header-only，当前 CMake 是按源文件编译，include 目录已导出）；
3. `standard_robot_pp_ros2.hpp/.cpp` 按接入说明接构造注入 + `receiveData()` switch 新增分支；
4. `config/standard_robot_pp_ros2.yaml`：`baud_rate: 115200 → 921600`（带宽核算见 `Start/doc/协议规格书.md` §4）。

### 步骤 5：新建 `ats_bringup` 统一启动包

```
D:\ATS\src\ats_bringup/
├── package.xml        ← ament_cmake；exec_depend 两侧关键包
├── CMakeLists.txt     ← install launch 目录
└── launch/ats_full.launch.py
```

launch 内容（核心逻辑）：

```python
# 1) 导航串口节点 standard_robot_pp_ros2（含桥接，参数指向修订后 yaml）
# 2) 视觉主程序 ros2 run sp_vision25 auto_aim_debug_mpc configs/sentry.yaml
# 3)（可选参数）include pb2025_sentry_bringup 的导航 bringup
```

启动指令即用户要求的"同时被指令启动"：

```bash
ros2 launch ats_bringup ats_full.launch.py
```

### 步骤 6：顶层脚本与文档

1. `NAV2.sh` 等 4 个脚本按 3.2 修订路径（自定位 + `install/setup.bash`）；
2. 合并 4 份 README 为 `D:\ATS\README.md`：编译、启动、联调、回滚一站式；
3. `Start/` 原样保留为方案存档，README 加一行"已并入，见总 README"。

---

## 五、修订后的终端指令（新路径版）

```bash
# ===== 编译（WSL / Ubuntu 22.04, ROS 2 Humble）=====
cd /mnt/d/ATS            # Windows 下；实机为实际挂载路径
source /opt/ros/humble/setup.bash
export HIKROBOT_SDK_ROOT=/opt/MVS        # 或保留工作区内 SDK 回退路径
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 1                    # WSL 内存紧张；实机可提高

# 只重编改动的包
colcon build --packages-select sp_vision25 standard_robot_pp_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# ===== 启动 =====
source install/setup.bash
ros2 launch ats_bringup ats_full.launch.py

# ===== 验证桥接通信 =====
ros2 topic hz /serial/gimbal_state        # 应 ≥200Hz（电控→视觉）
ros2 topic echo /serial/gimbal_cmd         # 视觉决策帧
```

---

## 六、风险与验证清单

| # | 风险 | 验证手段 |
|---|---|---|
| 1 | HikRobot SDK 路径迁移后找不到 | 编译 sp_vision25 观察 "Using HikRobot SDK" 日志；必要时 `export HIKROBOT_SDK_ROOT` |
| 2 | sp_msgs 与视觉包依赖拓扑 | colcon 自动排序，观察构建顺序先 sp_msgs 后 sp_vision25 |
| 3 | OpenVINO 绝对路径（`$ENV{HOME}/openvino_toolkit_...`） | 迁移不改此路径，但换机器需重装；写进总 README 环境要求 |
| 4 | 串口仍被视觉旧代码占用 | 确认主程序只构造 `GimbalTopicBridge`，`io::Gimbal` 不再实例化 |
| 5 | 桥接后视觉收不到 200Hz | `ros2 topic hz /serial/gimbal_state`；不足则 slerp 精度下降（已知待办，规格书 §3.1） |
| 6 | Windows↔WSL 文件权限/软链 | `--symlink-install` 在 NTFS 挂载上可能异常；建议正式编译前 `git clone` 到 WSL 原生 ext4 或去掉 `--symlink-install` |
| 7 | 删除 `.bak` 后想找回 | 用户已备份；且 `_github_upload` 副本仍在备份中 |

## 七、不做的事（明确边界）

- 不改任何包的 `<name>`（包名改动会牵连几十处引用，无收益）；
- 不删除 `assets/demo` 大视频（除非用户确认纯实车）；
- 不动导航侧 13 种现有接收包字段与 mode 语义（协议规格书约束）；
- 不改电控/下位机代码（不在本仓库）。
