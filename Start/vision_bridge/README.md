# vision_bridge 接入说明（视觉仓库侧）

> 本目录代码**不直接改动** `sp_vision25-debug`，评审通过后按下述步骤并入。
> 改造基准：`src/auto_aim_debug_mpc.cpp`（用户确认的哨兵实际运行程序）。

## 文件清单

| 文件 | 作用 | 并入目标 |
|---|---|---|
| `gimbal_topic_bridge.hpp/.cpp` | `io::Gimbal` 的话题化替身，接口完全兼容 | 新增 `io/ros2/gimbal_topic_bridge.hpp/.cpp` |
| `auto_aim_debug_mpc.diff` | 主程序 3 行改动 | 应用于 `src/auto_aim_debug_mpc.cpp` |

## auto_aim_debug_mpc.cpp 的全部改动（就这 3 处）

```diff
 // 头部 include 区
+#include "io/ros2/gimbal_topic_bridge.hpp"

 // 构造（原第 156 行附近）
-  io::Gimbal gimbal(config_path);
+  io::GimbalTopicBridge gimbal;   // 不再打开串口；com_port 配置对它无效

 // 其余所有 gimbal.q(t) / gimbal.state() / gimbal.send(...)
 // 调用点【零改动】——接口签名完全一致
```

## 设计说明

### 为什么做"接口兼容替身"而不是改主程序

`auto_aim_debug_mpc.cpp` 实际只有 **4 处** `gimbal.*` 调用：`q(t)`(L194)、`state()`(L202)、
`send(...)`(L209)、`send(false,...)`(L549)；**无任何 `mode()`/`str()` 调用**。做接口兼容的
`GimbalTopicBridge` 后主程序只改 1 行构造，其余零改动——回滚也只改回 1 行。

### 与原 io::Gimbal 的行为对应

| 原（串口版） | 本桥接版 | 一致性 |
|---|---|---|
| read_thread 收 42B 帧 → 姿态队列 | 订阅 gimbal_state 回调 → 双样本缓存 | 字段与 wxyz 语义一致 |
| q(t) 双样本 slerp + `pop_for(100ms)` 等待 | 相同算法 + 条件变量 100ms 等待 | 分支一致，等待语义等价实现 |
| mode 0~3 映射 + 非法回退 AUTO_AIM+告警 | 相同 + 扩展 4/5 丢包（见下表） | 兼容超集 |
| send 参数化版 mode=control?(fire?2:1):0 | 相同（4/5 时丢包） | 一致 |
| send 结构体版（不拷 head） | 结构体版转为参数化版发送 | head 概念随串口消失 |
| 串口重连/CRC | 不再存在（导航节点负责串口） | 职责移交 |

### mode 映射留档（仲裁：导航优先，记录表同步见 Start/README.md）

**接收（gimbal_state[0] → 内部 GimbalMode）：**

| 收到 | 映射结果 | 行为 |
|---|---|---|
| 0 | IDLE | 正常空闲 |
| 1 | AUTO_AIM | 正常自瞄决策发送 |
| 2 | SMALL_BUFF | 打小符 |
| 3 | BIG_BUFF | 打大符 |
| 4 NAV_OVERRIDE | **IDLE + send 丢包** | 导航接管：**桥接层 send() 直接返回，不发布 gimbal_cmd** |
| 5 CONFLICT | **IDLE + send 丢包** | 冲突降级：桥接层丢包 + 告警日志 |
| 6/其他 | AUTO_AIM + 告警 | 与原协议非法值处理一致 |

> **实现依据（重要）**：基准程序 `auto_aim_debug_mpc.cpp` **不读取 `gimbal.mode()`**
> （全文仅 q/state/send 四类调用，主循环无条件调用 `gimbal.send(...)`），不存在
> "主程序 else 分支"可言——若不在 `send()` 内丢包，mode 4/5 对视觉侧完全不生效，
> 仲裁只剩电控端单侧屏蔽。因此本版本已在 `GimbalTopicBridge::send()` 内对 4/5
> 统一丢包；电控端同时按协议规格书 §3.3 屏蔽视觉指令，形成双侧防护。

**发送（gimbal_cmd[0]）：** 0/1/2 语义与原 VisionToGimbal.mode 完全一致，不扩展。

## 构建接入

`io/CMakeLists.txt` 的 ROS2 条件块中追加：

```cmake
target_sources(io PRIVATE ros2/gimbal_topic_bridge.cpp)
```

（视觉侧本来就有 rclcpp/sp_msgs 依赖与 ROS 环境探测，无需新增 find_package。）

## 已知限制 / 待办

- [ ] gimbal_state 话题发布频率依赖导航节点接收回调（即电控 ID_GIMBAL_STATE 频率）；
      联调时确认 ≥200Hz，否则 slerp 插值精度下降
- [ ] `Float64MultiArray` 为过渡方案；若导航侧后续启用 sp_msgs 强类型消息，同步替换
- [ ] 回滚方案：主程序构造行改回 `io::Gimbal gimbal(config_path);` 即恢复原串口链路