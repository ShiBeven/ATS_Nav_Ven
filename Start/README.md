# 串口融合通信改造（Start）

> 本目录保存"自瞄串口并入导航串口"改造的**新增代码与文档**。
> 原两个仓库源码一律不动，所有融合相关新代码集中在本目录，便于评审与回滚。

## 背景与决策记录（必读）

| 项 | 决策 | 决策人 |
|---|---|---|
| 串口合并方式 | 两个 USB 串口合并为一个，由**导航节点 standard_robot_pp_ros2 独占**，视觉经 ROS2 话题接入 | 老师(B) |
| 帧格式 | 沿用导航现有 `HeaderFrame+time_stamp+data+CRC16` 统一帧 | 老师(B)：把同济的融进去 |
| 64B 硬约束 | 所有帧总长 ≤ 64 字节（下位机 USB CDC 单帧限制） | 老师(B) |
| 分包 | **不做分包**。唯一超 64B 的 debug 包(160B)在哨兵配置中禁用（debug:false），下位机不得在合并链路发送 | C 提议，本次采纳 |
| 板子 | 两块板子相同，保留任一块均可 | 用户 |
| 哨兵视觉主程序 | 以 `sp_vision25-debug/src/auto_aim_debug_mpc.cpp` 为基准改造 | 用户 |
| 仲裁规则 | **导航优先**。模式冲突时不动导航现有 mode 值(0~3)，新增扩展值 4/5/6 表示融合链路新增状态，**必须保留映射表留档** | 用户 |
| 角度单位 | 一律**弧度制**（含角速度 rad/s） | 用户 |
| 波特率 | 建议提升至 921600（115200 带宽不足以承载 200Hz 导航包+自瞄包），**需电控确认板端** | 本方案 |

### 仲裁 / mode 扩展记录表（重要：电控与上位机必须逐项对齐）

**接收方向（电控→上位机）`ID_GIMBAL_STATE` 的 mode 字段**：

| mode | 含义 | 来源 | 备注 |
|---|---|---|---|
| 0 | 空闲 | 原视觉协议 GimbalToVision.mode=0 | 与视觉 GimbalMode::IDLE 对应 |
| 1 | 自瞄 | 原视觉协议 mode=1 | GimbalMode::AUTO_AIM |
| 2 | 小符 | 原视觉协议 mode=2 | GimbalMode::SMALL_BUFF |
| 3 | 大符 | 原视觉协议 mode=3 | GimbalMode::BIG_BUFF |
| **4** | **导航接管（NAV_OVERRIDE）** | **融合新增** | 导航仲裁优先时，电控置此值，视觉**桥接层丢包**（不发布 gimbal_cmd；基准主程序不读 mode，故必须在桥接层实现） |
| **5** | **导航/自瞄并发冲突（CONFLICT）** | **融合新增** | 电控检测到两路指令同时有效时置此值，视觉**桥接层丢包**并记日志 |
| **6** | **保留** | 融合预留 | 未定义，收到的视觉侧按 1(自瞄)回退并告警（与原协议非法值行为一致） |

> 说明：**不改导航现有 `SendRobotCmdData` 及 13 种接收包的任何字段与 mode 语义**；上表 4/5/6 只出现在新增的 `ID_GIMBAL_STATE` 包中。视觉侧映射记录见 `vision_bridge/README.md`。
>
> **仲裁落地点（重要）**：改造基准 `auto_aim_debug_mpc.cpp` **不调用 `gimbal.mode()`**（全文仅 `q/state/send` 四类调用，主循环无条件决策发送），因此"视觉侧停止发送/降级"必须由 `GimbalTopicBridge::send()` 在 mode=4/5 时直接丢包实现，**不能**依赖主程序按 mode 分支。电控端同时按 §3.3 屏蔽视觉指令，形成双侧防护。

## 目录结构

```
Start/
├── README.md                 ← 本文件：决策记录 + 总体方案
├── doc/
│   └── 协议规格书.md          ← 给电控的协议规格书（交付给 A）
├── nav_bridge/               ← 导航仓库侧新增/修改代码（拷贝进对应路径）
│   ├── packet_typedef_addon.hpp   新增包定义（含 static_assert ≤64B）
│   ├── ros2_addon.hpp/.cpp        新增 publisher/subscription 接线
│   └── ...
└── vision_bridge/            ← 视觉仓库侧新增/修改代码（拷贝进对应路径）
    ├── gimbal_topic_bridge.hpp/.cpp   Gimbal 话题化桥接（替代串口）
    └── README.md                       视觉侧接入说明 + mode 映射留档
```

## 数据流（目标架构）

```
[合并后单板(电控)]
   ║ 唯一 USB 串口(建议 921600, 8N1)
   ▼
[导航节点 standard_robot_pp_ros2]  ←— 现有 13 收包 + 1 发包全部不变
   ├─ 发布 /serial/gimbal_state (ID_GIMBAL_STATE 0x0E, ≥200Hz)
   └─ 订阅 /serial/gimbal_cmd  → 组包 ID_GIMBAL_CMD 0x02 发出
   ▲
   │ ROS2 (DDS)
   ▼
[视觉 auto_aim_debug_mpc(改造版)]
   ├─ 订阅 /serial/gimbal_state → 注入 io::Gimbal(时间戳队列/slerp/状态)
   └─ 发布 /serial/gimbal_cmd   ← 原 gimbal.send(...) 全部改为发话题
```

> **话题命名空间约束（部署必读）**：两侧话题均为相对名（`serial/gimbal_state` / `serial/gimbal_cmd`）。导航节点 launch 支持 `namespace` 参数（`PushRosNamespace`），一旦以非空 namespace 启动，话题会变为 `<ns>/serial/...` 而视觉节点无 namespace，**两侧将无法互通**。部署时导航必须以空 namespace 启动，或给视觉节点加相同 namespace。

## 交付物清单

- [x] 决策记录（本文件）
- [x] 给电控的协议规格书：`doc/协议规格书.md`
- [ ] nav_bridge 代码（拷贝进 ATS_2026_snetry_test-main 对应文件）
- [ ] vision_bridge 代码（拷贝进 sp_vision25-debug 对应文件）
- [ ] 回环联调（PTY 模拟下位机）验证