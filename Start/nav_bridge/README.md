# nav_bridge 接入说明（导航仓库侧）

> 本目录代码**不直接改动** `ATS_2026_snetry_test-main`，评审通过后按下述步骤并入。

## 文件清单

| 文件 | 作用 | 并入目标 |
|---|---|---|
| `packet_typedef_addon.hpp` | 新增 ID_GIMBAL_STATE(0x0E)/ID_GIMBAL_CMD(0x02) 包定义 + static_assert(≤64B, 49B, 35B) + mode 扩展枚举 | 拷贝内容并入 `include/standard_robot_pp_ros2/packet_typedef.hpp` 末尾（namespace 内） |
| `gimbal_bridge_addon.hpp` | 视觉桥接：gimbal_state 发布 + gimbal_cmd 订阅转发（独立线程） | 新增为 `include/standard_robot_pp_ros2/gimbal_bridge_addon.hpp` |

## 并入步骤

1. **包定义**：把 `packet_typedef_addon.hpp` 的内容并入 `packet_typedef.hpp`（保留 `static_assert`，防止后续改动破坏 64B 约束）。

2. **节点接线**（`standard_robot_pp_ros2.hpp` / `.cpp`）：
   ```cpp
   // hpp: 类成员（值成员即可，不要用 make_unique 给值成员赋值）
   GimbalBridgeAddon gimbal_bridge_;

   // cpp 构造函数成员初始化列表（串口发送函数必须构造时注入，
   // 发送线程在构造函数内即启动，事后注入会有 std::function 并发读写 UB）：
   gimbal_bridge_(
     [this](const std::vector<uint8_t> & v) {
       if (is_usb_ok_) {
         try { serial_driver_->port()->send(v); }
         catch (const std::exception &) { is_usb_ok_ = false; }
       }
     }),
   ```

3. **接收分发**（`receiveData()` 的 switch 新增分支，放在 ID_BUFF 之后）：
   ```cpp
   case ID_GIMBAL_STATE: {
     ReceiveGimbalStateData gimbal_state_data = fromVector<ReceiveGimbalStateData>(data_buf);
     gimbal_bridge_->publishGimbalState(gimbal_state_data);
   } break;
   ```

4. **配置**（`config/standard_robot_pp_ros2.yaml`）：
   ```yaml
   baud_rate: 921600   # 原 115200；带宽核算见协议规格书 §4
   debug: false        # 禁用 160B debug 包（64B 约束）
   ```

## 话题接口

| 话题 | 类型 | 方向 | 内容（Float64MultiArray 索引） |
|---|---|---|---|
| `serial/gimbal_state` | std_msgs/Float64MultiArray | 导航→视觉 | [0]mode [1..4]q_wxyz [5]yaw [6]yaw_vel [7]pitch [8]pitch_vel [9]bullet_speed [10]bullet_count |
| `serial/gimbal_cmd` | std_msgs/Float64MultiArray | 视觉→导航 | [0]mode [1]yaw [2]yaw_vel [3]yaw_acc [4]pitch [5]pitch_vel [6]pitch_acc |

单位：一律弧度 / rad/s / rad/s²。

## 设计要点

- **gimbal_cmd 用独立线程转发（上限 200Hz）**，不并入现有 `sendData()` 的 5ms 周期：视觉按相机帧率（~100-250Hz）更新话题，桥接线程以最新值打包发送，保证指令低延迟且不干扰导航发包节拍。
- **失联检测（与协议规格书 §3.2 对齐）**：转发线程记录话题最后一帧到达时刻，超过 **100ms 无新帧即停发**（对应电控端"≥100ms 无新帧按 mode=0 处理"的超时语义）。绝不在视觉失联后无限重发旧帧——否则电控超时机制永远无法触发。
- **gimbal_state 直接在 receiveData 回调里发布**，带 ROS `now()` 时间戳语义（视觉 slerp 依赖到达时序，不透传下位机 time_stamp）。
- **不动现有任何包字段**；新增 ID 只用空闲的 0x0E（接收）与 0x02（发送）。
- static_assert 在编译期锁定 49B/35B 与 ≤64B，防止后人误加字段破坏 USB 约束。

## 已知待办（联调前）

- [ ] 若希望强类型消息（替代 Float64MultiArray），在 `sp_msgs` 定义 `GimbalStateMsg/GimbalCmdMsg` 并同步两侧
- [ ] 回环测试：`ros2 topic pub /serial/gimbal_cmd ...` 后用串口助手/逻辑分析仪确认 35B 帧