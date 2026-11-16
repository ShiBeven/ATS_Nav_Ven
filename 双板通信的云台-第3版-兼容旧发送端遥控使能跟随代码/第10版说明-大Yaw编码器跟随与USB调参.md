# 第10版：大Yaw编码器回中跟随与USB调参

日期：2026-10-15。基于“云台新”工程制作独立修改版，原桌面目录不改动。

## 2026-10-16 补充：已合入布局修复

- `Inc/Vision.h` 的 `#pragma pack(1)` 改为 `#pragma pack(push, 1)`，在末尾 `#endif` 前添加 `#pragma pack(pop)`，避免紧凑布局泄漏到后续头文件。
- 该头文件由 GBK 转为 UTF-8 无 BOM；按原编码解码并忽略换行差异核对，文本仅有上述两处修复。对比 304 个源码/工程文件，仅此头文件发生变化；未修改 PID、跟随逻辑、USB 协议或保护。
- 对 `PROJECT` 执行完整 Rebuild：0 错误、72 警告。最新日志为 `build-v10-pack-fix.log`；`MDK-ARM/OBJ/PROJECT.hex`、`PROJECT.axf` 已更新，保持 `USB_YAW_DEBUG=1`。
- 本次未烧录或进行实机测试。下文旧构建日志为历史记录，当前应使用上述最新固件。

## 控制关系

机械层级：底盘 -> 大Yaw -> DM所在框架 -> 小Yaw -> Pitch -> C板/枪管。

- 小Yaw仍使用C板INS世界航向和经过Pitch投影的角速度，实现枪管世界稳定。
- 普通遥控 `GIMBAL_ABSOLUTE_ANGLE` 模式的大Yaw外环改成小Yaw编码器相对中位偏角的比例回中控制；大Yaw内环使用DM `gyro[2]`。
- 大Yaw外环使用连续死区：小Yaw偏角超过死区后，扣除死区再乘Kp，避免死区边缘的速度突跳。目标速度限幅。
- 不使用DM的欧拉Yaw，避免把六轴IMU长期航向漂移引入回中控制。
- 视觉、导航模式原有控制律不改成编码器回中；仍保留原航向变量供这些模式使用。本版首先验证普通遥控模式。

## 修改文件

| 文件 | 修改 |
|---|---|
| application/gimbal_task.c/.h | 回中控制、RAM调试变量、DM保护、输出限幅、速度测试脉冲、六通道快照 |
| components/devices/dm_imu.c/.h | 中断/任务一致的角速度快照、20ms有效性检查、volatile字段、标准函数声明 |
| application/CAN_receive.c | 检查HAL接收结果，以及DM帧的标准帧/数据帧/DLC |
| application/usb_task.c/.h | USB调试模式、100Hz二进制波形、忙时跳帧、调试模式丢弃USB输入 |
| Src/freertos.c | USB任务栈增至256，移除弱test_task中的冗余USB初始化 |

USART1遥控器转发、USART3本地遥控接收、CAN遥控帧不删除、不注释。两个桌面工程的遥控代码均保留。本版不实现UART波形发送。

原 `test_task` 在application/test_task.c有强定义，覆盖freertos.c弱定义；移除弱函数里的USB初始化是消除潜在重复入口，不代表原固件必然重复初始化。USB由usb_task负责初始化。

普通速度PID初始化与自瞄专用PID分开。普通Kp默认8500，保留旧程序被覆盖后实际生效的8500/0/10，避免突然恢复成20000。后续仍需按DM反馈重新调参。

重复YAW2宏合并，保留原最终生效的速度输出20000和角度宏0~60度；这些宏不代表已经核实的机械限位。校准静止判断改为角速度绝对值。

## 安装与参数

- 大Yaw机械零位7773，小Yaw1118保持原值；小Yaw实际居中时 `relative_angle` 应接近0。上机前核对。
- DM：CAN、被动应答/请求模式、1Mbps、CAN_ID=0x30、MST_ID=0x31，保存并重启。
- TIM2约500Hz，隔次请求角速度，DM请求约250Hz。
- DM固定于大Yaw框架，不随小Yaw/Pitch单独运动；输出Z轴平行大Yaw轴。
- `DM_IMU_YAW_GYRO_SIGN` 默认+1，需要手动旋转验证。C板Pitch投影符号也需要实测验证。
- 大Yaw速度反馈是框架相对世界的Yaw轴角速度；适用同轴Yaw、底盘近水平条件。大倾斜时不能把单个机体Gz直接等同于欧拉Yaw导数。

## 上电和掉线保护

`dm_gyro_online`：至少收到一帧、年龄不超过20ms且Gz在范围内；未收到时年龄为0xFFFFFFFF。

`dm_fault_latched`初值1。DM掉线、板载陀螺仪掉线或遥控错误会锁定零力，清PID、电流和速度测试目标。所有云台模式均经过此输出门控。

健康数据恢复后，需要两拨杆均在下位才能解除锁定，再重新进入控制并完成机械归零。不会在拨杆保持工作位时自行恢复带力输出。

大Yaw所有模式的最终发送电流指令另受 `big_follow_current_limit` 限制。默认3000可能不足以完成某些负载的机械归零，可能初始化超时；须结合机构阻力和波形逐步调整，不能为“进模式”直接取消DM保护。

## USB与网站

1. C板使用原有电源，电机使用原动力电源；用带数据功能的USB线连接C板到电脑。
2. `application/usb_task.h` 中 `USB_YAW_DEBUG=1` 为调参模式，`=0`恢复原视觉/导航USB协议；修改后重新编译下载。
3. 默认调参模式不解析USB命令，也不发送导航控制和原0x0E状态包；原CAN定时通信保留。不要同时连接视觉/导航上位机。
4. Chrome/Edge打开 <https://yssickjgd.github.io/serial_plotter/>，选择原生串口Web Serial，连接C板COM口。可填115200、8N1，USB虚拟串口的物理传输速度不由该波特率决定。
5. 导入同目录 `SerialPlotter-双Yaw调参.json`。如果导入失败，手动设置以下表格。

| 帧格式设置 | 值 |
|---|---|
| 帧首 | AA 55，启用 |
| 数据类型/端序 | Float32、小端LE |
| 通道数 | 6 |
| 帧尾 | 0D 0A，启用 |
| 校验 | 关闭 |

每帧28字节，100Hz，约2800字节/秒；USB忙时跳过本帧。仅USB任务发送该帧，静态缓冲区在上帧完成后才重写。

| 通道 | 含义 | 单位 |
|---|---|---|
| CH1 | 大Yaw目标速度 | rad/s |
| CH2 | 大Yaw实际DM反馈速度 | rad/s |
| CH3 | 大Yaw限幅后的given_current发送指令 | 指令值，非安培 |
| CH4 | 小Yaw编码器相对机械中位偏角 | rad |
| CH5 | 枪管目标航向 | rad |
| CH6 | 枪管实际C板航向 | rad |

先显示CH1/CH2调速度，CH3单独看限幅；显示CH4看回中，CH5/CH6看枪管世界稳定。接近正负Pi时航向曲线会折返，这是角度包络，不应直接当作机械突跳。

网站说明与配置字段核对来源：<https://github.com/yssickjgd/serial_plotter>。网页侧尚未实接设备验证，JSON字段与作者配置格式对应。

## Keil在线调参

使用ST-Link/J-Link的SWD，观察波形时保持CPU运行，不暂停带力控制。

Watch添加：

```c
dm_gyro_online
dm_gyro_age_ms
dm_fault_latched
big_follow_kp
big_follow_sign
big_follow_deadband
big_follow_speed_limit
big_follow_current_limit
big_speed_test_enable
big_speed_test_set
gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid
```

默认外环Kp=2、死区0.02618rad(约1.5度)、速度上限0.5rad/s、电流上限3000，均为初始试调值。参数校验：Kp允许0~20、死区0~0.5rad、速度上限0~5rad/s、方向只能+1或-1；无效值会撤销相应目标/输出。

先零力手转核对DM轴向和符号，再调速度内环。普通速度PID在 `gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid`：Kp/Ki/Kd，开始Ki=Kd=0，从较低Kp逐步试调，注意CH3是否饱和。直接修改初始化数组不会更新已经初始化的PID结构。

速度测试仅在普通遥控模式运行，不绕过遥控器、DM和初始化保护：进入普通模式后先设置 `big_speed_test_set=0.1`，再置 `big_speed_test_enable=1`。脉冲最多2秒，此后仍保持测试模式但目标为0。再次测试先置enable=0让控制任务执行一次，再设置新目标并置enable=1。进入零力或故障时自动清除测试。

速度环稳定后，置 `big_speed_test_set=0`、`big_speed_test_enable=0`，调外环：回中慢增加Kp，左右摆动降低Kp，中位频繁微动适度增大死区。偏角越来越大先检查big_follow_sign，不要同时翻转电机和DM符号。

RAM参数重启会恢复源码初值；最终把速度PID写回gimbal_task.h，外环参数写回gimbal_task.c。网页发送Text/Hex没有PID修改作用，本版忽略调试模式USB输入。

## 验证记录

使用本机 `D:\keil\UV4\UV4.exe` 和 ARMCC 5.06，对 `PROJECT` 目标执行完整重构建，两个开关分支均通过：

| 配置 | 结果 | 日志 |
|---|---|---|
| USB_YAW_DEBUG=0，原视觉/导航协议 | 0错误、70警告 | build-v10-normal.log |
| USB_YAW_DEBUG=1，最终调参固件 | 0错误、72警告 | build-v10-debug.log |

最终源码保持USB_YAW_DEBUG=1，`MDK-ARM/OBJ/PROJECT.hex`、`PROJECT.axf`均为该配置最后一次完整构建产物。Keil退出码1代表有警告，日志中没有编译/链接错误。原基准完整构建日志为172警告。

剩余警告包括原有旧式导航函数声明、CDC缓冲区宏重复、未使用函数/变量及浮点提升；调试分支还会保留未调用的原USB协议函数。未将这些警告当成已解决。

离线检查：网站JSON可正常解析且通道数为6，帧头/帧尾与固件一致；remote_control.c、usart.c、bsp_pid.c与基准SHA256一致。无需中断遥控链路。

没有连接C板、电机或DM，没有烧录、实机闭环或网站串口验证。编译通过不等于轴向、PID或机械零位已经实机验证。

必须现场验证：初始化是否能到位、两种方向符号、低速正负阶跃、DM拔线后零力且需拨杆重新使能、小Yaw回中过程枪管航向稳定、电流限幅是否适合当前负载。
