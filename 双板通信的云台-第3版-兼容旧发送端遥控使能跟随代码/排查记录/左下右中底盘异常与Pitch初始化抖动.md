# 左下右中底盘异常与Pitch初始化抖动

2026-10-22，用户确认底盘异常时左下右中，初始化抖动疑似Pitch，尚未完全辨清轴。

## 云台已完成的最小修复

当前工程：D:/code-ai/双板通信的云台。Gimbal_Init原来只同步两个Yaw的相对目标，Pitch保留旧目标。在校准改变编码器零位或从端点切入时，旧目标造成瞬时跳变。

现Pitch从非ENCODE进入初始化时，先将relative_angle_set同步为当前relative_angle，再执行原有回零增量。后续INIT周期继续累加，再次进入时重新同步。本次只改gimbal_task.c的该入口，不修改普通PID、初始化增量/判停、Flash、CAN和底盘参数。

19组实际C函数解释检查通过，包含4种旧目标/新编码器角情形、连续INIT推进和退出后再次进入；此测试把PID替换为桩，只验证目标连续性，不验证实际机械稳定性。Keil增量0错误、3警告。未烧录。

## 底盘代码核查

当前底盘：C:/Users/cuining/Desktop/通信版/底盘-第2版。

左下右中选择CHASSIS_FOLLOW_CHASSIS_YAW，枚举值3。ChassisFollowChassis不读取大Yaw角度，调用的运动学/轮速控制也不依赖云台零位；因此该模式没有“小Yaw限位校准零点直接改变底盘运动方向”的计算路径。

已确认的配置疑点（尚未证明为现场根因）：

- 遥控基本帧超过100ms会清ch0/1/2并把拨杆置双下，导致模式退出。应先看接收年龄是否跨过100ms。
- 斜坡frame_period初始化0.1秒，循环osDelay(1)，若FreeRTOS为1ms tick则实际步长差约100倍。加减速参数0.7导致每轮目标变化0.07，明显快于按真实周期计算的斜坡。
- 遥控系数0.02，标准约±660满杆输入对应±13.2，斜坡输出限制±10；当前量纲按m/s继续换算电机RPM。旋转斜坡输出另乘10。这些是旧版已有配置，必须结合实际输入、速度请求、反馈判断，不能直接称本轮校准引入。
- 舵轮零位/顺序为FR360、FL3098、BL315、BR1437（NEW_CHASSIS=1）；是否匹配现场机械未验证。

## 下一组运行证据

请在底盘板的调试会话中添加：

```
ChassisData.ChassisMode
RCData.rc.ch
uwTick - rc_basic_last_tick
ChassisData.VxSet
ChassisData.Vyset
ChassisData.Wzset
```

左下右中且通信正常时mode应为3。若mode在0/3之间变化或接收年龄反复超过100ms，优先沿合一云台C板本地DBUS→CAN0x111→底盘接收检查；若mode3、通道连续但速度请求过大，核对遥控比例/斜坡；若请求正常而轮组动作不符，再看MotorRudder、WheelAngle、MotorWheel和speed_set。

没有独立导航板，接收机在合一云台C板。本轮没有修改或编译底盘工程；尚未抓取实际CAN或确认运行根因。
