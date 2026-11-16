# Pitch下一步：无力反馈与Yaw投影检查（2026-10-23）

## 当前前提

用户最后确认小Yaw通电、实际发零时Pitch稳定；保持该诊断状态，不恢复小Yaw正常PID出力。本轮不修改控制源码、PID、初始化状态、Flash或Watch变量。

最新截图108aae4e：gimbal_behaviour1 INIT，gimbal_cali_status4、valid1、error0，DM在线1/fault0；机械校准已加载，不再按早前valid0/offset0处理。保存的PROJECT.uvoptx Watch表达式确认-2.90039182对应小Yawrelative_angle*57.2958，是度而非大Yaw角。小Yaw初始化容差0.02rad约1.1459度，发零时不能主动将-2.9度回中。

## 本轮操作

1. 两个拨杆都下，摇杆回中。保持电机驱动、C板和外置IMU供电，确认gimbal_behaviour0且三轴软件命令yaw_can_set_current、pitch_can_set_current、yaw2_can_set_current均0，再手动操作；不要拔掉Pitch驱动电源，否则编码器不更新。
2. 支撑枪管避免重力跌落，在中间安全行程小幅、缓慢上下俯仰，避开上下硬限位，不触发机械校准。
3. 现有20路记录全部通道。静止约3秒，缓慢上抬2秒，停2秒，缓慢回落2秒，停3秒，约12秒。重点显示4/5（Pitch相对/绝对角）、7（Pitch gyroY）、12（小Yaw投影速度）；同时检查2模式0、8/13/18软件命令0、10与15大小Yaw编码器。
4. 看角度是否同向、连续，停下后gyro是否回近0；两角零点可以略有差异，不能要求完全相等。只在大小Yaw编码器基本不变且机构没有真实Yaw转动的片段，检查小Yaw投影速度是否随纯Pitch运动明显异常；不能把手动带出的真实Yaw运动误判为串轴。
5. 保存全20路CSV，并记录近水平的Pitch relative_angle/absolute_angle/offset_ecd/valid。当前硬编码CAN_cmd_gimbal_3(0)，模式3的第13路可能是未真正发送的PID计算变量；本轮mode0时该变量应为0。

## 带力测试之前的状态机问题

当前不依赖三轴原INIT做Pitch单轴带力测试，也不手改behaviour3或valid1绕过门控。已独立解释执行实际GetgimbalMode并保留静态局部变量，条件为双中/有效校准/IMU与RC正常/小Yaw固定-2.9004度，其模式轨迹为：第1次1，第6001次0，第6002次3。源码超时return未同步函数尾部static last_gimbal_behaviour，下一轮可能绕过重新INIT而进ABS。该结果是逻辑复现，不是硬件采样；旧验证器默认每次重建局部变量，不能拿旧单次调用用例覆盖静态状态机时序。

因此应先修正初始化失败/超时后的明确锁存与重新使能流程，再设计Yaw出力被隔离的Pitch单轴带力测试。原三轴INIT还要求两根Yaw都回中，与当前小Yaw强制发零的隔离条件不匹配。
