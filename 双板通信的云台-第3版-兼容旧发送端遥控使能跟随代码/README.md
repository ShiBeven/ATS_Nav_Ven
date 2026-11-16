> 2026-10-22 更新：本工程已改为 Pitch + 小 Yaw 机械限位校准，大 Yaw 校准期间零命令。
> 首次烧录后旧云台参数不直接沿用，需显式重新校准。参见 [小Yaw校准修改与操作说明.md](小Yaw校准修改与操作说明.md)。
> 本次源码和原固件已备份；尚未烧录或实机验证。

# 双板通信云台：第10版大Yaw回中跟随与USB波形调试

修改基准：`C:\Users\cuining\Desktop\双板通信的云台新`。交付目录：`D:\code-ai\双板通信的云台`。两份桌面工程不修改。

使用前先阅读 [第10版修改与调参说明](第10版说明-大Yaw编码器跟随与USB调参.md)。Keil打开 `MDK-ARM/PROJECT.uvprojx`，目标为 `PROJECT`。

本版默认 `USB_YAW_DEBUG=1`：C板自身USB只上传6通道调试波形，视觉/导航USB接收关闭。遥控器接收、USART1遥控转发和CAN通信保留。网页显示波形，Keil修改RAM参数；网页Text命令不能修改PID。

DM设置：CAN、请求/被动应答模式、1Mbps、CAN_ID=0x30、MST_ID=0x31。DM安装在大Yaw输出端且位于小Yaw/Pitch之前，配置后的Z轴平行大Yaw转轴。C板仍在Pitch上方。

初次启动及DM掉线恢复时，将两拨杆均置下位，确认DM和C板IMU在线，再进入正常控制。默认大Yaw电流指令上限为3000、跟随速度上限0.5rad/s；均为试调上限，不是最终参数。

构建与实机验证结果见第10版说明。无需USB转TTL；C板USB连接电脑，ST-Link/J-Link通过SWD连接Keil，两条链路同时使用。
