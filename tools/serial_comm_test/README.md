# 电控串口固定值联调

这个目录用于和电控单独验证上下位机串口协议，不启动 ROS，也不改正式节点。

## 运行前提

- 主机与电控通过实际串口连接。
- 先停止 `standard_robot_pp_ros2_node`，一个串口不能同时被两个程序占用。
- 主机安装 `pyserial`：

```bash
python3 -m pip install --user pyserial
```

## 测试命令

默认使用 `/dev/ttyACM0`、921600 波特率，持续 10 秒，每 100 ms 发送一帧：

```bash
python3 tools/serial_comm_test/serial_comm_test.py
```

固定发送值为：`vx=0.10, vy=0.20, wz=0.30, pitch=0.40, yaw=0.50`。
也可以显式指定串口和时长：

```bash
python3 tools/serial_comm_test/serial_comm_test.py \
  --port /dev/ttyACM1 --duration 30 --vx 0 --vy 0 --wz 0
```

## 和电控一起判定

1. 电控端确认收到 `sof=0x5A`、`id=0x01` 的 50 字节帧。
2. 电控端检查 CRC8 和 CRC16，并打印/回传收到的 `0.10, 0.20, 0.30, 0.40, 0.50`。
3. 电控端按现有协议回传任意有效帧（例如 IMU、机器人状态或云台状态）。
4. 主机看到 `接收有效帧` 增加且没有 CRC 错误，即可确认回传方向也正常。

注意：程序统计的是协议层有效帧，不是“串口 write 调用成功”。如果只有发送计数而没有电控回传，不能判定双向通信已经打通。
