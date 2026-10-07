#!/usr/bin/env python3
"""Fixed-value test for the StandardRobot++ serial protocol.

This utility deliberately does not depend on ROS. Stop the normal serial node
before running it because a serial device must have one owner at a time.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from collections import Counter


SOF = 0x5A
TX_ID_ROBOT_CMD = 0x01

CRC8_TABLE = (
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20,
    0xA3, 0xFD, 0x1F, 0x41, 0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC, 0x23, 0x7D, 0x9F, 0xC1,
    0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E,
    0x1D, 0x43, 0xA1, 0xFF, 0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07, 0xDB, 0x85, 0x67, 0x39,
    0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45,
    0xC6, 0x98, 0x7A, 0x24, 0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9, 0x8C, 0xD2, 0x30, 0x6E,
    0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31,
    0xB2, 0xEC, 0x0E, 0x50, 0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE, 0x32, 0x6C, 0x8E, 0xD0,
    0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA,
    0x69, 0x37, 0xD5, 0x8B, 0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16, 0xE9, 0xB7, 0x55, 0x0B,
    0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54,
    0xD7, 0x89, 0x6B, 0x35,
)

# Same reflected CRC16 table used by standard_robot_pp_ros2.
CRC16_POLY = 0x8408


def crc8(data: bytes) -> int:
    value = 0xFF
    for byte in data:
        value = CRC8_TABLE[value ^ byte]
    return value


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ CRC16_POLY if value & 1 else value >> 1
    return value


def make_command(timestamp_ms: int, values: tuple[float, ...]) -> bytes:
    vx, vy, wz, pitch, yaw = values
    # Header + timestamp + SendRobotCmdData.data + CRC16, packed as little endian.
    payload = struct.pack(
        "<Ifff?ffffffBB?",
        timestamp_ms,
        vx,
        vy,
        wz,
        False,
        0.0,
        0.0,
        0.0,
        0.0,
        pitch,
        yaw,
        0,
        0,
        False,
    )
    header = bytearray((SOF, len(payload), TX_ID_ROBOT_CMD, 0))
    header[3] = crc8(header[:3])
    frame = header + payload + b"\x00\x00"
    checksum = crc16(frame[:-2])
    frame[-2:] = struct.pack("<H", checksum)
    return bytes(frame)


def read_frame(port) -> tuple[int, bytes] | None:
    while True:
        first = port.read(1)
        if not first:
            return None
        if first[0] != SOF:
            continue
        header_tail = port.read(3)
        if len(header_tail) != 3:
            return None
        header = first + header_tail
        if crc8(header[:3]) != header[3]:
            continue
        body = port.read(header[1] + 2)
        if len(body) != header[1] + 2:
            return None
        frame = header + body
        if crc16(frame[:-2]) != struct.unpack_from("<H", frame, len(frame) - 2)[0]:
            raise ValueError(f"CRC16 error for id=0x{header[2]:02X}")
        return header[2], frame


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--vx", type=float, default=0.10)
    parser.add_argument("--vy", type=float, default=0.20)
    parser.add_argument("--wz", type=float, default=0.30)
    parser.add_argument("--pitch", type=float, default=0.40)
    parser.add_argument("--yaw", type=float, default=0.50)
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("缺少 pyserial，请执行: python3 -m pip install --user pyserial", file=sys.stderr)
        return 2

    values = (args.vx, args.vy, args.wz, args.pitch, args.yaw)
    counts: Counter[int] = Counter()
    tx_count = 0
    rx_count = 0
    deadline = time.monotonic() + args.duration
    next_tx = time.monotonic()
    print(f"打开 {args.port} @ {args.baud}，固定值 {values}，测试 {args.duration:g}s")
    try:
        with serial.Serial(args.port, args.baud, timeout=0.01) as port:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_tx:
                    port.write(make_command(int(time.time() * 1000) & 0xFFFFFFFF, values))
                    tx_count += 1
                    next_tx += args.interval
                try:
                    received = read_frame(port)
                except ValueError as exc:
                    print(f"接收错误: {exc}")
                    continue
                if received is not None:
                    packet_id, _ = received
                    counts[packet_id] += 1
                    rx_count += 1
    except (OSError, serial.SerialException) as exc:
        print(f"串口失败: {exc}", file=sys.stderr)
        return 1

    print(f"发送帧: {tx_count}，接收有效帧: {rx_count}，接收 ID: "
          + (", ".join(f"0x{k:02X}={v}" for k, v in sorted(counts.items())) or "无") )
    print("判定：发送帧数大于 0 只能证明主机写串口成功；电控回传有效帧且 CRC 正确才证明双向通信通了。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
