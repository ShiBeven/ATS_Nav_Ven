"""Compile and exercise the actual USB parser/TX functions with host USB stubs."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import struct

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
VC = Path(r"C:\Program Files (x86)\Microsoft Visual Studio\VC98")
source = (ROOT / "application/usb_task.c").read_text(encoding="utf-8")
header = (ROOT / "application/usb_task.h").read_text(encoding="utf-8")


def decode_tool_output(data):
    try:
        return data.decode("mbcs", errors="replace")
    except LookupError:
        return data.decode("utf-8", errors="replace")


def uncomment(text):
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)


clean = uncomment(source)


def function(name):
    match = re.search(r"^(?:static\s+)?[\w* ]+\b" + name + r"\([^;]*?\)\s*\{", clean, re.M)
    if not match:
        raise AssertionError("Function missing: " + name)
    start = clean.index("{", match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (clean[end] == "{") - (clean[end] == "}")
        end += 1
    return clean[match.start():end]


def packed_type(name):
    match = re.search(r"typedef\s+__packed\s+struct\s*\{[^}]*\}\s*" + name + r";", uncomment(header), re.S)
    assert match, name
    return match.group().replace("__packed", "")


prefix = r'''
#include <stdio.h>
#include <string.h>
#include <math.h>
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
#define USB_YAW_DEBUG 0
#define EXPECTED_INPUT_VISION_HEDER 0xA5
#define SET_OUTPUT_VISION_HEDER 0x5A
#define ATS_GCC_NAVIGATION_FRAME_SIZE 54U
#define ATS_GCC_NAVIGATION_PAYLOAD_LEN 48U
#define USB_ATS_GCC_GIMBAL_MODE_FALLBACK 1
#define CRC16_INIT 0xFFFF
#define USBD_STATE_CONFIGURED 3
#define INS_GYRO_Z_ADDRESS_OFFSET 2
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2
#pragma pack(push, 1)
'''
prefix += packed_type("VisionRecvData_t") + "\n" + packed_type("Vision_send_t")
prefix += r'''
typedef struct { uint8_t sof, len, id, crc; } UnifiedFrameHeader_t;
typedef struct {
  UnifiedFrameHeader_t frame_header;
  uint32_t time_stamp;
  struct {
    struct { float vx, vy, wz; uint8_t stop, mode; } speed_vector;
    struct { float roll, pitch, yaw, leg_lenth; } chassis;
    struct { float pitch, yaw; } gimbal;
    struct { uint8_t fire, fric_on; } shoot;
    struct { uint8_t tracking; } tracking;
    struct { uint8_t custom_flag; } custom_flag;
  } data;
  uint16_t checksum;
} ReceiveDataRobotCmd_s;
typedef struct { float vx, vy, wz; } ReceivedPacketTwist_s;
#pragma pack(pop)
typedef struct { uint32_t TxState; } USBD_CDC_HandleTypeDef;
typedef struct { int dev_state; void *pClassData; } USBD_HandleTypeDef;
static USBD_HandleTypeDef hUsbDeviceFS;
static USBD_CDC_HandleTypeDef cdc;
static volatile uint8_t rx_ring[512];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_overflow;
static uint32_t nav_last_rx, vision_last_rx, tick, apply_count, send_count;
static uint8_t nav_override, Follow_Vision;
static VisionRecvData_t vision_cmd, ReceivedPacketVision;
static Vision_send_t SendPacketVision;
static uint8_t state_tx[49];
static uint8_t ats_imu_state_tx[34];
static uint8_t ats_joint_state_tx[22];
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceivedPacketTwist_s Navigation_speed_set;
static float INS_quat[4], INS_gyro[3], angles[3];
static struct { float launching_frequency; } Refree1_data;
static const float *gimbal_INT_gyro_angle_point;
static uint8_t *submitted;
static uint16_t submitted_length;
static uint32_t HAL_GetTick(void) { return tick; }
static void Update_to_resolve(void) { ++apply_count; }
static const float *get_INS_angle_point(void) { return angles; }
static uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length) {
  submitted = data; submitted_length = length; ++send_count; cdc.TxState = 1;
  return 0;
}
typedef struct { uint16_t checksum; } SendPacketAllRobotHP_s;
typedef struct { uint16_t checksum; } SendPacketGameStatus_s;
typedef struct { uint16_t checksum; } SendPacketRobotStatus_s;
typedef struct { uint16_t checksum; } OutputPCData_s;
'''
table = re.search(r"(?:static\s+)?const\s+uint16_t\s+W_CRC_TABLE\[256\]\s*=\s*\{.*?\};", clean, re.S)
assert table, "CRC table missing"
parts = [prefix, table.group()]
for name in ("Get_CRC16_Check_Sum", "Verify_CRC16_Check_Sum", "Append_CRC16_Check_Sum",
             "unified_usb_receive_bytes", "unified_crc8", "unified_nav_apply_ats_gcc_54",
             "unified_nav_apply_50",
             "usb_apply_vision_command", "unified_usb_frame", "unified_usb_process", "usb_send_vision",
             "unified_usb_send_state", "unified_usb_send_imu_state", "unified_usb_send_joint_state"):
    parts.append(function(name))

tests = r'''
static int checks;
#define CHECK(x) do { ++checks; if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); return 1; } } while (0)
static void checksum(uint8_t *data, int length) {
  uint16_t crc = Get_CRC16_Check_Sum(data, length - 2, CRC16_INIT);
  data[length - 2] = (uint8_t)crc; data[length - 1] = (uint8_t)(crc >> 8);
}
static void legacy(uint8_t *out, uint8_t mode, float yaw) {
  VisionRecvData_t cmd;
  memset(&cmd, 0, sizeof(cmd)); cmd.head = 0xA5; cmd.mode = mode;
  cmd.yaw = yaw; cmd.yaw_vel = 2; cmd.yaw_acc = 3;
  cmd.pitch = 0.125f; cmd.pitch_vel = 5; cmd.pitch_acc = 6;
  memcpy(out, &cmd, sizeof(cmd)); checksum(out, sizeof(cmd));
}
static void unified(uint8_t *out, uint8_t id, int length) {
  memset(out, 0, length); out[0] = 0x5A; out[1] = (uint8_t)(length - 6); out[2] = id;
  out[3] = unified_crc8(out, 3); checksum(out, length);
}
static void unified54(uint8_t *out, uint8_t stop, uint8_t mode, uint8_t fire,
                      uint8_t fric_on, uint8_t tracking) {
  float vx = 0.75f, vy = -0.25f, wz = 1.5f;
  float roll = 0.1f, pitch = 0.2f, chassis_yaw = -0.3f, leg_length = 0.4f;
  float gimbal_pitch = 0.35f, gimbal_yaw = -0.45f;
  memset(out, 0, 54); out[0] = 0x5A; out[1] = 48; out[2] = 1;
  out[3] = unified_crc8(out, 3);
  out[4] = 0x04; out[5] = 0x03; out[6] = 0x02; out[7] = 0x01;
  memcpy(out + 8, &vx, 4); memcpy(out + 12, &vy, 4); memcpy(out + 16, &wz, 4);
  out[20] = stop; out[21] = mode;
  memcpy(out + 25, &roll, 4); memcpy(out + 29, &pitch, 4);
  memcpy(out + 33, &chassis_yaw, 4); memcpy(out + 37, &leg_length, 4);
  memcpy(out + 41, &gimbal_pitch, 4); memcpy(out + 45, &gimbal_yaw, 4);
  out[49] = fire; out[50] = fric_on; out[51] = tracking;
  checksum(out, 54);
}
static void feed(const uint8_t *data, int length) {
  unified_usb_receive_bytes(data, length); unified_usb_process();
}
int main(void) {
  uint8_t one[28], two[28], bad[28], new_vision[35], nav50[50], nav52[52], nav54[54], nav54_attack[54], nav54_stop[54], stream[128], saved[42], noise[64];
  uint32_t before;
  int split, i;
  float vx = 0.75f;
  float field;
  uint8_t state_saved[49];
  FILE *capture;
  CHECK(sizeof(VisionRecvData_t) == 28 && sizeof(Vision_send_t) == 42);
  CHECK(sizeof(ReceiveDataRobotCmd_s) == 52);
  legacy(one, 1, 0.25f); legacy(two, 2, -0.5f);
  for (split = 0; split <= 28; ++split) {
    before = apply_count; ++tick;
    feed(one, split); feed(one + split, 28 - split);
    CHECK(apply_count == before + 1 && vision_last_rx == tick);
    CHECK(ReceivedPacketVision.head == 0xA5 && ReceivedPacketVision.mode == 1);
    CHECK(ReceivedPacketVision.yaw == 0.25f && ReceivedPacketVision.pitch_acc == 6);
    CHECK(Verify_CRC16_Check_Sum((uint8_t *)&ReceivedPacketVision, 28));
  }
  memcpy(stream, one, 28); memcpy(stream + 28, two, 28); before = apply_count;
  feed(stream, 56); CHECK(apply_count == before + 2 && ReceivedPacketVision.yaw == -0.5f);
  memcpy(bad, one, 28); bad[5] ^= 0x01; before = apply_count;
  feed(bad, 28); CHECK(apply_count == before);
  feed(two, 28); CHECK(apply_count == before + 1);
  legacy(bad, 3, 0.75f); before = apply_count;
  feed(bad, 28); CHECK(apply_count == before);
  feed(one, 28); CHECK(apply_count == before + 1);
  memset(noise, 0xFF, sizeof(noise));
  for (i = 0; i < 40; ++i) feed(noise, sizeof(noise));
  feed(two, 28); CHECK(ReceivedPacketVision.yaw == -0.5f);
  stream[0] = 0x5A; stream[1] = 0xFF; stream[2] = 0; stream[3] = 0;
  memcpy(stream + 4, one, 28); feed(stream, 32);
  CHECK(ReceivedPacketVision.yaw == 0.25f);
  unified(new_vision, 2, 35); new_vision[8] = 2; memcpy(new_vision + 9, two + 2, 24); checksum(new_vision, 35);
  for (split = 0; split <= 35; ++split) {
    before = apply_count; feed(new_vision, split); feed(new_vision + split, 35 - split);
    CHECK(apply_count == before + 1 && ReceivedPacketVision.mode == 2);
  }
  unified(nav50, 1, 50); memcpy(nav50 + 8, &vx, 4); checksum(nav50, 50);
  for (split = 0; split <= 50; ++split) {
    ++tick; feed(nav50, split); feed(nav50 + split, 50 - split);
    CHECK(Navigation_speed_set.vx == vx && nav_last_rx == tick);
  }
  unified(nav52, 1, 52); memcpy(nav52 + 8, &vx, 4); nav52[48] = 1; checksum(nav52, 52);
  for (split = 0; split <= 52; ++split) {
    ++tick; feed(nav52, split); feed(nav52 + split, 52 - split);
    CHECK(nav_override == 1 && Navigation_speed_set.vx == vx && nav_last_rx == tick);
  }
  unified54(nav54, 0, 2, 1, 1, 1);
  for (split = 0; split <= 54; ++split) {
    ++tick; feed(nav54, split); feed(nav54 + split, 54 - split);
    CHECK(nav_last_rx == tick && Navigation_speed_set.vx == 0.75f);
    CHECK(Navigation_speed_set.vy == -0.25f && Navigation_speed_set.wz == 1.5f);
    CHECK(nav_override == 1 && RECEIVE_ROBOT_CMD_DATA.data.speed_vector.mode == 2);
    CHECK(ReceivedPacketVision.mode == 2 && ReceivedPacketVision.pitch == 0.35f);
    CHECK(ReceivedPacketVision.yaw == -0.45f);
  }
  memcpy(stream, nav54, 54); memcpy(stream + 54, nav50, 50); before = nav_last_rx;
  ++tick; feed(stream, 104);
  CHECK(nav_last_rx == tick && Navigation_speed_set.vx == vx);
  unified54(nav54_attack, 0, 1, 0, 1, 0);
  ++tick; feed(nav54_attack, 54);
  CHECK(nav_last_rx == tick && nav_override == 1 && ReceivedPacketVision.mode == 1);
  CHECK(ReceivedPacketVision.pitch == 0.35f && ReceivedPacketVision.yaw == -0.45f);
  memcpy(nav54_stop, nav54, 54); nav54_stop[20] = 1; nav54_stop[49] = 0; nav54_stop[51] = 0;
  nav54_stop[3] = unified_crc8(nav54_stop, 3); checksum(nav54_stop, 54);
  ++tick; feed(nav54_stop, 54);
  CHECK(nav_last_rx == tick && Navigation_speed_set.vx == 0 && Navigation_speed_set.vy == 0);
  CHECK(nav_override == 0);
  memcpy(bad, nav54, 28); /* Keep the existing short bad-frame test below independent. */
  nav54[10] ^= 0x01; before = nav_last_rx; ++tick; feed(nav54, 54);
  CHECK(nav_last_rx == before);
  nav54[10] ^= 0x01;
  before = apply_count; ++tick; feed(two, 28);
  CHECK(vision_last_rx == tick && vision_cmd.yaw == -0.5f && apply_count == before + 1);
  nav52[48] = 0; checksum(nav52, 52); feed(nav52, 52);
  CHECK(nav_override == 0 && ReceivedPacketVision.yaw == -0.5f);
  memcpy(stream, one, 28); memcpy(stream + 28, nav50, 50); memcpy(stream + 78, two, 28);
  feed(stream, 106); CHECK(ReceivedPacketVision.yaw == -0.5f && Navigation_speed_set.vx == vx);
  unified(stream, 0x7F, 64); feed(stream, 64); feed(one, 28);
  CHECK(ReceivedPacketVision.yaw == 0.25f);
  hUsbDeviceFS.dev_state = 0; hUsbDeviceFS.pClassData = &cdc;
  usb_send_vision(); CHECK(send_count == 0);
  hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED; hUsbDeviceFS.pClassData = NULL;
  usb_send_vision(); CHECK(send_count == 0);
  hUsbDeviceFS.pClassData = &cdc; cdc.TxState = 1;
  usb_send_vision(); CHECK(send_count == 0);
  cdc.TxState = 0; angles[0] = 0.5f; angles[1] = -0.25f; Follow_Vision = 1;
  INS_quat[0] = 1; Refree1_data.launching_frequency = 22;
  usb_send_vision(); CHECK(send_count == 1 && submitted_length == 42 && submitted[0] == 0x5A);
  CHECK(Verify_CRC16_Check_Sum(submitted, submitted_length));
  CHECK(fabs(SendPacketVision.yaw - 28.6478898f) < 0.0001);
  CHECK(SendPacketVision.mode == 1 && SendPacketVision.q[0] == 1 && SendPacketVision.bullet_speed == 22);
  memcpy(saved, submitted, 42); angles[0] = 1.0f; usb_send_vision();
  CHECK(send_count == 1 && memcmp(saved, submitted, 42) == 0);
  send_count = 0;
  hUsbDeviceFS.dev_state = 0; hUsbDeviceFS.pClassData = NULL; cdc.TxState = 0;
  unified_usb_send_state(); CHECK(send_count == 0);
  hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED; hUsbDeviceFS.pClassData = NULL;
  unified_usb_send_state(); CHECK(send_count == 0);
  hUsbDeviceFS.pClassData = &cdc; cdc.TxState = 1;
  unified_usb_send_state(); CHECK(send_count == 0);
  cdc.TxState = 0; nav_override = 0; Follow_Vision = 1; tick = 0x12345678;
  angles[0] = 0.5f; angles[1] = -0.25f;
  INS_gyro[0] = 3.0f; INS_gyro[1] = 1.25f; INS_gyro[2] = -0.75f;
  unified_usb_send_state();
  CHECK(send_count == 1 && submitted_length == 49);
  CHECK(submitted[0] == 0x5A && submitted[1] == 43 && submitted[2] == 0x0E);
  CHECK(submitted[3] == unified_crc8(submitted, 3));
  CHECK(Verify_CRC16_Check_Sum(submitted, 49));
  CHECK(submitted[4] == 0x78 && submitted[5] == 0x56 && submitted[6] == 0x34 && submitted[7] == 0x12);
  CHECK(submitted[8] == 1);
  memcpy(&field, submitted + 9, 4); CHECK(field == 1.0f);
  for (i = 13; i < 25; ++i) CHECK(submitted[i] == 0);
  memcpy(&field, submitted + 25, 4); CHECK(field == 0.5f);
  memcpy(&field, submitted + 29, 4); CHECK(field == 3.0f);
  memcpy(&field, submitted + 33, 4); CHECK(field == -0.25f);
  memcpy(&field, submitted + 37, 4); CHECK(field == 1.25f);
  memcpy(&field, submitted + 41, 4); CHECK(field == 22.0f);
  CHECK(submitted[45] == 0 && submitted[46] == 0);
  capture = fopen("state-frame.bin", "wb"); CHECK(capture != NULL);
  CHECK(fwrite(submitted, 1, 49, capture) == 49); fclose(capture);
  memcpy(state_saved, submitted, 49); angles[0] = 1.0f; unified_usb_send_state();
  CHECK(send_count == 1 && memcmp(state_saved, submitted, 49) == 0);
  send_count = 0; cdc.TxState = 0; tick = 0x01020304;
  angles[0] = 0.5f; angles[1] = -0.25f; angles[2] = 0.125f;
  INS_gyro[0] = 3.0f; INS_gyro[1] = 1.25f; INS_gyro[2] = -0.75f;
  unified_usb_send_imu_state();
  CHECK(send_count == 1 && submitted_length == 34);
  CHECK(submitted[0] == 0x5A && submitted[1] == 28 && submitted[2] == 0x02);
  CHECK(submitted[3] == unified_crc8(submitted, 3) && Verify_CRC16_Check_Sum(submitted, 34));
  memcpy(&field, submitted + 8, 4); CHECK(field == 0.5f);
  memcpy(&field, submitted + 12, 4); CHECK(field == -0.25f);
  memcpy(&field, submitted + 16, 4); CHECK(field == 0.125f);
  memcpy(&field, submitted + 20, 4); CHECK(field == -0.75f);
  memcpy(&field, submitted + 24, 4); CHECK(field == 1.25f);
  memcpy(&field, submitted + 28, 4); CHECK(field == 3.0f);
  cdc.TxState = 0;
  unified_usb_send_joint_state();
  CHECK(send_count == 2 && submitted_length == 22);
  CHECK(submitted[0] == 0x5A && submitted[1] == 16 && submitted[2] == 0x0C);
  CHECK(submitted[3] == unified_crc8(submitted, 3) && Verify_CRC16_Check_Sum(submitted, 22));
  memcpy(&field, submitted + 8, 4); CHECK(field == 0.5f);
  memcpy(&field, submitted + 12, 4); CHECK(field == 0.5f);
  memcpy(&field, submitted + 16, 4); CHECK(field == -0.25f);
  printf("PASS %d checks: actual RX parser, legacy compatibility, navigation and unified 49-byte state, units, axes, CRC, busy guards\n", checks);
  return 0;
}
'''
host = OUT / "usb_protocol_host_test.cpp"
host.write_text("\n".join(parts) + tests, encoding="ascii")
env = os.environ.copy()
env["PATH"] = str(VC / "Bin") + os.pathsep + str(VC.parent / "Common/MSDev98/Bin") + os.pathsep + env.get("PATH", "")
env["INCLUDE"] = str(VC / "Include")
env["LIB"] = str(VC / "Lib")
exe = OUT / "usb_protocol_host_test.exe"
cl = VC / "Bin/CL.EXE"
if cl.exists():
    build_command = [str(cl), "/nologo", "/TP", "/MLd", "/W3", str(host), "/Fe" + str(exe)]
else:
    # The firmware is built with Keil/ARMCC, but the parser test is also
    # intentionally runnable on the Linux development machine.  Use the
    # available host C++ compiler when the historical VC98 toolchain is not
    # installed; the generated source contains only portable test stubs.
    host_cxx = shutil.which("g++") or shutil.which("c++")
    if not host_cxx:
        raise SystemExit("Neither VC98 CL.EXE nor a host C++ compiler was found")
    build_command = [host_cxx, "-std=c++11", "-Wall", "-Wextra", str(host), "-o", str(exe)]
build = subprocess.run(build_command, cwd=OUT, env=env, capture_output=True)
log = decode_tool_output(build.stdout) + decode_tool_output(build.stderr)
(OUT / "host-build.log").write_text(log, encoding="utf-8")
if build.returncode:
    raise SystemExit(log)
run = subprocess.run([str(exe)], cwd=OUT, capture_output=True)
result = decode_tool_output(run.stdout) + decode_tool_output(run.stderr)
print(result.strip())
if run.returncode:
    raise SystemExit(run.returncode)

backup = ROOT / "排查记录/20261029-按视觉接口文档对齐-修改前"
original = (backup / "usb_task.c").read_text(encoding="utf-8")
start = "    if ((uint32_t)(now - last_chassis) >= 10U) {"
end = "    if ((uint32_t)(now - last_state) >= 5U) {"
assert source[source.index(start):source.index(end)] == original[original.index(start):original.index(end)], "Timeout/CAN forwarding changed"
hashes = json.loads((backup / "源码哈希.json").read_text(encoding="utf-8-sig"))
for item in hashes:
    path = Path(item["Path"])
    if not path.exists():
        # The baseline report was produced on the Windows checkout.  Resolve
        # its repository-relative suffix when this verifier runs on Linux.
        windows_path = item["Path"].replace("\\", "/")
        marker = "/双板通信的云台/"
        if marker in windows_path:
            path = ROOT / windows_path.split(marker, 1)[1]
    if path.name not in ("usb_task.c", "usb_task.h"):
        assert hashlib.sha256(path.read_bytes()).hexdigest().upper() == item["Hash"], str(path) + " changed"
report = {"host_test": result.strip(), "timeout_and_can_forwarding_unchanged": True,
          "non_usb_sources_unchanged": True, "hardware_tested": False,
          "usb_source_sha256": hashlib.sha256((ROOT / "application/usb_task.c").read_bytes()).hexdigest()}

# Independent bitwise reference and document-defined field offsets; do not use
# the firmware CRC table or serialization routine to construct the expectation.
def reference_crc(data, initial, poly, width):
    crc = initial
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (poly if crc & 1 else 0)
    return crc & ((1 << width) - 1)


prefix = bytes([0x5A, 43, 0x0E])
packet = prefix + bytes([reference_crc(prefix, 0xFF, 0x8C, 8)])
packet += struct.pack("<IB4f5fH", 0x12345678, 1, 1.0, 0.0, 0.0, 0.0,
                      0.5, 3.0, -0.25, 1.25, 22.0, 0)
packet += struct.pack("<H", reference_crc(packet, 0xFFFF, 0x8408, 16))
assert len(packet) == 49
assert packet == (OUT / "state-frame.bin").read_bytes(), "Wire reply differs from independent reference"
assert re.search(r"^#define USB_YAW_DEBUG\s+0\s*$", header, re.M)
assert re.search(r"^#define USB_LEGACY_VISION_TX\s+1\s*$", header, re.M)
assert re.search(r"^#define USB_ATS_COMPAT_FEEDBACK_TX\s+1\s*$", header, re.M)
assert "enum mode { Move = 0, Attack = 1, Defend = 2 }" in header
report["independent_49_byte_reference"] = packet.hex(" ")
report["legacy_vision_tx_preserved"] = True
report["ats_compat_feedback_tx_selected"] = True
report["legacy_chassis_mode_enum_preserved"] = True
print("PASS: independent 49-byte wire reference, legacy TX preserved, additive ATS feedback and chassis modes")
(OUT / "验证结果.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
print("PASS: existing timeout/CAN forwarding and non-USB source hashes unchanged")
