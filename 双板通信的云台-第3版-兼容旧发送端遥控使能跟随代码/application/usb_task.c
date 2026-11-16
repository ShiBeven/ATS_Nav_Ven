/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      usb outputs the IMU and gimbal data to the miniPC
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-7-11       Penguin         1. done
  *  V1.0.1     Oct-31-2023     LihanChen       1. 完成核心框架的构建，使其与调试模式和 miniPC 模式兼容。
  *  V1.0.2     Nov-01-2023     LihanChen       1. 将 Append_CRC16_Check_Sum_SendPacketVision() 和 Append_CRC-16_Check_Stum_OutputPCData() 合并到 Append_CRC16_Check_Sum() 中
  *  v2.0.0     Feb-24-2024     LihanChen       1. 重构代码，将 usb_task 的发送和接收分离，并实现分包发送和接收视觉和导航数据
  *  v2.0.1     Mar-02-2024     LihanChen       1. 删除未使用的 USB_RECEIVE_STATE 定义，去除编译警告
  *  v2.1.0     Mar-04-2024     LihanChen       1. 添加裁判系统信息转发（保留接口，待填入真实数值）
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
*/

#include "usb_task.h"
#include "Vision.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "dm_imu.h"
#include "detect_task.h"
#include "arm_math.h"
#include "user_lib.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
//#include "referee.h"
// Constants
#define OUTPUT_VISION_STATE 0
#define OUTPUT_NAVITATION_STATE 0
#define OUTPUT_PC_STATE 1
#define OUTPUT_NAVITAION_STATE

#define EXPECTED_INPUT_NAVIGATION_HEDER 0x01
#define SET_OUTPUT_NAVIGATION_HEDER 0x0C
#define EXPECTED_INPUT_VISION_HEDER 0xA5
#define EXPECTED_INPUT_PC_HEDER 0xA6
#define SET_OUTPUT_VISION_HEDER 0x5A
#define SET_OUTPUT_AllRobotHP_HEDER 0x5B
#define SET_OUTPUT_GameStatus_HEDER 0x5C
#define SET_OUTPUT_RobotStatus_HEDER 0x5D

#define CRC16_INIT 0xFFFF
#define USB_STATE_INVALID 0xFF
//#define APP_RX_DATA_SIZE 40
//#define APP_TX_DATA_SIZE 28
#define USB_RECEIVE_LEN 384

// Variable Declarations
uint8_t usb_tx_buf[APP_TX_DATA_SIZE];
 uint8_t usb_rx_buf[APP_RX_DATA_SIZE];
static uint8_t usb_buf[256];
 ReceivedPacketTwist_s ReceivedPacketTwist;
 Vision_send_t SendPacketVision;
 VisionRecvData_t ReceivedPacketVision;
static SendPacketAllRobotHP_s SendPacketAllRobotHP;
static SendPacketGameStatus_s SendPacketGameStatus;
static SendPacketRobotStatus_s SendPacketRobotStatus;
const fp32 *gimbal_INT_gyro_angle_point;
static InputPCData_s InputPCData;
static OutputPCData_s OutputPCData;
static uint8_t USB_SEND_STATE;
ReceivedPacketTwist_s Navigation_speed_set;
SendNavigation_t SendPacketNavigstion;

ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static volatile uint8_t rx_ring[512];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_overflow;
static uint32_t nav_last_rx, vision_last_rx;
static uint8_t nav_override;
/* Read-only diagnostic: 1 means locally authorized legacy navigation aim. */
volatile uint8_t usb_nav_compat_active = 0U;
static uint8_t nav54_stamp_seen, nav54_compat_angles_valid;
static uint32_t nav54_sender_stamp, nav54_stamp_changed_at;
static uint8_t state_tx[49];
static uint8_t ats_imu_state_tx[34];
static uint8_t ats_joint_state_tx[22];
static uint8_t ats_feedback_tx_phase;
static VisionRecvData_t vision_cmd;
/* The legacy ARMCC navigation structure is 52 bytes, while the unified
 * protocol defines ID 0x01 as a 50-byte frame (len=44). Both are accepted. */
typedef char navigation_frame_legacy_is_52_bytes[(sizeof(ReceiveDataRobotCmd_s) == 52) ? 1 : -1];
typedef char navigation_frame_ats_gcc_is_54_bytes[
  (sizeof(AtsGccNavigationFrame54_t) == ATS_GCC_NAVIGATION_FRAME_SIZE) ? 1 : -1];
typedef char vision_frame_is_28_bytes[(sizeof(VisionRecvData_t) == 28) ? 1 : -1];
typedef char vision_reply_is_42_bytes[(sizeof(Vision_send_t) == 42) ? 1 : -1];


// Function Prototypes
static void usb_send_vision(void);
static void usb_send_navigation(void);
//static void usb_send_AllRobotHP(void);
static void usb_send_GameStatus(void);
static void usb_send_RobotStatus(void);
static void usb_send_outputPC(void);
static void usb_receive_navigation(ReceivedPacketTwist_s *ReceivedPacketTwist);
static void usb_receive_vision(VisionRecvData_t *ReceivedPacketVision);
static void usb_receive(void);
static void unified_usb_process(void);
static uint8_t usb_nav_compat_allowed(uint32_t now);
static void unified_usb_send_state(void);
static void unified_usb_send_imu_state(void);
static void unified_usb_send_joint_state(void);
static void unified_usb_send_ats_feedback(void);
static void usb_send_yaw_debug(void);
static void char_to_uint(uint8_t *word, const char *str);
static void usb_printf(const char *fmt, ...);
static uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
static uint32_t Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength);
static void Append_CRC16_Check_Sum(void *pchMessage, uint32_t dwLength, int dataType);

extern volatile float chassis_yaw_rx_rad;
extern volatile uint32_t chassis_yaw_rx_ms;
extern volatile uint8_t chassis_yaw_seen;

extern volatile uint32_t dm_euler_rx_ms;
extern volatile uint8_t dm_euler_seen;

/* Keil Watch 写 1 触发对零，采样成功后自动清零。 */
volatile uint8_t yaw_joint_zero_request = 0;
volatile uint8_t yaw_joint_zero_valid = 0;


/**
  * @brief      USB任务主函数，switch中为发送，接收部分在usb_receive()中处理
  * @param[in]  argument: 任务参数
  * @retval     None
  */
void usb_task(void const *argument)
{
  uint32_t last_state = 0;
#if !USB_YAW_DEBUG
  uint32_t last_chassis = 0;
#endif
  MX_USB_DEVICE_Init();
  while (1)
  {
    uint32_t now = HAL_GetTick();
#if USB_YAW_DEBUG
    if ((uint32_t)(now - last_state) >= 10U) {
      last_state = now;
      usb_send_yaw_debug();
    }
#else
    unified_usb_process();
    /* RX processing may advance the tick and refresh last_rx timestamps.
     * Sample now afterwards so fresh frames cannot underflow the age. */
    now = HAL_GetTick();
    /* Revoke local ownership even when no new USB frame arrives. */
    if (usb_nav_compat_active && !usb_nav_compat_allowed(now)) {
      usb_nav_compat_active = 0U;
      nav_override = 0U;
      if ((uint32_t)(now - vision_last_rx) <= 100U) {
        ReceivedPacketVision = vision_cmd;
      } else {
        ReceivedPacketVision.mode = 0;
      }
      Update_to_resolve();
    }
    if ((uint32_t)(now - last_chassis) >= 10U) {
      last_chassis = now;
      if ((uint32_t)(now - nav_last_rx) > 100U) {
        Navigation_speed_set.vx = 0;
        Navigation_speed_set.vy = 0;
        Navigation_speed_set.wz = 0;
        if (nav_override) {
          usb_nav_compat_active = 0U;
          nav_override = 0;
          if ((uint32_t)(now - vision_last_rx) <= 100U) {
            ReceivedPacketVision = vision_cmd;
          } else {
            ReceivedPacketVision.mode = 0;
          }
          Update_to_resolve();
        }
      }
      CAN_CmdBoard3(&Navigation_speed_set);
    }
    if ((uint32_t)(now - vision_last_rx) > 100U && !nav_override && ReceivedPacketVision.mode != 0) {
      ReceivedPacketVision.mode = 0;
      Update_to_resolve();
    }
    if ((uint32_t)(now - last_state) >= 5U) {
      last_state = now;
#if USB_ATS_COMPAT_FEEDBACK_TX
      unified_usb_send_ats_feedback();
#else
#if USB_LEGACY_VISION_TX
      usb_send_vision();
#else
      unified_usb_send_state();
#endif
#endif
    }
#endif
    osDelay(1);
  }
}

/* The USB task is the sole active sender in debug mode. Never overwrite a
 * buffer owned by an outstanding CDC transfer. No USB work in motor IRQs. */
static void usb_send_yaw_debug(void)
{
  static uint8_t packet[2U + GIMBAL_DEBUG_CHANNELS * sizeof(float) + 2U];
  float values[GIMBAL_DEBUG_CHANNELS];
  USBD_CDC_HandleTypeDef *cdc;
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
  cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (cdc == NULL || cdc->TxState != 0U) return;
  gimbal_get_debug_snapshot(values);
  packet[0] = 0xAA;
  packet[1] = 0x55;
  memcpy(packet + 2, values, sizeof(values));
  packet[sizeof(packet) - 2U] = 0x0D;
  packet[sizeof(packet) - 1U] = 0x0A;
  (void)CDC_Transmit_FS(packet, sizeof(packet));
}

void unified_usb_receive_bytes(const uint8_t *bytes, uint32_t length)
{
#if USB_YAW_DEBUG
  /* Plotter TX must not be interpreted as navigation, shooting or vision. */
  (void)bytes;
  (void)length;
#else
  uint32_t i;
  for (i = 0; i < length; ++i) {
    uint16_t next = (rx_head + 1U) & 511U;
    if (next == rx_tail) {
      rx_overflow = 1;
      break;
    }
    rx_ring[rx_head] = bytes[i];
    rx_head = next;
  }
#endif
}

static uint8_t unified_crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0xFF, i;
  while (length--) {
    crc ^= *data++;
    for (i = 0; i < 8; ++i)
      crc = (crc & 1U) ? (crc >> 1) ^ 0x8CU : crc >> 1;
  }
  return crc;
}

/*
 * Parse the current ATS GCC packed navigation frame without relying on the
 * ARM compiler's enum/bool ABI.  The existing 50/52-byte parsers below are
 * intentionally kept for older senders; this is an additive compatibility
 * path for ID 0x01, len=48, total length=54.
 */
/* Legacy ATS senders omit tracking. Local RC authorization accepts their
 * commanded angles; it cannot infer whether a visual target is still valid. */
static uint8_t usb_nav_compat_allowed(uint32_t now)
{
#if USB_ATS_LEGACY_RC_AIM_COMPAT
  const RC_ctrl_t *rc = get_remote_control_point();
  if (!nav54_stamp_seen || !nav54_compat_angles_valid ||
      (uint32_t)(now - nav_last_rx) > 100U ||
      (uint32_t)(now - nav54_stamp_changed_at) > 100U) return 0U;
  if (RC_data_is_error()) return 0U;
  return (switch_is_mid(rc->rc.s[RC_Left]) &&
          switch_is_down(rc->rc.s[RC_Right])) ? 1U : 0U;
#else
  (void)now;
  return 0U;
#endif
}

static void unified_nav_apply_ats_gcc_54(const uint8_t *frame, uint32_t now)
{
  float decoded[4];
  uint16_t checksum;

  memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(RECEIVE_ROBOT_CMD_DATA));

  /* Header and timestamp have the same representation on both targets. */
  memcpy((uint8_t *)&RECEIVE_ROBOT_CMD_DATA, frame, 4U);
  memcpy((uint8_t *)&RECEIVE_ROBOT_CMD_DATA + 4U, frame + 4U, 4U);

  /* ATS GCC offsets: vx=8, vy=12, wz=16, stop=20, enum mode=21..24. */
  /* ARMCC5 memcpy cannot take pointers to __packed scalar members. */
  memcpy(decoded, frame + 8U, 12U);
  RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx = decoded[0];
  RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy = decoded[1];
  RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz = decoded[2];
  RECEIVE_ROBOT_CMD_DATA.data.speed_vector.stop = frame[20U] != 0U;
  RECEIVE_ROBOT_CMD_DATA.data.speed_vector.mode = (enum mode)frame[21U];

  /* The four-byte GCC enum shifts the remaining fields by three bytes. */
  memcpy(decoded, frame + 25U, 16U);
  RECEIVE_ROBOT_CMD_DATA.data.chassis.roll = decoded[0];
  RECEIVE_ROBOT_CMD_DATA.data.chassis.pitch = decoded[1];
  RECEIVE_ROBOT_CMD_DATA.data.chassis.yaw = decoded[2];
  RECEIVE_ROBOT_CMD_DATA.data.chassis.leg_lenth = decoded[3];
  memcpy(decoded, frame + 41U, 8U);
  RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch = decoded[0];
  RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw = decoded[1];
  RECEIVE_ROBOT_CMD_DATA.data.shoot.fire = frame[49U];
  RECEIVE_ROBOT_CMD_DATA.data.shoot.fric_on = frame[50U];
  RECEIVE_ROBOT_CMD_DATA.data.tracking.tracking = frame[51U] != 0U;
  RECEIVE_ROBOT_CMD_DATA.data.custom_flag.custom_flag = 0U;
  memcpy(&checksum, frame + 52U, 2U);
  RECEIVE_ROBOT_CMD_DATA.checksum = checksum;

  nav_last_rx = now;
  Navigation_speed_set.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.stop ?
    0 : RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
  Navigation_speed_set.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.stop ?
    0 : RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
  Navigation_speed_set.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;

  if (!nav54_stamp_seen ||
      nav54_sender_stamp != RECEIVE_ROBOT_CMD_DATA.time_stamp) {
    nav54_stamp_seen = 1U;
    nav54_sender_stamp = RECEIVE_ROBOT_CMD_DATA.time_stamp;
    nav54_stamp_changed_at = now;
  }
  nav54_compat_angles_valid =
    (isfinite(RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch) &&
     isfinite(RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw)) ? 1U : 0U;
  /* Keep the received tracking bit intact. A missing tracking signal may
   * use explicit left-middle/right-down RC authorization, never fire/Move. */
  usb_nav_compat_active =
    !RECEIVE_ROBOT_CMD_DATA.data.tracking.tracking && usb_nav_compat_allowed(now);
  nav_override = RECEIVE_ROBOT_CMD_DATA.data.tracking.tracking || usb_nav_compat_active;
  if (nav_override) {
    ReceivedPacketVision.yaw = RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw;
    ReceivedPacketVision.pitch = RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch;
    ReceivedPacketVision.yaw_vel = 0;
    ReceivedPacketVision.pitch_vel = 0;
    ReceivedPacketVision.mode = RECEIVE_ROBOT_CMD_DATA.data.shoot.fire ? 2 : 1;
    Update_to_resolve();
  } else if ((uint32_t)(now - vision_last_rx) <= 100U) {
    ReceivedPacketVision = vision_cmd;
    Update_to_resolve();
  } else {
    ReceivedPacketVision.mode = 0;
    Update_to_resolve();
  }
}

static void unified_nav_apply_50(const uint8_t *frame, uint32_t now)
{
  usb_nav_compat_active = 0U;
  /* Standard 50-byte layout: header[0..3], timestamp[4..7],
   * speed[8..21], chassis[22..37], gimbal[38..45], shoot[46..47],
   * CRC16[48..49]. The two legacy one-byte fields are absent. */
  memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(RECEIVE_ROBOT_CMD_DATA));
  memcpy((uint8_t *)&RECEIVE_ROBOT_CMD_DATA, frame, 4);
  memcpy((uint8_t *)&RECEIVE_ROBOT_CMD_DATA + 4, frame + 4, 4);
  memcpy((uint8_t *)&RECEIVE_ROBOT_CMD_DATA + 8, frame + 8, 40);
  RECEIVE_ROBOT_CMD_DATA.data.shoot.fire = frame[46];
  RECEIVE_ROBOT_CMD_DATA.data.shoot.fric_on = frame[47];

  nav_last_rx = now;
  Navigation_speed_set.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.stop ? 0 : RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
  Navigation_speed_set.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.stop ? 0 : RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
  Navigation_speed_set.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;
  /* The 50-byte standard frame has no tracking flag. A fire command is
   * still allowed to request the navigation gimbal override; normal aiming
   * remains on ID 0x02. */
  nav_override = RECEIVE_ROBOT_CMD_DATA.data.shoot.fire ? 1U : 0U;
  if (nav_override) {
    ReceivedPacketVision.yaw = RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw;
    ReceivedPacketVision.pitch = RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch;
    ReceivedPacketVision.yaw_vel = 0;
    ReceivedPacketVision.pitch_vel = 0;
    ReceivedPacketVision.mode = 2;
    Update_to_resolve();
  }
}

static void usb_apply_vision_command(uint32_t now)
{
  vision_last_rx = now;
  if (!nav_override) {
    ReceivedPacketVision = vision_cmd;
    Update_to_resolve();
  }
}

static void unified_usb_frame(const uint8_t *frame, uint8_t length)
{
  uint32_t now = HAL_GetTick();
  if (!Verify_CRC16_Check_Sum(frame, length)) return;
  if (frame[2] == 0x01 && length == ATS_GCC_NAVIGATION_FRAME_SIZE) {
    unified_nav_apply_ats_gcc_54(frame, now);
  } else if (frame[2] == 0x01 && length == 50U) {
    unified_nav_apply_50(frame, now);
  } else if (frame[2] == 0x01 && length == sizeof(ReceiveDataRobotCmd_s)) {
    usb_nav_compat_active = 0U;
    memcpy(&RECEIVE_ROBOT_CMD_DATA, frame, length);
    

			nav_last_rx= now;
    Navigation_speed_set.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.stop ? 0 : RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
    Navigation_speed_set.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.stop ? 0 : RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
    Navigation_speed_set.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;
    nav_override = RECEIVE_ROBOT_CMD_DATA.data.tracking.tracking ||
                   RECEIVE_ROBOT_CMD_DATA.data.shoot.fire;
    if (nav_override) {
      ReceivedPacketVision.yaw = RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw;
      ReceivedPacketVision.pitch = RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch;
      ReceivedPacketVision.yaw_vel = 0;
      ReceivedPacketVision.pitch_vel = 0;
      ReceivedPacketVision.mode = RECEIVE_ROBOT_CMD_DATA.data.shoot.fire ? 2 : 1;
      Update_to_resolve();
    } else if ((uint32_t)(now - vision_last_rx) <= 100U) {
      ReceivedPacketVision = vision_cmd;
      Update_to_resolve();
    } else {
      ReceivedPacketVision.mode = 0;
      Update_to_resolve();
    }
  } else if (frame[2] == 0x02 && length == 35U) {
    uint8_t mode = frame[8];
    if (mode > 2U) return;
    vision_cmd.head = 0xA5;
    vision_cmd.mode = mode;
    memcpy((uint8_t *)&vision_cmd + 2, frame + 9, 24);
    usb_apply_vision_command(now);
  }
}

static void unified_usb_process(void)
{
  static uint8_t frame[64];
  static uint8_t used;
  uint8_t consumed, length;
  if (rx_overflow) {
    used = 0;
    rx_overflow = 0;
  }
  while (rx_tail != rx_head) {
    frame[used++] = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1U) & 511U;
    /* USB packets are arbitrary chunks: keep partial frames and consume all
     * complete frames. On bad data, shift one byte to find the next header. */
    while (used != 0U) {
      consumed = 1U;
      if (frame[0] == EXPECTED_INPUT_VISION_HEDER) {
        if (used < 2U) break;
        if (frame[1] <= 2U) {
          if (used < sizeof(VisionRecvData_t)) break;
          if (Verify_CRC16_Check_Sum(frame, sizeof(VisionRecvData_t))) {
            memcpy(&vision_cmd, frame, sizeof(vision_cmd));
            usb_apply_vision_command(HAL_GetTick());
            consumed = sizeof(VisionRecvData_t);
          }
        }
      } else if (frame[0] == 0x5AU) {
        if (used < 4U) break;
        if (frame[1] >= 4U && frame[1] <= sizeof(frame) - 6U &&
            unified_crc8(frame, 3U) == frame[3]) {
          length = frame[1] + 6U;
          if (used < length) break;
          if (Verify_CRC16_Check_Sum(frame, length)) {
            unified_usb_frame(frame, length);
            consumed = length;
          }
        }
      }
      used -= consumed;
      if (used != 0U) memmove(frame, frame + consumed, used);
    }
  }
}

static void unified_usb_send_state(void)
{
  USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  uint16_t crc;
  const float *angles;
  if (cdc == NULL || cdc->TxState != 0) return;
  state_tx[0] = 0x5A;
  state_tx[1] = 43;
  state_tx[2] = 0x0E;
  state_tx[3] = unified_crc8(state_tx, 3);
  {
    uint32_t tick = HAL_GetTick();
    memcpy(state_tx + 4, &tick, 4);
  }
  state_tx[8] = nav_override ? 4 : (Follow_Vision ? 1 : 0);
  memcpy(state_tx + 9, INS_quat, 16);
  angles = get_INS_angle_point();
  memcpy(state_tx + 25, &angles[0], 4);
  memcpy(state_tx + 29, &INS_gyro[0], 4);
  memcpy(state_tx + 33, &angles[1], 4);
  memcpy(state_tx + 37, &INS_gyro[1], 4);
  {
    float bullet_speed = Refree1_data.launching_frequency;
    uint16_t bullet_count = 0;
    memcpy(state_tx + 41, &bullet_speed, 4);
    memcpy(state_tx + 45, &bullet_count, 2);
  }
  crc = Get_CRC16_Check_Sum(state_tx, 47, CRC16_INIT);
  state_tx[47] = crc & 0xFF;
  state_tx[48] = crc >> 8;
  (void)CDC_Transmit_FS(state_tx, sizeof(state_tx));
}


/*
 * Feedback frames consumed by the current ATS standard_robot_pp_ros2 node.
 * Only ID 0x02/0x0C frames are scheduled in ATS feedback mode. The old
 * 42-byte vision and ID 0x0E state packets are not sent in this mode.
 * Existing CDC buffer ownership checks remain effective.
 */
static void unified_usb_send_imu_state(void)
{
  USBD_CDC_HandleTypeDef *cdc;
  uint16_t crc;
  const float *angles;
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
  cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (cdc == NULL || cdc->TxState != 0U) return;
	 
	
	
  ats_imu_state_tx[0] = 0x5A;
  ats_imu_state_tx[1] = 28U;
  ats_imu_state_tx[2] = 0x02;
  ats_imu_state_tx[3] = unified_crc8(ats_imu_state_tx, 3U);
  {
    uint32_t tick = HAL_GetTick();
    memcpy(ats_imu_state_tx + 4U, &tick, 4U);
  }
  angles = get_INS_angle_point();
  /* ATS ReceiveImuData order: yaw, pitch, roll, then Z/Y/X rates. */
  memcpy(ats_imu_state_tx + 8U, &angles[INS_YAW_ADDRESS_OFFSET], 4U);
  memcpy(ats_imu_state_tx + 12U, &angles[INS_PITCH_ADDRESS_OFFSET], 4U);
  memcpy(ats_imu_state_tx + 16U, &angles[INS_ROLL_ADDRESS_OFFSET], 4U);
  memcpy(ats_imu_state_tx + 20U, &INS_gyro[INS_GYRO_Z_ADDRESS_OFFSET], 4U);
  memcpy(ats_imu_state_tx + 24U, &INS_gyro[INS_GYRO_Y_ADDRESS_OFFSET], 4U);
  memcpy(ats_imu_state_tx + 28U, &INS_gyro[INS_GYRO_X_ADDRESS_OFFSET], 4U);
  crc = Get_CRC16_Check_Sum(ats_imu_state_tx, 32U, CRC16_INIT);
  ats_imu_state_tx[32U] = (uint8_t)(crc & 0xFFU);
  ats_imu_state_tx[33U] = (uint8_t)(crc >> 8);
  (void)CDC_Transmit_FS(ats_imu_state_tx, sizeof(ats_imu_state_tx));
}

static uint8_t get_yaw_joint_angles(float *big_joint, float *small_joint)
{
    static float chassis_zero, dm_zero, small_zero;
    const float chassis_sign = 1.0f;
    const float dm_sign = 1.0f;
    const float small_sign = 1.0f;
    float chassis_raw, dm_raw, small_raw;
    float chassis_abs, big_abs, small_abs;
    uint32_t irq_state, now, chassis_tick, dm_tick;
    uint8_t chassis_seen, dm_seen;

    /* 一起读取 CAN 中断写入的角度和时间戳。 */
    irq_state = __get_PRIMASK();
    __disable_irq();
    chassis_raw = chassis_yaw_rx_rad;
    chassis_tick = chassis_yaw_rx_ms;
    chassis_seen = chassis_yaw_seen;
    dm_raw = imu.yaw;
    dm_tick = dm_euler_rx_ms;
    dm_seen = dm_euler_seen;
    small_raw = get_INS_angle_point()[INS_YAW_ADDRESS_OFFSET];
    now = HAL_GetTick();
    __set_PRIMASK(irq_state);

    if (!chassis_seen || !dm_seen ||
        (uint32_t)(now - chassis_tick) > 100U ||
        (uint32_t)(now - dm_tick) > 100U ||
        toe_is_error(BOARD_GYRO_TOE))
    {
        return 0U;
    }

    dm_raw *= 0.01745329252f;  /* DM 的度转成 rad。 */

    if (yaw_joint_zero_request)
    {
        chassis_zero = chassis_raw;
        dm_zero = dm_raw;
        small_zero = small_raw;
        yaw_joint_zero_valid = 1U;
        yaw_joint_zero_request = 0U;
    }

    if (!yaw_joint_zero_valid)
        return 0U;

    chassis_abs = rad_format(chassis_sign * (chassis_raw - chassis_zero));
    big_abs = rad_format(dm_sign * (dm_raw - dm_zero));
    small_abs = rad_format(small_sign * (small_raw - small_zero));

    *big_joint = rad_format(big_abs - chassis_abs);
    *small_joint = rad_format(small_abs - big_abs);
    return 1U;
}


static void unified_usb_send_joint_state(void)
{
  USBD_CDC_HandleTypeDef *cdc;
  uint16_t crc;
  const float *angles;
	float big_joint, small_joint;
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
  cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (cdc == NULL || cdc->TxState != 0U) return;

	if (get_yaw_joint_angles(&big_joint, &small_joint) == 0U) return;
  ats_joint_state_tx[0] = 0x5A;
  ats_joint_state_tx[1] = 16U;
  ats_joint_state_tx[2] = 0x0C;
  ats_joint_state_tx[3] = unified_crc8(ats_joint_state_tx, 3U);
  {
    uint32_t tick = HAL_GetTick();
    memcpy(ats_joint_state_tx + 4U, &tick, 4U);
  }
  angles = get_INS_angle_point();
  /* Both yaw motors currently use the same calibrated INS absolute yaw. */
  memcpy(ats_joint_state_tx + 8U,  &big_joint,   sizeof(big_joint));
  memcpy(ats_joint_state_tx + 12U, &small_joint, sizeof(small_joint));
  memcpy(ats_joint_state_tx + 16U, &angles[INS_PITCH_ADDRESS_OFFSET], 4U);
  crc = Get_CRC16_Check_Sum(ats_joint_state_tx, 20U, CRC16_INIT);
  ats_joint_state_tx[20U] = (uint8_t)(crc & 0xFFU);
  ats_joint_state_tx[21U] = (uint8_t)(crc >> 8);
  (void)CDC_Transmit_FS(ats_joint_state_tx, sizeof(ats_joint_state_tx));
}

static void unified_usb_send_ats_feedback(void)
{
  switch (ats_feedback_tx_phase) {
    case 2U:
      unified_usb_send_imu_state();
      break;
    case 3U:
      unified_usb_send_joint_state();
      break;
    default:
      /* Leave old TX slots idle to preserve each frame's 20 ms cadence. */
      break;
  }
  ats_feedback_tx_phase = (uint8_t)((ats_feedback_tx_phase + 1U) % 4U);
}

/**
  * @brief      USB接收主函数，处理接收到的原始数据，根据数据头判断数据类型并分包处理
  * @param      None
  * @retval     None
  */
static void usb_receive(void)
{
  uint32_t len = USB_RECEIVE_LEN;
  CDC_Receive_FS(usb_rx_buf, &len); // Read data into the buffer

	if(usb_rx_buf[0] ==EXPECTED_INPUT_VISION_HEDER)
	{
		memcpy(&ReceivedPacketVision, usb_rx_buf, sizeof(VisionRecvData_t));
    usb_receive_vision(&ReceivedPacketVision);
	}
	osDelay(5);

  
}

/**
  * @brief      为视觉部分发送数据
  * @param      None
  * @retval     None
  */
int jjjj = 0;
//static void usb_send_vision(void)
//{
//  uint8_t crc_ok = Verify_CRC16_Check_Sum((const uint8_t *)(&ReceivedPacketVision), sizeof(ReceivedPacketVision));
//  SendPacketVision.header = SET_OUTPUT_VISION_HEDER; // 别放入 crc_ok
//  if (crc_ok)
//  {
//		
//    gimbal_INT_gyro_angle_point = get_INS_angle_point(); // 获取云台IMU欧拉角：0:yaw, 1:pitch, 2:roll（弧度）

//    SendPacketVision.detect_color = Refree_data.Vision_ID;  // TODO: 由裁判系统赋值，0-Red, 1-Blue
//    SendPacketVision.reset_tracker = 0; // TODO: 由自瞄模式赋值，是否重置追踪器
//    SendPacketVision.reserved = 0; // 保留位，无用
//    SendPacketVision.roll = gimbal_INT_gyro_angle_point[2];
//    SendPacketVision.pitch = gimbal_INT_gyro_angle_point[1];
//    SendPacketVision.yaw = gimbal_INT_gyro_angle_point[0];
//    SendPacketVision.aim_x =aim_x ; // TODO: SendPacketVision.aim_x 由自瞄模式赋值，主要用于上位机可视化，不影响实际功能
//    SendPacketVision.aim_y =aim_y; // TODO: SendPacketVision.aim_y 同上
//    SendPacketVision.aim_z =aim_z; // TODO: SendPacketVision.aim_y 同上
//		SendPacketVision.timestamp = HAL_GetTick(); 
//  }

//  Append_CRC16_Check_Sum(&SendPacketVision, sizeof(Vision_send_t), 1);
//  memcpy(usb_tx_buf, &SendPacketVision, sizeof(Vision_send_t));
////	jjjj = sizeof(Vision_send_t);
//  CDC_Transmit_FS(usb_tx_buf, sizeof(Vision_send_t));
//}
static void usb_send_vision(void)
{
  static uint8_t packet[sizeof(Vision_send_t)];
  USBD_CDC_HandleTypeDef *cdc;
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
  cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (cdc == NULL || cdc->TxState != 0U) return;
	
	
  SendPacketVision.header = SET_OUTPUT_VISION_HEDER;
  if (1)
  {
		
    gimbal_INT_gyro_angle_point = get_INS_angle_point(); 
    
const float RAD2DEG = 57.29577951308232f;
    gimbal_INT_gyro_angle_point = get_INS_angle_point(); // yaw=0 pitch=1 roll=2 (rad)
    /* mode ???????????????t??????????????? Follow_Vision ????????*/
    SendPacketVision.mode = (Follow_Vision ? 1 : 0);

	  
    /* ????? */
    SendPacketVision.q[0] = INS_quat[0];
    SendPacketVision.q[1] = INS_quat[1];
    SendPacketVision.q[2] = INS_quat[2];
    SendPacketVision.q[3] = INS_quat[3];

    SendPacketVision.yaw     = gimbal_INT_gyro_angle_point[0] * RAD2DEG;
    SendPacketVision.yaw_vel = INS_gyro[0] * RAD2DEG;
    SendPacketVision.pitch   = gimbal_INT_gyro_angle_point[1] * RAD2DEG;
    SendPacketVision.pitch_vel = INS_gyro[1] * RAD2DEG;

    //SendPacketVision.bullet_speed = Judge_Shoot_Data.bullet_speed;
//		SendPacketVision.bullet_speed = shoot_data_t.launching_frequency;
//		SendPacketVision.bullet_speed = shoot_data.bullet_speed;

//    SendPacketVision.bullet_count = 0; 


SendPacketVision.bullet_speed = Refree1_data.launching_frequency;
//SendPacketVision.bullet_speed =24;
SendPacketVision.bullet_count = 0;

		
  }
  Append_CRC16_Check_Sum(&SendPacketVision, sizeof(Vision_send_t), 1);
  /* Keep this buffer unchanged until the USB transfer completes. */
  memcpy(packet, &SendPacketVision, sizeof(packet));
//	jjjj = sizeof(Vision_send_t);
  (void)CDC_Transmit_FS(packet, sizeof(packet));
}

/**
  * @brief      接收导航数据
  * @param[in]  ReceivedPacketTwist: USB接收到的导航数据
  *             右手系，x轴向前，y轴向左，z轴向上
  *             - x: x轴方向的线速度，单位为m/s
  *             - y: y轴方向的线速度，单位为m/s
  *             - z: z轴方向的线速度，单位为m/s，置零
  *             - roll: 绕x轴的角速度，单位为rad/s，置零
  *             - pitch: 绕y轴的角速度，单位为rad/s，置零
  *             - yaw: 绕z轴的角速度，单位为rad/s
  * @retval     None
  */
static void usb_receive_navigation(ReceivedPacketTwist_s *ReceivedPacketTwist){
  uint8_t crc_ok = Verify_CRC16_Check_Sum((const uint8_t *)ReceivedPacketTwist, sizeof(ReceivedPacketTwist_s));
  if (crc_ok)
  {
			Navigation_speed_set.vx = ReceivedPacketTwist->vx;
		  Navigation_speed_set.vy = ReceivedPacketTwist->vy;
		  Navigation_speed_set.wz = ReceivedPacketTwist->wz;

  }
}

ReceivedPacketTwist_s *get_navigation_data_point()
{
  return &Navigation_speed_set;
}

/**
  * @brief      接收视觉数据
  * @param[in]  ReceivedPacketVision: USB接收到的视觉数据
  * @retval     None
  */
static void usb_receive_vision(VisionRecvData_t *ReceivedPacketVision)
{
  uint8_t crc_ok = Verify_CRC16_Check_Sum((const uint8_t *)ReceivedPacketVision, sizeof(VisionRecvData_t));
  if (crc_ok)
  {
			  
	  Update_to_resolve();
	  

    // buzzer_on(500, 30000);
    // TODO：与电控部分视觉解算对接赋值
  }
}

/**
  * @brief      USB打印函数，类似于printf，将数据发送到USB
  * @param[in]  fmt: 格式化字符串
  * @retval     None
  */
static void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;
    va_start(ap, fmt);
    len = vsprintf((char *)usb_buf, fmt, ap);
    va_end(ap);
    CDC_Transmit_FS(usb_buf, len);
}

/**
  * @brief      将字符串转换为uint8_t数组
  * @param[out] word: 转换后的数组
  * @param[in]  str: 原始字符串
  * @retval     None
  */
void char_to_uint(uint8_t *word, const char *str)
{
    int i = 0;
    while (str[i] != '\0' && i < 10)
    {
        word[i] = str[i];
        i++;
    }
}

// CRC Table
const uint16_t W_CRC_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/**
 * @brief CRC16 Caculation function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
 * @return : CRC16 checksum
 */
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;
  if (pchMessage == NULL)
      return 0xFFFF;
  while (dwLength--)
  {
      ch_data = *pchMessage++;
      (wCRC) =
          ((uint16_t)(wCRC) >> 8) ^ W_CRC_TABLE[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
  }
  return wCRC;
}

/**
 * @brief CRC16 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */
uint32_t Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;
  if ((pchMessage == NULL) || (dwLength <= 2))
    return false;
  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT); // dwLength - 2 ->dwLength - 4
  return (
    (w_expected & 0xff) == pchMessage[dwLength - 2] &&
    ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**
 * @brief CRC16 Append function for different data structures
 * @param[in] pchMessage : Data to append CRC
 * @param[in] dwLength : Stream length = Data length + checksum
 * @param[in] dataType : Type of data (0 for OutputPCData_s, 1 for SendPacketVision_s, 2 for SendPacketAllRobotHP_s, 3 for SendPacketGameStatus_s, 4 for SendPacketRobotStatus_s)
 * @return : True or False (CRC Verify Result)
 */
void Append_CRC16_Check_Sum(void *pchMessage, uint32_t dwLength, int dataType)
{
  uint16_t w_crc = 0;

  if (pchMessage == NULL || dwLength <= 2)
  {
    return;
  }
  if (dataType == 1)
  {
    Vision_send_t *send_data = (Vision_send_t *)pchMessage;
    w_crc = Get_CRC16_Check_Sum((uint8_t *)(send_data), dwLength - 2, CRC16_INIT);
    send_data->checksum = w_crc;
  }
  else if (dataType == 2) {
    SendPacketAllRobotHP_s *send_data = (SendPacketAllRobotHP_s *)pchMessage;
    w_crc = Get_CRC16_Check_Sum((uint8_t *)(send_data), dwLength - 2, CRC16_INIT);
    send_data->checksum = w_crc;
  }
  else if (dataType == 3)
  {
    SendPacketGameStatus_s *send_data = (SendPacketGameStatus_s *)pchMessage;
    w_crc = Get_CRC16_Check_Sum((uint8_t *)(send_data), dwLength - 2, CRC16_INIT);
    send_data->checksum = w_crc;
  }
  else if (dataType == 4)
  {
    SendPacketRobotStatus_s *send_data = (SendPacketRobotStatus_s *)pchMessage;
    w_crc = Get_CRC16_Check_Sum((uint8_t *)(send_data), dwLength - 2, CRC16_INIT);
    send_data->checksum = w_crc;
  }
  else if (dataType == 0)
  {
    OutputPCData_s *output_data = (OutputPCData_s *)pchMessage;
    w_crc = Get_CRC16_Check_Sum((uint8_t *)(output_data), dwLength - 2, CRC16_INIT);
    output_data->checksum = w_crc;
  }
}
