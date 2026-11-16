
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
typedef   struct
{

	uint8_t head ; 
  uint8_t mode;  
  float yaw;     
  float yaw_vel; 
  float yaw_acc; 
  float pitch;   
  float pitch_vel; 
  float pitch_acc; 
  uint16_t crc16; 
}VisionRecvData_t;
typedef  struct
{
	uint8_t header;
  uint8_t mode;       
  float q[4];         
  float yaw;          
  float yaw_vel;      
  float pitch;        
  float pitch_vel;    
  float bullet_speed; 
  uint16_t bullet_count;
  uint16_t checksum;  
} Vision_send_t;
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


enum mode { Attack=1, Defend=2, Move=3, EnhancedAttack=4, EnhancedDefend=5, EnhancedMove=6 };
static uint8_t get_yaw_joint_angles(float *big_joint, float *small_joint) {
  *big_joint=angles[0]; *small_joint=angles[0]; return 1;
}

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
uint32_t Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;
  if ((pchMessage == NULL) || (dwLength <= 2))
    return false;
  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT); 
  return (
    (w_expected & 0xff) == pchMessage[dwLength - 2] &&
    ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
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
void unified_usb_receive_bytes(const uint8_t *bytes, uint32_t length)
{
#if USB_YAW_DEBUG
  
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
static void unified_nav_apply_ats_gcc_54(const uint8_t *frame, uint32_t now)
{
  float decoded[4];
  uint16_t checksum;

  memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(RECEIVE_ROBOT_CMD_DATA));

  
  memcpy((uint8_t *)&RECEIVE_ROBOT_CMD_DATA, frame, 4U);
  memcpy((uint8_t *)&RECEIVE_ROBOT_CMD_DATA + 4U, frame + 4U, 4U);

  
  
  memcpy(decoded, frame + 8U, 12U);
  RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx = decoded[0];
  RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy = decoded[1];
  RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz = decoded[2];
  RECEIVE_ROBOT_CMD_DATA.data.speed_vector.stop = frame[20U] != 0U;
  RECEIVE_ROBOT_CMD_DATA.data.speed_vector.mode = (enum mode)frame[21U];

  
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

  nav_override = RECEIVE_ROBOT_CMD_DATA.data.tracking.tracking ||
                 RECEIVE_ROBOT_CMD_DATA.data.shoot.fire;
#if USB_ATS_GCC_GIMBAL_MODE_FALLBACK
  
  nav_override = nav_override ||
                 (RECEIVE_ROBOT_CMD_DATA.data.speed_vector.mode == 1U);
#endif
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
    gimbal_INT_gyro_angle_point = get_INS_angle_point(); 
    
    SendPacketVision.mode = (Follow_Vision ? 1 : 0);

	  
    
    SendPacketVision.q[0] = INS_quat[0];
    SendPacketVision.q[1] = INS_quat[1];
    SendPacketVision.q[2] = INS_quat[2];
    SendPacketVision.q[3] = INS_quat[3];

    SendPacketVision.yaw     = gimbal_INT_gyro_angle_point[0] * RAD2DEG;
    SendPacketVision.yaw_vel = INS_gyro[0] * RAD2DEG;
    SendPacketVision.pitch   = gimbal_INT_gyro_angle_point[1] * RAD2DEG;
    SendPacketVision.pitch_vel = INS_gyro[1] * RAD2DEG;

    






SendPacketVision.bullet_speed = Refree1_data.launching_frequency;

SendPacketVision.bullet_count = 0;

		
  }
  Append_CRC16_Check_Sum(&SendPacketVision, sizeof(Vision_send_t), 1);
  
  memcpy(packet, &SendPacketVision, sizeof(packet));

  (void)CDC_Transmit_FS(packet, sizeof(packet));
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
  
  memcpy(ats_joint_state_tx + 8U,  &big_joint,   sizeof(big_joint));
  memcpy(ats_joint_state_tx + 12U, &small_joint, sizeof(small_joint));
  memcpy(ats_joint_state_tx + 16U, &angles[INS_PITCH_ADDRESS_OFFSET], 4U);
  crc = Get_CRC16_Check_Sum(ats_joint_state_tx, 20U, CRC16_INIT);
  ats_joint_state_tx[20U] = (uint8_t)(crc & 0xFFU);
  ats_joint_state_tx[21U] = (uint8_t)(crc >> 8);
  (void)CDC_Transmit_FS(ats_joint_state_tx, sizeof(ats_joint_state_tx));
}
#include <setjmp.h>
#define USB_ATS_COMPAT_FEEDBACK_TX 1
#define USB_LEGACY_VISION_TX 1
static jmp_buf loop_end;
static void MX_USB_DEVICE_Init(void) {}
static void CAN_CmdBoard3(void *p) { (void)p; }
static void unified_usb_send_ats_feedback(void) {}
static void osDelay(int n) { (void)n; longjmp(loop_end,1); }
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
    
    now = HAL_GetTick();
    if ((uint32_t)(now - last_chassis) >= 10U) {
      last_chassis = now;
      if ((uint32_t)(now - nav_last_rx) > 100U) {
        Navigation_speed_set.vx = 0;
        Navigation_speed_set.vy = 0;
        Navigation_speed_set.wz = 0;
        if (nav_override) {
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
static void run_task_once(void) { if (!setjmp(loop_end)) usb_task(0); }

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
  tick += 200; feed(nav54_attack, 54);
  CHECK(nav_last_rx == tick && nav_override == 0 && ReceivedPacketVision.mode == 0);
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

  /* Replay the observed Move/tracking=0/fire toggle: fire cannot own aim. */
  tick += 200;
  for (i = 0; i < 8; ++i) {
    unified54(nav54, 0, 3, (uint8_t)(i & 1), 1, 0);
    tick += 20; feed(nav54,54); run_task_once();
    CHECK(nav_override == 0 && ReceivedPacketVision.mode == 0);
    CHECK(Navigation_speed_set.vx == 0.75f);
  }
  /* A fresh valid target tracks across all fire transitions and postures. */
  for (i = 0; i < 20; ++i) {
    unified54(nav54, 0, (uint8_t)(i % 7), (uint8_t)(i & 1), 1, 1);
    field = 0.1f + i * 0.01f; memcpy(nav54+41,&field,4); checksum(nav54,54);
    tick += 20; feed(nav54,54); run_task_once();
    CHECK(nav_override == 1 && ReceivedPacketVision.mode == ((i&1)?2:1));
    CHECK(ReceivedPacketVision.pitch == field && nav_last_rx == tick);
  }
  /* Explicit loss and conflicting fire must release immediately. */
  unified54(nav54,0,1,1,1,0); ++tick; feed(nav54,54);
  CHECK(nav_override == 0 && ReceivedPacketVision.mode == 0);
  /* Fresh independent vision is held while nav owns aim, then restored. */
  unified54(nav54,0,3,0,1,1); ++tick; feed(nav54,54);
  legacy(one,1,0.75f); feed(one,28);
  CHECK(nav_override == 1 && ReceivedPacketVision.yaw == -0.45f);
  unified54(nav54,0,3,0,1,0); feed(nav54,54);
  CHECK(nav_override == 0 && ReceivedPacketVision.mode == 1 && ReceivedPacketVision.yaw == 0.75f);
  /* No stale independent source can silently resume control. */
  tick += 101; feed(nav54,54); run_task_once();
  CHECK(nav_override == 0 && ReceivedPacketVision.mode == 0);
  /* CRC failure may not drop a currently valid target. */
  unified54(nav54,0,3,0,1,1); ++tick; feed(nav54,54);
  before=nav_last_rx; nav54[51]=0; ++tick; feed(nav54,54);
  CHECK(nav_override == 1 && nav_last_rx == before && ReceivedPacketVision.mode == 1);
  /* Real task timeout, including the fresh-vision handover at timeout. */
  unified54(nav54,0,3,0,1,1); ++tick; feed(nav54,54);
  tick += 100; run_task_once(); CHECK(nav_override == 1 && ReceivedPacketVision.mode == 1);
  ++tick; run_task_once(); CHECK(nav_override == 0 && ReceivedPacketVision.mode == 0);
  CHECK(Navigation_speed_set.vx == 0);
  ++tick; feed(nav54,54); tick += 90; legacy(one,1,0.75f); feed(one,28);
  tick += 11; run_task_once();
  CHECK(nav_override == 0 && ReceivedPacketVision.mode == 1 && ReceivedPacketVision.yaw == 0.75f);
  /* 32-bit tick rollover preserves fresh navigation until the boundary. */
  tick=0xFFFFFFF0U; feed(nav54,54); vision_last_rx=tick-200;
  tick=0x20U; run_task_once(); CHECK(nav_override == 1 && ReceivedPacketVision.mode == 1);
  tick=0x55U; run_task_once(); CHECK(nav_override == 0 && ReceivedPacketVision.mode == 0);
  printf("PASS %d checks: actual RX parser, legacy compatibility, navigation and unified 49-byte state, units, axes, CRC, busy guards\n", checks);
  return 0;
}
