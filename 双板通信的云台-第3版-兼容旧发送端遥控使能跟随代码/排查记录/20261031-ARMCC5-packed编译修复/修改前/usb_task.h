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

#ifndef USB_TASK_H
#define USB_TASK_H

/* 1: SerialPlotter telemetry, 0: vision/navigation USB communication. */
#define USB_YAW_DEBUG 0

/* Normal-mode TX: 1 = original 42-byte vision reply, 0 = unified ID 0x0E.
 * RX accepts original 28-byte vision commands and existing unified frames. */
#define USB_LEGACY_VISION_TX 1

/* Additive feedback TX for the current ATS receiver: keep the original ID 0x0E state frame and also emit ID 0x02/0x0C compatibility frames. */
#define USB_ATS_COMPAT_FEEDBACK_TX 1

/* The current /home/ats/ATS ROS bridge writes gimbal angles into the 54-byte
 * robot frame but does not populate data.tracking.  Its Attack posture (1) is
 * the only unchanged signal that means visual gimbal control is enabled.
 * Keep this fallback limited to the additive GCC frame path below. */
#define USB_ATS_GCC_GIMBAL_MODE_FALLBACK 1

#include <stdint.h>
#include <stdbool.h>

/* Exact field order of the navigation board's ReceiveDataRobotCmd_s. */
typedef __packed struct { uint8_t sof, len, id, crc; } UnifiedFrameHeader_t;


/*
 * Compatibility wire description for the current ATS navigation sender.
 *
 * ATS is compiled with GCC.  Its packed SendRobotCmdData contains a
 * four-byte C++ enum for speed_vector.mode, so the bytes on USB are 54 bytes
 * long (header + timestamp + payload + CRC16), with len == 48.  Do not cast
 * this frame to ReceiveDataRobotCmd_s: the ARM compiler uses a one-byte enum
 * in that legacy structure and all fields after mode would be shifted.
 */
#define ATS_GCC_NAVIGATION_FRAME_SIZE 54U
#define ATS_GCC_NAVIGATION_PAYLOAD_LEN 48U
typedef __packed struct AtsGccNavigationFrame54
{
  UnifiedFrameHeader_t frame_header;
  uint32_t time_stamp;
  float vx;
  float vy;
  float wz;
  uint8_t stop;
  uint8_t mode;
  uint8_t mode_reserved[3];
  float chassis_roll;
  float chassis_pitch;
  float chassis_yaw;
  float chassis_leg_length;
  float gimbal_pitch;
  float gimbal_yaw;
  uint8_t fire;
  uint8_t fric_on;
  uint8_t tracking;
  uint16_t checksum;
} AtsGccNavigationFrame54_t;
typedef __packed struct RobotCmdData
{
  UnifiedFrameHeader_t frame_header;
  uint32_t time_stamp;
  __packed struct
  {
    __packed struct
    {
      float vx, vy, wz;
      bool stop;
      enum mode { Move = 0, Attack = 1, Defend = 2 } mode;
    } speed_vector;
    __packed struct { float roll, pitch, yaw, leg_lenth; } chassis;
    __packed struct { float pitch, yaw; } gimbal;
    __packed struct { uint8_t fire, fric_on; } shoot;
    __packed struct { bool tracking; } tracking;
    __packed struct { uint8_t custom_flag; } custom_flag;
  } data;
  uint16_t checksum;
} ReceiveDataRobotCmd_s;

extern ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
void unified_usb_receive_bytes(const uint8_t *bytes, uint32_t length);

//typedef  __packed struct
//{
//  uint8_t header;
//	uint8_t state : 2;
//	uint8_t id : 3;
//	uint8_t armors_num : 2;
//  float x;
//  float y;
//  float z;
//  float yaw;
//  float vx;
//  float vy;
//  float vz;
//  float v_yaw;
//  float r1;
//  float r2;
//  float dz;
//	//float armor_area;
//	uint32_t cap_timestamp;
//	uint16_t t_offset;
//  uint16_t checksum;
//}VisionRecvData_t;

typedef  __packed struct
{

	uint8_t head ; // ???????????A5
  uint8_t mode;  // ????????0-??????, 1-???????????????2-????????????
  float yaw;     // ???????????λ?????
  float yaw_vel; // ??????????????λ????/??
  float yaw_acc; // ???????????????λ????/??2??
  float pitch;   // ??????????λ?????
  float pitch_vel; // ?????????????λ????/??
  float pitch_acc; // ??????????????λ????/??2??
  uint16_t crc16; // CRC16У???	
}VisionRecvData_t;

//typedef __packed struct
//{
//	uint8_t header;
//  uint8_t detect_color :1;
//  uint8_t task_mode :2; 
//  bool reset_tracker :1;
//  uint8_t is_play :1;
//  bool change_target :1;
//	uint8_t reserved :2;
//  float roll;
//  float pitch;
//  float yaw;
//  float aim_x;
//  float aim_y;
//  float aim_z;
//  uint16_t game_time;  // (s) game time [0, 450]
//  uint32_t timestamp;  // (ms) board time
//  uint16_t checksum;
//} Vision_send_t;
//typedef __packed struct
//{
//uint8_t mode;
//st.k = 0.045;                                  // 空气阻力系数
//st.bullet_type = BULLET_17;                    // 子弹类型
//st.current_v = 22;                             // 子弹速度
//st.current_pitch = VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle;  // 当前俯仰角
//st.current_yaw = VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle;      
//	
//}visionrecv_CRC

typedef __packed struct
{
	uint8_t header;
  uint8_t mode;       /* ???????????0-????,1-?????????????????? */
  float q[4];         /* ????? w,x,y,z */
  float yaw;          /* deg */
  float yaw_vel;      /* deg/s */
  float pitch;        /* deg */
  float pitch_vel;    /* deg/s */
  float bullet_speed; /* m/s */
  uint16_t bullet_count;
  uint16_t checksum;  /* CRC16??С??洢 */
} Vision_send_t;


typedef struct
{
  float pitch;
  float yaw;
} __attribute__((packed)) SendNavigation_t;
	
typedef struct
{
  float vx;
  float vy;
  float wz;
} __attribute__((packed)) ReceivedPacketTwist_s;

typedef struct
{
  uint8_t header;
  uint16_t red_1_robot_hp;
  uint16_t red_2_robot_hp;
  uint16_t red_3_robot_hp;
  uint16_t red_4_robot_hp;
  uint16_t red_5_robot_hp;
  uint16_t red_7_robot_hp;
  uint16_t red_outpost_hp;
  uint16_t red_base_hp;
  uint16_t blue_1_robot_hp;
  uint16_t blue_2_robot_hp;
  uint16_t blue_3_robot_hp;
  uint16_t blue_4_robot_hp;
  uint16_t blue_5_robot_hp;
  uint16_t blue_7_robot_hp;
  uint16_t blue_outpost_hp;
  uint16_t blue_base_hp;
  uint16_t checksum;
} __attribute__((packed)) SendPacketAllRobotHP_s;

typedef struct
{
  uint8_t header;
  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint16_t checksum;
} __attribute__((packed)) SendPacketGameStatus_s;

typedef struct
{
  uint8_t header;
  uint8_t robot_id;
  uint16_t current_hp;
  uint16_t shooter_heat;
  bool team_color; // 0-red 1-blue
  bool is_attacked;
  uint16_t checksum;
} __attribute__((packed)) SendPacketRobotStatus_s;

typedef struct
{
  uint8_t header;
  uint16_t ecd_set;
  uint16_t checksum;
} __attribute__((packed)) InputPCData_s;

typedef struct
{
  uint8_t header;
  uint16_t length;
  uint8_t name_1[10];
  uint8_t type_1;
  float data_1;
  uint8_t name_2[10];
  uint8_t type_2;
  float data_2;
  uint8_t name_3[10];
  uint8_t type_3;
  float data_3;
  uint8_t name_4[10];
  uint8_t type_4;
  float data_4;
  uint8_t name_5[10];
  uint8_t type_5;
  float data_5;
  uint8_t name_6[10];
  uint8_t type_6;
  float data_6;
  uint8_t name_7[10];
  uint8_t type_7;
  float data_7;
  uint8_t name_8[10];
  uint8_t type_8;
  float data_8;
  uint8_t name_9[10];
  uint8_t type_9;
  float data_9;
  uint8_t name_10[10];
  uint8_t type_10;
  float data_10;
  uint16_t checksum;
} __attribute__((packed)) OutputPCData_s;

extern void usb_task(void const *argument);
extern ReceivedPacketTwist_s Navigation_speed_set;
extern  VisionRecvData_t ReceivedPacketVision;
extern  Vision_send_t SendPacketVision;
extern ReceivedPacketTwist_s *get_navigation_data_point();

#endif

