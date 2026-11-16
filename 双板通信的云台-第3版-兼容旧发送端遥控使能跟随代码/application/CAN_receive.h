#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "main.h"
#include "struct_typedef.h"
#include "usb_task.h"
#include "remote_control.h"
#define BOARD_CAN hcan1
// #define Yaw_CAN hcan1
#define Gimbal_CAN hcan1
#define SHOOT_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
  CAN_GIMBAL_ALL_ID = 0x1FF,
  CAN_SHOOT_ALL_ID = 0x200,
  /*can1*/
	CAN_GM6020_YAW_ID = 0x205,
	CAN_GM6020_PITCH_ID = 0x206,
	CAN_GM6020_YAW2_ID=0x20B,
	CAN_GM3508_RIGHT_FIRE_ID = 0x204,
  CAN_GM3508_LEFT_FIRE_ID = 0x203,
  CAN_GM2006_AMMUNITION_BOOSTER_ID = 0x201, 
	CAN_INS_ID=0x301,

  /*can2*/  
	CAN_Board_ID = 0x111,
	CAN_Board2_ID = 0x112,
  CAN_Board3_ID	= 0x113,
	CAN_Board4_ID = 0x114,
	CAN_Board5_ID = 0x115,
	CAN_Board6_ID = 0x116,
	CAN_Board7_ID = 0x117,
	CAN_Board8_ID = 0x118,
	CAN_Board9_ID = 0x119,
	CAN_RC_MOUSE_ID = 0x120,
	CAN_GIMBAL_STATE_ID = 0x121,
	CAN_CHASSIS_YAW_ID  = 0x122,
} can_msg_id_e;

// rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
  int32_t total_angle;
  int32_t round_cnt;
} motor_measure_t;

//typedef struct
//{
//  uint16_t shooter_17mm_1_barrel_heat;
//	uint8_t ID;
//	uint16_t red_outpost_HP;
//	uint16_t blue_outpost_HP;	
//  uint8_t launching_frequency;
//  uint16_t projectile_allowance_17mm;
//	uint8_t ad_YN;
//}shoot_data_t;

typedef struct
{
	uint8_t Vision_ID;
	uint16_t shooter_17mm_barrel_heat; //17mm弹丸射击热量
	uint8_t gimbal_state;//uint16_t shooter_barrel_cooling_value;//
	uint16_t shooter_barrel_heat_limit; //发射机构热量限制
}refree_data_t;

typedef struct
{
	float launching_frequency;//弹丸初速度
}refree1_data_t;
typedef struct
{
	uint16_t current_Hp;
    uint8_t game_progress : 4;//比赛状态进程
	uint16_t shooter_barrel_cooling_value;
} game_state_t;
typedef struct
{
	float groy;
}refree1_ins_t;
typedef struct
{
	uint16_t stage_remain_time;
}stage_remain_time_t;	
typedef struct
{
	int8_t azzz;
}aaaz_t;
typedef union
{
  uint8_t byte[4];
  float f_data;
} CharUFloat_t;


#define MOTOR1  0x04   //1电机ID
#define MOTOR2  0x02   //2电机ID
#define MOTOR3  0x01  //3电机ID
#define P_MIN   -12.5  //位置最小值
#define P_MAX   12.5   //位置最大值
#define V_MIN   -45    //速度最小值
#define V_MAX   45     //速度最大值
#define KP_MIN  0      //Kp最小值
#define KP_MAX  500    //Kp最大值
#define KD_MIN  0      //Kd最小值
#define KD_MAX  5      //Kd最大值
#define T_MIN   -18    //转矩最大值
#define T_MAX   18     //转矩最小值

//电机结构体
typedef struct
{
  int p_int;
  int v_int;
  int t_int;
  int position;
  int velocity;
  int torque;
}Motor_t;
//发送包
typedef struct
{
	CAN_TxHeaderTypeDef hdr;
	uint8_t payload[8];
}CAN_TxPacketTypeDef;
//接收包
typedef struct
{
	CAN_RxHeaderTypeDef hdr;
	uint8_t payload[8];
}CAN_RxPacketTypeDef;



extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_shoot_trigger_measure_point(void);
extern const motor_measure_t *get_shoot_fire_left_point(void);
extern const motor_measure_t *get_shoot_fire_right_point(void);
extern void CAN_CmdShoot(int16_t right, int16_t left, int16_t Trigger,int16_t A);
extern void CAN_CmdBoard3(ReceivedPacketTwist_s *Navigation_speed_set);
extern void CAN_SendRcBasic(const RC_ctrl_t *rc);
extern void CAN_SendRcAux(const RC_ctrl_t *rc);
extern void CAN_SendRcMouse(const RC_ctrl_t *rc);
extern void CAN_cmd_gimbal( int16_t pitch);
extern const motor_measure_t *get_yaw2_gimbal_motor_measure_point(void);
extern motor_measure_t ShootMotor[3];
extern motor_measure_t GimbalMotor[3];
extern motor_measure_t MOTOR_YAW2;
extern  Motor_t MOTOR1_t, MOTOR2_t, MOTOR3_t;
extern void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _pos, float _vel);
extern void Enable_CtrlMotor(CAN_HandleTypeDef* hcan,uint8_t ID, uint8_t data0, uint8_t data1,uint8_t data2, uint8_t data3, uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7);
extern void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq);
extern void CAN_cmd_gimbal_2(int16_t yaw);
extern void CAN_cmd_gimbal_3(int16_t yaw2);

extern refree_data_t Refree_data;
extern refree1_data_t Refree1_data;//弹速

extern void CAN_CmdBoard5(uint8_t vision_state);
extern void CAN_SendGimbalState(void);



/* Fresh, coherent feedback snapshot: index 0 Pitch, 1 big-Yaw, 2 small-Yaw. */
extern bool_t gimbal_get_motor_snapshot(uint8_t index, motor_measure_t *out, uint32_t *age_ms);

#endif

