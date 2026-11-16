#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "main.h"
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "pid.h"
#include "remote_control.h"
extern uint16_t AUTO;

#define SHOOT_OPEN_FIRE KEY_PRESSED_OFFSET_E
#define FIRE_INTERVAL_MS 80  // 每次开火最少间隔100ms，可调节

#define SHOOT_FULL_TIME 80 
#define SHOOT_CONTROL_TIME 100

//#define  Single_Data  36864
#define Normal_Mode 1

/*摩擦轮电机最大输出限制*/
#define FIRE_left_SPEED_PID_MAX_OUT 15000.0f
#define FIRE_left_SPEED_PID_MAX_IOUT 1000.0f

#define FIRE_right_SPEED_PID_MAX_OUT 15000.0f
#define FIRE_right_SPEED_PID_MAX_IOUT 1000.0f

#define FIRE_MOMENT_PID_MAX_OUT 15000.0f
#define FIRE_MOMENT_PID_MAX_IOUT 1000.0f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP 0.3f
#define TRIGGER_ANGLE_PID_KI 0.0f
#define TRIGGER_ANGLE_PID_KD 0.9f

#define TRIGGER_SPEED_PID_KP 9.0f
#define TRIGGER_SPEED_PID_KI 0.0f
#define TRIGGER_SPEED_PID_KD 1.0f
/*拨弹盘双环PID输出限制*/
#define BULLET_ANG_PID_MAX_OUT 10000.0f
#define BULLET_ANG_PID_MAX_IOUT 0.0f
#define BULLET_SPEED_PID_MAX_OUT 10000.0f
#define BUFFET_SPEED_PID_MAX_IOUT 1000.0f

#define TRIGGER_MAX_SPEED 6000.0f   // 拨弹盘电机最大转速（rpm）
// 拨弹盘控制模式
typedef enum {
    TRIGGER_NORMAL_MODE,      // 正常模式：角度环输出不限制
    TRIGGER_HEAT_LIMIT_MODE   // 热量限制模式：角度环输出受热量限制
} TriggerControlMode;

// 全局变量
//float max_speed_allowed = 0.0f;    // 当前允许的最大转速（rpm）

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_STAR,
}ShootMode_e;



typedef struct
{
    const motor_measure_t *LeftFireData;
    const motor_measure_t *RightFireData;
    const motor_measure_t *TriggerData;
    const RC_ctrl_t *RC_data;
    ShootMode_e ShootMode;
    int8_t last_rc_s;
    int8_t last_rc_T;
    fp32 Fire_left_speed_set;
    fp32 Fire_right_speed_set;
    int TriggerMotor_pos_set;
    int16_t give_current[3];
		uint16_t block_time;
		uint16_t Reset_ECD;
	

} ShootControl_t;


void ShootInit(void);
void Set_ShootMode(void);
void ShootDataUpdata(void);
void Set_ShootVal(void);

extern void Shoot_PIDCalc(void);
extern void ShootTask(void const *pvParameters);



#endif
