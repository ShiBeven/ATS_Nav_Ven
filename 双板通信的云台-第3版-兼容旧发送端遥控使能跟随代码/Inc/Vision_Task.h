#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "kalman.h"
#include "referee_lib.h"
#include "Vision.h"
#include "bsp_usart.h"
#include "stdlib.h"
#include "INS_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "math.h"
#include "remote_control.h"
#include "user_lib.h"
#include "gimbal_task.h"
#include "Vision.h"
#include "SolveTrajectory.h"



typedef struct
{
	float PredictionYaw;
	float PredictionPitch;

	float PredictionYawRpm;
	float PredictionPitchRpm;

	float YawOffset;
	float PitchOffset;
}PredictionData_t;



typedef struct 
{

    const RC_ctrl_t *RcCtrl;
    const fp32 *get_angle_point;
    const fp32 *get_gyro_point;
	PredictionData_t PredictionData;
	gimbal_control_t *GetGimbalData;
//		VisionRecvData_t *DataStruct;
}VisionData_t;
extern uint8_t shoot_control;
void Vision_send_Control(void);
const VisionData_t *Data(void);
void AutoDataUpdate(void);
extern void Vision_task(void const *pvParameters);
extern uint8_t Follow_Vision;
//extern gimbal_behaviour_e gimbal_behaviour;  // 声明全局云台模式变量


#endif

