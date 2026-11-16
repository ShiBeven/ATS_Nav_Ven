#ifndef BSP_PID_H
#define BSP_PID_H
#include "main.h"
#include "gimbal_task.h"
#include "Filter.h"
#include "remote_control.h"
#include "usb_task.h"




typedef struct
{
	const RC_ctrl_t *RC;
	gimbal_control_t *gimbal_data;
	ReceivedPacketTwist_s *navigation_data;
}CAN_Send_t_YU;

typedef struct {
    float kf;       // 动态调整的前馈增益
    float last_in;  // 记录上次输入
} Feedforward_t;

//extern void SystemPIDInit(void);
extern void Feedforward_Init(Feedforward_t *ff, float kf_init);
extern float Feedforward_Update(Feedforward_t *ff, float in);

#endif//BSP_PID_H

