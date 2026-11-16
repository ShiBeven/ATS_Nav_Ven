#include "bsp_pid.h"
#include "Vision.h"
#include "Vision_Task.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "dm_imu.h"
//SystemPIDCalc_t SystemPIDCalc;
//CAN_Send_t CANSendData;
CAN_Send_t_YU CANSendData_YU;
extern uint8_t Vision_state;

static uint8_t Frequency = 0;
uint8_t FreTime=0;
extern ReceivedPacketTwist_s *get_navigation_data_point(void);
static uint8_t can2_slot = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  FreTime++;
    CANSendData_YU.RC = get_remote_control_point();
		CANSendData_YU.gimbal_data = get_gimlal_data();
	if(htim->Instance == TIM6)
	{
		/* 云台板只回传真实编码器状态；遥控器数据统一由导航板广播。 */
		remote_control_can_timeout_check();
		CAN_SendGimbalState();
	}
//	  CANSendData_YU.navigation_data = get_navigation_data_point();
	
    if(htim->Instance == TIM2)
    {
			Frequency++;
      if(Frequency % 2 == 0 )
        {
					CAN_cmd_gimbal_2(yaw_can_set_current);
					
					CAN_cmd_gimbal(pitch_can_set_current);
				
//					CAN_cmd_gimbal_3(yaw2_can_set_current);
				}
				else
				{
					imu_request_gyro();
				}

        /* CAN2 also carries the chassis control stream. Keep the same
         * 20 ms slot schedule used by the working fusion-upgrade build. */
        can2_slot++;
        if (can2_slot >= 20U) can2_slot = 0U;
        switch (can2_slot)
        {
          case 0U: CAN_SendRcBasic(get_remote_control_point()); break;
          case 2U: CAN_SendRcAux(get_remote_control_point()); break;
          case 4U: CAN_SendRcMouse(get_remote_control_point()); break;
          case 8U: CAN_CmdBoard3(get_navigation_data_point()); break;
          default: break;
        }
					
			
				 if(Frequency >= 10)
    {
        Frequency = 0;
    }
    }
		
		if(htim->Instance == TIM6)
		{
			CAN_CmdBoard5(Vision_state);
 		}

   
}

/*
增益系数：T = 0.001
转动惯量：J = 1
摩擦系数：f = 1
角速度/力矩：G（s） = 1/（s+1）
前馈环节：Gf(s) = s+1
输出：out = in' + in = (in - last_in)/T + in;
*/
float last_in = 0;
float T = 0.0001;
float forwardfeed(float in)
{
	float out;
	out = (in - last_in)/T + in;
	last_in = in;
	return out;
}

float last_in_p = 0;
float T_p = 0.0001;
float forwardfeed_p(float in)
{
	float out;
	out = (in - last_in_p)/T_p + in;
	last_in_p = in;
	return out;
}

void Feedforward_Init(Feedforward_t *ff, float kf_init)
{
    ff->kf = kf_init;
    ff->last_in = 0.0f;
}

float Feedforward_Update(Feedforward_t *ff, float in)
{
    float err = in - ff->last_in;
    float out = ff->kf * err;
    ff->last_in = in;
    return out;
}


