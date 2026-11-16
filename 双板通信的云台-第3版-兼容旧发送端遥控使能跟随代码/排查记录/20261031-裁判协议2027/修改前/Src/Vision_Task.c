#include "Vision_Task.h"
#include "my_pid.h"
#include "SolveTrajectory.h"
#include "Vision.h"
#include "usb_task.h"
#include "bsp_pid.h"
#include "pid.h"

//#include "CAN_receive.h"

extern gimbal_control_t gimbal_control;
extern float add_angle;
uint8_t shoot_control;
//outpost_data_t outpost_data;
VisionData_t VisionData;
extern  SolveTrajectoryParams st;
static float PitchANglePID[3] = {15.0, 0.1, 0.3};//16，0，0
static float PitchSpeedPID[3] = {8000.0, 0.4, 0.9};//11000，0，100
fp32 add_angle = 0;
//视觉PID
static float Yaw2ANglePID[3] = {12, 0.0, 0};//35 0 0.02//25 0 0.022//18//10 0 0.3
static float Yaw2SpeedPID[3] = {7000.0, 0, 150};//13500 0.01 320//12000 0 300//11500 0 110
static float YawANglePID[3] = {15, 0.0, 0.02};//35 0 0.02//25 0 0.022//18//10 0 0.3
static float YawSpeedPID[3] = {7000, 0, 0.01};//13500 0.01 320//12000 0 300//11500 0 110
void VisionPIDInit(void); 
extern float allow_fire_ang_max, allow_fire_ang_min;
extern uint8_t armor_choose;
float ff_yaw_init = 800.0f;
float ff_pitch_init = 0.0f;
Feedforward_t feedforward_yaw;
Feedforward_t feedforward_pitch;
extern float forwardfeed(float in);
extern float forwardfeed_p(float in);
TickType_t execution_time;
uint8_t Vision_state = 0;
pid_type_def VisionPitchSpeedPID;
 gimbal_PID_t VisionPitchAnglePID, VisionYaw2AnglePID,VisionYawAnglePID;
extern MotionPredictor pred_st;
// pid_type_def VisionPitchAnglePID, VisionYawAnglePID;
//pid_type_def VisionYawSpeedPID;
pid_type_def VisionYaw2SpeedPID;
pid_type_def VisionYawSpeedPID;

extern float yaw_add_angle;
extern float  pitch_can_send;             // 俯仰CAN发送值
extern float  yaw_can_send;             // 俯仰CAN发送值
extern float  yaw2_can_send;   

Low_Pass_Filter_t v_yaw_Filter;
SMC YawSMC;
void Vision_task(void const *pvParameters)
{
//	const TickType_t xFrequency = pdMS_TO_TICKS(20);
//	vTaskDelay(201);
	TickType_t start_time;
	TickType_t end_time;
	VisionPIDInit();
	Vision_init();
	Feedforward_Init(&feedforward_yaw,ff_yaw_init);
	Feedforward_Init(&feedforward_pitch,ff_pitch_init);
	Low_Pass_Filter_Init(&v_yaw_Filter,0.0067,0.18);
	SMC_Init(&YawSMC, 20, 120, 0, 0.001f, 25000, 0.8f, 0.5f);


	while (1)
	{
		
		Vision_state = st.mode;
		//Vision_state = 0;
		start_time=	xTaskGetTickCount();
		AutoDataUpdate();  // 视觉控制云台
		end_time=xTaskGetTickCount();
	    execution_time = end_time - start_time;
//		Low_Pass_Filter_OUT(&v_yaw_Filter,st.v_yaw);
//		YawSMC.ref = VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle_set;
//		SMC_Tick(&YawSMC,VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle, VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro);
		vTaskDelay(2);
		

		
	}
}
 
float SetYaw = 0, SetPitch = 0;
int Off_Vision;
uint16_t Red_outpost_HP=0,Blue_outpost_HP=0;
// int sing_data;
//void AutoDataUpdate(void)
//{

//  VisionData.get_angle_point = get_INS_angle_point();
//	VisionData.get_gyro_point  = get_gyro_data_point();
//	VisionData.RcCtrl          = get_remote_control_point();
//	VisionData.GetGimbalData = get_gimlal_data();

//		if(abs(VisionData.GetGimbalData->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm) >= 50)
//		{
//			yaw_add_angle = 0.07;
//		}
//		else
//		{
//			yaw_add_angle = 0;
//		}
//			if(UseVision())
//				{		
// //&&((Refree_data.Vision_ID == 0&&shoot_data.red_outpost_HP==0)||(shoot_data.ID == 1&&shoot_data.blue_outpost_HP==0))))	
//					{
// 						autoSolveTrajectory(&VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle_set, &VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle_set ,&aim_x, &aim_y, &aim_z);
//						VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&VisionPitchAnglePID, VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle, VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle_set, VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro);
//						PID_calc(&VisionPitchSpeedPID, VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro, VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro_set);//7500
//						VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&VisionYawAnglePID, VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle, VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle_set, VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro);
//						PID_calc(&VisionYawSpeedPID, VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro, VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro_set);
//					} 
//					{
//						Off_Vision++;
//					}
//		if(Off_Vision >= 40)
//			{
//				Follow_Vision = 0;		 
//				Off_Vision = 0;
//			}	

//				
////		VisionData.GetGimbalData->gimbal_pitch_motor.vision_current_set = (int16_t)VisionPitchSpeedPID.out + Feedforward_Update(&feedforward_pitch ,VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle_set);//+ forwardfeed_p(VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle_set);
//		VisionData.GetGimbalData->gimbal_pitch_motor.vision_current_set = (int16_t)VisionPitchSpeedPID.out;
//		VisionData.GetGimbalData->gimbal_yaw_motor.vision_current_set = (int16_t)VisionYawSpeedPID.out + Feedforward_Update(&feedforward_yaw , VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle_set);
//		if(VisionData.GetGimbalData->gimbal_yaw_motor.vision_current_set >= 28000)
//			VisionData.GetGimbalData->gimbal_yaw_motor.vision_current_set = 28000;

//		}
//}
/**
 * @brief 自动数据更新函数
 * @note 处理视觉数据、PID计算、云台控制量更新
 */
float  look_yaw_Yu=0.0001,look_pitch_Yu=0.0001,app_change=1;
void AutoDataUpdate(void)
{
    // 获取传感器和控制数据
    VisionData.get_angle_point = get_INS_angle_point();    // 获取INS角度数据
    VisionData.get_gyro_point  = get_gyro_data_point();     // 获取陀螺仪数据
    VisionData.RcCtrl          = get_remote_control_point();// 获取遥控器数据
    VisionData.GetGimbalData   = get_gimlal_data();         // 获取云台数据

    // 无视觉目标时直接返回
    if (Vision_state == 0)
    {
        shoot_control = 0;
        return;
    }
    
    // 当前云台角度（弧度）
    float yaw2_now   = VisionData.GetGimbalData->gimbal_yaw2_motor.absolute_angle;
    float pitch_now = VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle;

    // 视觉指令角度（直接使用接收值，需确保单位统一）
    float yaw2_cmd   = ReceivedPacketVision.yaw;  
    float pitch_cmd = ReceivedPacketVision.pitch;
//	float yaw_cmd = VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle;
	    //float yaw_cmd   = 0;  z

    // 设置角度目标值
    VisionData.GetGimbalData->gimbal_yaw2_motor.absolute_angle_set   = yaw2_cmd;
    VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle_set = pitch_cmd;
    VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle_set   = yaw2_cmd;

 
    // 角度环PID计算 -> 速度环目标值
    VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro_set =
        gimbal_PID_calc(&VisionPitchAnglePID,
                        VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle,
                        VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle_set,
                        VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro);

    VisionData.GetGimbalData->gimbal_yaw2_motor.motor_gyro_set =
        gimbal_PID_calc(&VisionYaw2AnglePID,
                        VisionData.GetGimbalData->gimbal_yaw2_motor.absolute_angle,
                        VisionData.GetGimbalData->gimbal_yaw2_motor.absolute_angle_set,
                        VisionData.GetGimbalData->gimbal_yaw2_motor.motor_gyro);

	    VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro_set =
        gimbal_PID_calc(&VisionYawAnglePID,
                        VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle,
                        VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle_set,
                        VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro);
 

//VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro_set+=ReceivedPacketVision.yaw_vel*look_yaw_Yu;
//VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro_set+=ReceivedPacketVision.yaw_vel;

    // 速度环PID计算 -> 电流控制值
//    PID_calc(&VisionYawSpeedPID,
//             VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro,
//             VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro_set);
//						 
//    // 设置偏航电机电流目标值
//    VisionData.GetGimbalData->gimbal_yaw_motor.current_set = (int16_t)VisionYawSpeedPID.out;
	// VisionData.GetGimbalData->gimbal_yaw_motor.current_set = 0;
    // 视觉模式判断 
    // 模式定义：
    //   0: 无目标
    //   1: 跟踪目标（仅云台跟随，不射击）
    //   2: 打击目标（云台跟随+允许射击）
    if (Vision_state == 2)
    {
        shoot_control = 1;   // 允许射击
    }
    else
    {
        shoot_control = 0;   // 禁止射击
    }

    // 俯仰轴控制量（用于CAN发送）
//  	    VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro_set =gimbal_PID_calc(&VisionPitchAnglePID,
//                        VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle,
//                        VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle_set,
//                        VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro);

	  pitch_can_send =  PID_calc(&VisionPitchSpeedPID,
             VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro,
             VisionData.GetGimbalData->gimbal_pitch_motor.motor_gyro_set);
	
	yaw_can_send =  PID_calc(&VisionYawSpeedPID,
             VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro,
             VisionData.GetGimbalData->gimbal_yaw_motor.motor_gyro_set);
	yaw2_can_send =  PID_calc(&VisionYaw2SpeedPID,
             VisionData.GetGimbalData->gimbal_yaw2_motor.motor_gyro,
             VisionData.GetGimbalData->gimbal_yaw2_motor.motor_gyro_set);
	
		//pitch_can_send+=look_pitch_Yu*ReceivedPacketVision.pitch_vel;
		pitch_can_send+=ReceivedPacketVision.pitch_vel;
		yaw_can_send+=ReceivedPacketVision.yaw_vel;
		yaw2_can_send+=ReceivedPacketVision.yaw_vel;
			//yaw_can_send=yaw2_can_send*app_change;
}

void VisionPIDInit(void)
{
	gimbal_PID_init(&VisionPitchAnglePID, 15.0, 0, PitchANglePID[0], PitchANglePID[1], PitchANglePID[2]);
	PID_init(&VisionPitchSpeedPID, PID_POSITION, PitchSpeedPID, 25000.0f, 5000.0f);
	gimbal_PID_init(&VisionYaw2AnglePID, 30.0, 0.5, Yaw2ANglePID[0], Yaw2ANglePID[1], Yaw2ANglePID[2]);
	PID_init(&VisionYaw2SpeedPID, PID_POSITION, Yaw2SpeedPID, 25000.0f, 5000.0f);
	gimbal_PID_init(&VisionYawAnglePID,10.0,0,YawANglePID[0],YawANglePID[1],YawANglePID[2]);
	PID_init(&VisionYawSpeedPID,PID_POSITION,YawSpeedPID,25000.0f,5000.0f);
}

	float pitch_range,yaw_range;
//void Vision_shoot_control(void)
//{
//	 	pitch_range=get_delta_ang_pi(VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle_set,SendPacketVision.pitch);
//	  yaw_range=get_delta_ang_pi(VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle_set,SendPacketVision.yaw);
//	
////  if( yaw_range<allow_fire_ang_max &&yaw_range>allow_fire_ang_min && (ReceivedPacketVision.state==1)&&armor_choose ==1)
////	{
////	   shoot_control=1;
////		
////	}
////	else
////	{
////	 shoot_control=0;
////	}
//	
//    float lower_bound = pred_st.filtered_yaw + allow_fire_ang_min; // 负偏移 -> 左边界
//    float upper_bound = pred_st.filtered_yaw + allow_fire_ang_max; // 正偏移 -> 右边界
//    if (lower_bound <= upper_bound) {
//        shoot_control = (SendPacketVision.yaw >= lower_bound && SendPacketVision.yaw <= upper_bound) ? 1 : 0;
//    } else {
//        shoot_control = (SendPacketVision.yaw <= lower_bound || SendPacketVision.yaw >= upper_bound) ? 1 : 0;
//    }
//	
//	
//}

