#include "shoot_task.h"
#include "Filter.h"
#include "bsp_laser.h"
#include "Vision_Task.h"
#include "CAN_receive.h"
#include "SolveTrajectory.h"

#define shoot_laser_on() laser_on()   // 激光开启宏定义
#define shoot_laser_off() laser_off() // 激光关闭宏定义
ShootControl_t ShootControl;

Low_Pass_Filter_t ShootMotor_Left, ShootMotor_Right;

TriggerControlMode trigger_mode_e = TRIGGER_HEAT_LIMIT_MODE; // 默认使用热量限制


int Single_Data=36864;
int ShotTime =0;
int firetime=80;
uint16_t poke_spd_ref;
pid_type_def trigger_speed_motor_pid, trigger_Angle_motor_pid;
pid_type_def Fire_left_motor_pid;
pid_type_def Fire_right_motor_pid;
pid_type_def FireLeftMoment;
pid_type_def FireRightMoment;

extern gimbal_control_t gimbal_control;
extern game_state_t game_statee;
extern refree_data_t Refree_data;

fp32 Fire_left_speed_pid[3] = {13.0, 0.0, 10.0};
fp32 Fire_right_speed_pid[3] = {13.0, 0.0, 10.0};
fp32 FireLeftMomentPID[3] = {0.5, 0., 0.3};
fp32 FireRightMomentPID[3] = {0.5, 0., 0.3};
const fp32 TriggerAnglePID[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
const fp32 TriggerSpeedPID[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};

static void trigger_motor_turn_back(void);
extern 
void ShootInit(void)
{
    ShootControl.ShootMode = SHOOT_STOP;
    ShootControl.last_rc_s = RC_SW_UP;
    ShootControl.last_rc_T = RC_SW_DOWN;

    ShootControl.LeftFireData = get_shoot_fire_left_point();
    ShootControl.RightFireData = get_shoot_fire_right_point();
    ShootControl.TriggerData = get_shoot_trigger_measure_point();
    ShootControl.RC_data = get_remote_control_point();
	ShootControl.block_time = 0;
	ShootControl.TriggerMotor_pos_set = ShootControl.TriggerData->total_angle;
	
    PID_init(&trigger_speed_motor_pid, PID_POSITION, TriggerSpeedPID, BULLET_ANG_PID_MAX_OUT, BULLET_ANG_PID_MAX_IOUT);
    PID_init(&trigger_Angle_motor_pid, PID_POSITION, TriggerAnglePID, BULLET_SPEED_PID_MAX_OUT, BUFFET_SPEED_PID_MAX_IOUT);

    PID_init(&Fire_left_motor_pid, PID_POSITION, Fire_left_speed_pid, FIRE_left_SPEED_PID_MAX_OUT, FIRE_left_SPEED_PID_MAX_IOUT);
    PID_init(&Fire_right_motor_pid, PID_POSITION, Fire_right_speed_pid, FIRE_right_SPEED_PID_MAX_OUT, FIRE_right_SPEED_PID_MAX_IOUT);
    PID_init(&FireLeftMoment, PID_POSITION, FireLeftMomentPID, FIRE_MOMENT_PID_MAX_OUT, FIRE_MOMENT_PID_MAX_IOUT);
    PID_init(&FireRightMoment, PID_POSITION, FireRightMomentPID, FIRE_MOMENT_PID_MAX_OUT, FIRE_MOMENT_PID_MAX_IOUT);

    Low_Pass_Filter_Init(&ShootMotor_Left, 2, 1.6);
    Low_Pass_Filter_Init(&ShootMotor_Right, 2, 1.6);
}

void ShootDataUpdata(void)
{
    ShootControl.LeftFireData = get_shoot_fire_left_point();
    ShootControl.RightFireData = get_shoot_fire_right_point();
    ShootControl.TriggerData = get_shoot_trigger_measure_point();
    ShootControl.RC_data = get_remote_control_point();
}

void ShootTask(void const *pvParameters)
{
    vTaskDelay(2);
    ShootInit();
    while (1)
    {
        Set_ShootMode();
        ShootDataUpdata();
        Set_ShootVal();

        CAN_CmdShoot(ShootControl.give_current[2],0, ShootControl.give_current[1], ShootControl.give_current[0]);

        osDelay(1);
    }
}

int8_t LAST_SHOOT_OFF_KEYBOARD = 0;

void Set_ShootMode(void)
{

//   if ((switch_is_up(ShootControl.RC_data->rc.s[RC_Left]) /*&& switch_is_up(ShootControl.RC_data->rc.s[RC_Right])*/ && !switch_is_up(ShootControl.last_rc_s) && ShootControl.ShootMode == SHOOT_STOP)|| 
//										((ShootControl.RC_data->key.v & SHOOT_OPEN_FIRE) && (!(LAST_SHOOT_OFF_KEYBOARD & SHOOT_OPEN_FIRE)) && (ShootControl.ShootMode == SHOOT_STOP)))
////	if(switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[0])&&!switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[1]) )//右边下拨且左边不下拨，防止与校准冲突)
//	{
//		
//         ShootControl.ShootMode = SHOOT_STAR;
//	}		
			
//	if ((switch_is_up(ShootControl.RC_data->rc.s[RC_Left]) /*&& switch_is_up(ShootControl.RC_data->rc.s[RC_Right])*/ && !switch_is_up(ShootControl.last_rc_s) && ShootControl.ShootMode == SHOOT_STOP)|| 
//										((ShootControl.RC_data->key.v & SHOOT_OPEN_FIRE) && (!(LAST_SHOOT_OFF_KEYBOARD & SHOOT_OPEN_FIRE)) && (ShootControl.ShootMode == SHOOT_STOP)))
//    {
//        ShootControl.ShootMode = SHOOT_STAR;
//    }
//    else if ((switch_is_up(ShootControl.RC_data->rc.s[RC_Left]) && !switch_is_up(ShootControl.last_rc_s) && ShootControl.ShootMode != SHOOT_STOP /*&& switch_is_up(ShootControl.RC_data->rc.s[RC_Right])*/)|| 
//										((ShootControl.RC_data->key.v & SHOOT_OPEN_FIRE) && !(LAST_SHOOT_OFF_KEYBOARD & SHOOT_OPEN_FIRE) && (ShootControl.ShootMode != SHOOT_STOP)))
//    {
//        ShootControl.ShootMode = SHOOT_STOP;
//        ShotTime = 0;
//    }
    if(switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[0])&&switch_is_mid(gimbal_control.gimbal_rc_ctrl->rc.s[1]) )//右边下拨且左边不下拨，防止与校准冲突)
	{		
         ShootControl.ShootMode = SHOOT_STAR;
	}		
	else 
	{
         ShootControl.ShootMode = SHOOT_STOP;
		 ShotTime = 0;
	}
//     if ((switch_is_up(ShootControl.RC_data->rc.s[RC_Left]) && !switch_is_up(ShootControl.last_rc_s) && ShootControl.ShootMode != SHOOT_STOP /*&& switch_is_up(ShootControl.RC_data->rc.s[RC_Right])*/)|| 
//										((ShootControl.RC_data->key.v & SHOOT_OPEN_FIRE) && !(LAST_SHOOT_OFF_KEYBOARD & SHOOT_OPEN_FIRE) && (ShootControl.ShootMode != SHOOT_STOP)))
//    {   
//		ShootControl.ShootMode = SHOOT_STOP;
//        ShotTime = 0;
//    }
		if(ShootControl.RC_data->key.v&KEY_PRESSED_OFFSET_F)
		{
			TOP_CHASSIS=0.2f;
		
		}
		if(ShootControl.RC_data->key.v&KEY_PRESSED_OFFSET_R)
		{
			TOP_CHASSIS=0.0f;
		}
        ShootControl.last_rc_s = ShootControl.RC_data->rc.s[RC_Left];
		LAST_SHOOT_OFF_KEYBOARD=ShootControl.RC_data->key.v;

}

// 计算热量允许的最大转速
static float calc_trigger_max_speed_by_heat(void)
{
    float heat_limit = Refree_data.shooter_barrel_heat_limit;        // 热量上限
    float heat_current = Refree_data.shooter_17mm_barrel_heat;       // 当前热量
    float heat_remaining = heat_limit - heat_current;
    if (heat_remaining < 0) heat_remaining = 0;

    // 线性映射：热量剩余比例 * 最大转速
    float max_speed = TRIGGER_MAX_SPEED * (heat_remaining / heat_limit);
    
    // 加入余量，剩余热量小于10%时强制为0
    if (heat_remaining < heat_limit * 0.1f) {
        max_speed = 0;
    }
    return max_speed;
}

float speed_target;
void Shoot_PIDCalc(void)
{

    Low_Pass_Filter_OUT(&ShootMotor_Left, ShootControl.LeftFireData->speed_rpm);
    Low_Pass_Filter_OUT(&ShootMotor_Right, ShootControl.RightFireData->speed_rpm);
#if Normal_Mode
    ShootControl.give_current[0] = PID_calc(&Fire_right_motor_pid, ShootControl.RightFireData->speed_rpm, ShootControl.Fire_right_speed_set);
    ShootControl.give_current[1] = PID_calc(&Fire_left_motor_pid, ShootControl.LeftFireData->speed_rpm, ShootControl.Fire_left_speed_set);
#else
    PID_calc(&Fire_right_motor_pid, ShootControl.RightFireData->speed_rpm, ShootControl.Fire_right_speed_set);
    PID_calc(&Fire_left_motor_pid, ShootControl.LeftFireData->speed_rpm, ShootControl.Fire_left_speed_set);

    ShootControl.give_current[0] = PID_calc(&FireRightMoment, ShootControl.RightFireData->given_current, Fire_right_motor_pid.out);
    ShootControl.give_current[1] = PID_calc(&FireLeftMoment, ShootControl.LeftFireData->given_current, Fire_left_motor_pid.out);

#endif
 
//    PID_calc(&trigger_Angle_motor_pid, (fp32)ShootControl.TriggerData->total_angle, (fp32)ShootControl.TriggerMotor_pos_set);
//    ShootControl.give_current[2] = PID_calc(&trigger_speed_motor_pid, ShootControl.TriggerData->speed_rpm, trigger_Angle_motor_pid.out);
//	
	//拨弹盘角度环计算
    float angle_out = PID_calc(&trigger_Angle_motor_pid, (fp32)ShootControl.TriggerData->total_angle, (fp32)ShootControl.TriggerMotor_pos_set);

//		(fp32)ShootControl.TriggerData->total_angle=(fp32)ShootControl.TriggerMotor_pos_set;

    //根据模式确定速度环目标
    float speed_target = angle_out;   // 默认直接使用角度环输出
    if (trigger_mode_e == TRIGGER_HEAT_LIMIT_MODE) {
        // 计算当前允许的最大转速
        float max_speed_allowed = calc_trigger_max_speed_by_heat();
        // 限幅
        if (speed_target > max_speed_allowed) speed_target = max_speed_allowed;
    }

    //速度环计算并输出电流
    ShootControl.give_current[2] = PID_calc(&trigger_speed_motor_pid, ShootControl.TriggerData->speed_rpm, speed_target);
	
//    PID_calc(&trigger_Angle_motor_pid, (fp32)ShootControl.TriggerData->total_angle, (fp32)ShootControl.TriggerMotor_pos_set);
//    ShootControl.give_current[2] = PID_calc(&trigger_speed_motor_pid, ShootControl.TriggerData->speed_rpm, trigger_Angle_motor_pid.out);
}
uint16_t SYSQres;
static void Fire_Ctrl()
{
	SYSQres=Refree_data.shooter_barrel_heat_limit-Refree_data.shooter_17mm_barrel_heat;
	if(SYSQres>100)
	{
		speed_target=19.0f;
	}
	else if((SYSQres>20)&&(SYSQres<100))
	{
		float a=(float)(game_statee.shooter_barrel_cooling_value);
		float m=(float)SYSQres;
		float d=10;
	}	

}	
void Set_ShootVal(void)
{ 	
//	trigger_motor_turn_back();
	
    switch (ShootControl.ShootMode)
    {
    case SHOOT_STOP:
        ShootControl.Fire_left_speed_set = 0;
		
        ShootControl.Fire_right_speed_set = 0;
				    shoot_laser_off(); // 激光开启
        ShootControl.TriggerMotor_pos_set = ShootControl.TriggerData->total_angle;//拨弹盘电机位置为总角度
        Shoot_PIDCalc();
        break;
    case SHOOT_STAR:
        ShootControl.Fire_left_speed_set = 6100.0f;//6300 
        ShootControl.Fire_right_speed_set = -6100.0f;
				    shoot_laser_on(); // 激光开启
//        if (!switch_is_down(ShootControl.last_rc_T) && switch_is_down(ShootControl.RC_data->rc.s[RC_Left])&&shoot_control==1)  //遥控器控制单发
//        {
//            ShootControl.TriggerMotor_pos_set += Single_Data;
//        }
        if (ShootControl.RC_data->rc.ch[4] > 600 /*||(shoot_control == 0)*/)	
		{
		    ShotTime++;
		}  
		
		if(shoot_control==1)
        {
            ShotTime++;
		}
//		else if(shoot_control==0)
//		{
//			ShotTime=0;
//		}	
        if (ShotTime >= SHOOT_FULL_TIME)
        {
            ShootControl.TriggerMotor_pos_set += Single_Data;
            ShotTime = 0; 
        }
//		if((Refree_data.shooter_barrel_heat_limit-Refree_data.shooter_17mm_barrel_heat<40)&&(game_statee.game_progress==4))
//		{
//			Single_Data=0;
//		}
		else
		{
			Single_Data=36864;//8192*36/8
		}	
				
		
}
				// 以毫秒为单位记录当前时间
				static uint32_t last_fire_time = 0;
				const uint32_t SHOOT_INTERVAL_MS = 120;  // 每发间隔时间 = 150ms，对应射频≈6.6Hz，控制射击频率

		
//				// 在视觉判断通过、允许打击的逻辑中：
//				uint32_t now = HAL_GetTick();  // 获取当前系统时间
//////视觉数据包状态为真，射击控制模式为1，距离上次射击时间已经超过间隔、热量条件满足（剩余热量>=40）、且没有阻塞时间
//				if ((now - last_fire_time >= SHOOT_INTERVAL_MS))
//				{
//						ShootControl.TriggerMotor_pos_set += Single_Data - sing_data;
////						poke_spd_ref = ShootControl.TriggerData->total_angle % 36864;

//						last_fire_time = now;  // 更新上次发射时间戳
//					
//				}
        ShootControl.last_rc_T = ShootControl.RC_data->rc.s[RC_Left];
        Shoot_PIDCalc();
    
}
//void Set_ShootVal(void)
//{ 	
//	trigger_motor_turn_back();
//	
//    switch (ShootControl.ShootMode)
//    {
//    case SHOOT_STOP:
//        ShootControl.Fire_left_speed_set = 0;
//		
//        ShootControl.Fire_right_speed_set = 0;
//				    shoot_laser_off(); // 激光开启
//        ShootControl.TriggerMotor_pos_set = ShootControl.TriggerData->total_angle;//拨弹盘电机位置为总角度
//        Shoot_PIDCalc();
//        break;
//    case SHOOT_STAR:
//		
//        ShootControl.Fire_left_speed_set = 6200.0f;//6300 
//        ShootControl.Fire_right_speed_set = -6200.0f;
//				    shoot_laser_on(); // 激光开启
//        if (!switch_is_down(ShootControl.last_rc_T) && switch_is_down(ShootControl.RC_data->rc.s[RC_Left]))  //遥控器控制单发
//        {
//            ShootControl.TriggerMotor_pos_set += Single_Data;
//        }
////        if (ShootControl.RC_data->rc.ch[4] > 600 || ShootControl.RC_data->mouse.press_l /*||(shoot_control == 0)*/)	
//		  if(shoot_control==1)
//        {
//            ShotTime++;
//		}
//        if (ShotTime >= SHOOT_FULL_TIME)
//        {
//            ShootControl.TriggerMotor_pos_set += Single_Data;
//            ShotTime = 0; 
//        }
////		if((Refree_data.shooter_barrel_heat_limit-Refree_data.shooter_17mm_barrel_heat<40)&&(game_statee.game_progress==4))
////		{
////			Single_Data=0;
////		}
//		else
//		{
//			Single_Data=36864;//8192*36/8
//		}	
//				
//		
//}
				// 以毫秒为单位记录当前时间
				static uint32_t last_fire_time = 0;
				const uint32_t SHOOT_INTERVAL_MS = 120;  // 每发间隔时间 = 150ms，对应射频≈6.6Hz，控制射击频率

		
//				// 在视觉判断通过、允许打击的逻辑中：
//				uint32_t now = HAL_GetTick();  // 获取当前系统时间
//////视觉数据包状态为真，射击控制模式为1，距离上次射击时间已经超过间隔、热量条件满足（剩余热量>=40）、且没有阻塞时间
//				if ((now - last_fire_time >= SHOOT_INTERVAL_MS))
//				{
//						ShootControl.TriggerMotor_pos_set += Single_Data - sing_data;
////						poke_spd_ref = ShootControl.TriggerData->total_angle % 36864;

//						last_fire_time = now;  // 更新上次发射时间戳
//					
//				}
//        ShootControl.last_rc_T = ShootControl.RC_data->rc.s[RC_Left];
//        Shoot_PIDCalc();
//    
//}

//void Set_ShootVal(void)
//{ 	
//	trigger_motor_turn_back();
//	
//    switch (ShootControl.ShootMode)
//    {
//    case SHOOT_STOP:
//        ShootControl.Fire_left_speed_set = 0;
//		
//        ShootControl.Fire_right_speed_set = 0;
//				    shoot_laser_off(); // 激光开启
//        ShootControl.TriggerMotor_pos_set = ShootControl.TriggerData->total_angle;//拨弹盘电机位置为总角度
//        Shoot_PIDCalc();
//        break;
//    case SHOOT_STAR:
//		
//        ShootControl.Fire_left_speed_set = 6650.0f;//6300
//        ShootControl.Fire_right_speed_set = -6650.0f;
//				   shoot_laser_on(); // 激光开启
////        if (!switch_is_down(ShootControl.last_rc_T) && switch_is_down(ShootControl.RC_data->rc.s[RC_Left]))  //遥控器控制单发
////        {
////            ShootControl.TriggerMotor_pos_set += Single_Data;
////        }
////        if (ShootControl.RC_data->rc.ch[4] > 600 || ShootControl.RC_data->mouse.press_l /*||(shoot_control == 0)*/)	
//////		  if(shoot_control==1)
////        {
////            ShotTime++;
////		}
////        if (ShotTime >= SHOOT_FULL_TIME)
////        {
////            ShootControl.TriggerMotor_pos_set += Single_Data;
////            ShotTime = 0; 
////        }
////		if((Refree_data.shooter_barrel_heat_limit-Refree_data.shooter_17mm_1_barrel_heat<40)&&(game_statee.game_progress==4))
////		{
////		Single_Data=0;
////		}
////		else
////		{
////		Single_Data=36864;//8192*36/8
////		}	
//				
//}
//				// 以毫秒为单位记录当前时间
//				static uint32_t last_fire_time = 0;
//				const uint32_t SHOOT_INTERVAL_MS = 120;  // 每发间隔时间 = 150ms，对应射频≈6.6Hz，控制射击频率

//		
////				// 在视觉判断通过、允许打击的逻辑中：
////				uint32_t now = HAL_GetTick();  // 获取当前系统时间
////////视觉数据包状态为真，射击控制模式为1，距离上次射击时间已经超过间隔、热量条件满足（剩余热量>=40）、且没有阻塞时间
////				if ((now - last_fire_time >= SHOOT_INTERVAL_MS))
////				{
////						ShootControl.TriggerMotor_pos_set += Single_Data - sing_data;
//////						poke_spd_ref = ShootControl.TriggerData->total_angle % 36864;

////						last_fire_time = now;  // 更新上次发射时间戳
////					
////				}
//        ShootControl.last_rc_T = ShootControl.RC_data->rc.s[RC_Left];
//        Shoot_PIDCalc();
//    
//}

static void trigger_motor_turn_back(void)
{
	if(ShootControl.TriggerData->given_current >=9000)
	{
		ShootControl.TriggerMotor_pos_set = ShootControl.TriggerMotor_pos_set-36864;
	}

}
 
//void Shoot_Heatlimit()
//{



//}
