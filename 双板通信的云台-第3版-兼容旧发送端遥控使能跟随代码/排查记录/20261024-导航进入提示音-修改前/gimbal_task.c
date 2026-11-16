#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "pid.h"
#include "Vision_Task.h"
#include "usart.h"
#include "ahrs.h"
#include "dm_imu.h"

#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid2); 				   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
				gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw2_motor.gimbal_motor_absolute_angle_pid);  \
				gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw2_motor.gimbal_motor_relative_angle_pid);  \
				PID_clear(&(gimbal_clear)->gimbal_yaw2_motor.gimbal_motor_gyro_pid);                   \
    }
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

static void gimbal_init(gimbal_control_t *init);
static void gimbal_cali_fail(uint8_t error);
static void gimbal_cali_zero_outputs(void);

static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
static void big_yaw_follow_control(void);
static void big_yaw_output_limit(void);
static void gimbal_publish_debug_snapshot(void);
static volatile float gimbal_debug_values[GIMBAL_DEBUG_CHANNELS];
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
static bool_t calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
	
#if GIMBAL_TEST_MODE 
static void J_scope_gimbal_test(void);
#endif	

float yawANglePID[3] = {8.5f, 0, 0.9f};
float yawSpeedPID[3] = {9.0f, 0, 1};
float yaw2ANglePID[3] = {8.5f, 0, 0.9f};
float yaw2SpeedPID[3] = {9.0f, 0, 1};
float yaw2ECDPID[3]={0.05f,0.0f,0.01f};

float yaw2MotorPID[3]={460.0f,0.0f,50.0f};
//float Yaw2[3]={10.0f,0,0.1f};
float YawTemp = 0, PitchTemp = 0;
 float  yaw_can_send;
int16_t yaw_can_set_current = 0, pitch_can_set_current = 0,yaw2_can_set_current = 0;
 
uint16_t AUTO;
uint8_t PA_direction = 1;
fp32 add_Pitch = 500;
int sing_data = 0;
//fp32 yuyu;
float  pitch_can_send=0;
//float  yaw_can_send=0;
float  yaw2_can_send=0;
float INS_YAW2_ADDRESS_OFFSET ;
float AutoSpeed;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern uint8_t Vision_state;
extern stage_remain_time_t stage_remain_time;

//云台数据结构	
gimbal_control_t gimbal_control;
gimbal_motor_t gimbal_yaw_motor;
gimbal_motor_t gimbal_yaw2_motor;
gimbal_behaviour_e gimbal_behaviour;
static fp32 yaw_ins_zero_angle;
static bool_t yaw_ins_zero_valid;

volatile float big_follow_kp = 20.0f;
volatile float big_follow_sign = 1.0f;
volatile float big_follow_deadband = 0.02618f;
volatile float big_follow_speed_limit = 2.0;
volatile float big_follow_current_limit = 10000.0f;
volatile uint8_t big_speed_test_enable = 0U;
volatile float big_speed_test_set = 0.0f;
volatile uint8_t dm_gyro_online = 0U;
volatile uint32_t dm_gyro_age_ms = 0xFFFFFFFFU;
/* Require a healthy IMU and both RC switches down before enabling. */
volatile uint8_t dm_fault_latched = 1U;
static uint8_t big_speed_test_was_enabled;
 
//遥控器数据结构
CAN_Send_t CANSendData;

fp32 add_yaw_angle = 0.0f;
fp32 add_pitch_angle = 0.0f;
fp32 add_yaw2_angle = 0.0f;

fp32 add_init_yaw_angle =  0.0f;
fp32 add_init_pitch_angle = 0.0f;
fp32 add_init_yaw2_angle =  0.0f;

fp32 yaw_angle_set = 0;

pid_type_def yaw_pid_angle ;
pid_type_def yaw_pid_speed ;
pid_type_def yaw2_pid_angle ;
pid_type_def yaw2_pid_speed ;
pid_type_def yaw2_ecd;
pid_type_def yaw2_motor;
//pid_type_def yaw2;

//uint16_t YU;
//*******************************//
	
uint8_t ceshi = 2;
	
//云台任务函数	
void gimbal_task(void const *pvParameters)	
{
	//延迟2毫秒，以便其他任务初始化
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
	//初始化云台
    gimbal_init(&gimbal_control);
	//获取遥控器数据
//	CANSendData.RC = get_remote_control_point();
//	gimbal_control.gimbal_yaw2_motor.current_set = gimbal_control.gimbal_yaw2_motor.raw_cmd_current=500;
	//主任务循环
	while(1)
	{
		//获取云台模式
	    GetgimbalMode(); 
		//更新云台数据
		gimbal_feedback_update(&gimbal_control);

        if (!dm_gyro_online || toe_is_error(BOARD_GYRO_TOE) || RC_data_is_error())
        {
            dm_fault_latched = 1U;
        }
        if (dm_gyro_online && !toe_is_error(BOARD_GYRO_TOE) && !RC_data_is_error() &&
            switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[0]) &&
            switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[1]))
        {
            dm_fault_latched = 0U;
        }
        if (gimbal_cali_status == 1U &&
            (dm_fault_latched || gimbal_cali_abort ||
             !switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[0]) ||
             !switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[1])))
        {
            gimbal_cali_fail(gimbal_cali_abort ? 5U : (dm_fault_latched ? 3U : 1U));
        }
        if (dm_fault_latched)
        {
            gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }
		//各种模式下数据的传输
		gimbal_mode_change_control_transit(&gimbal_control);

		
		//根据当前云台模式执行相应的操作
		switch(gimbal_behaviour)
		{
			//无力模式
			case GIMBAL_ZERO_FORCE :
				Gimbal_Zero_Force ();
			break;
			//初始模式
			case GIMBAL_INIT:
				Gimbal_Init();
			break;
			//校准模式
			case GIMBAL_CALI:
				Gimbal_Cali();
			break;
			//绝对角度控制
			case GIMBAL_ABSOLUTE_ANGLE:
				Gimbal_Absolute_Angle();
			break;
			//视觉模式
			case GIMBAL_VISION:
				Gimbal_Vision();
			break;
			//导航模式
			case AUTO_AIM:
				Auto_Aim();
			break;

			default:
				Gimbal_Zero_Force();
			break;     				
		}	
        /* Common gate covers INIT, manual, vision and calibration outputs. */
        if (dm_fault_latched || !dm_gyro_online)
        {
            Gimbal_Zero_Force();
            gimbal_total_pid_clear(&gimbal_control);
            yaw_can_set_current = pitch_can_set_current = yaw2_can_set_current = 0;
        }
        big_yaw_output_limit();

		//引入一个小延迟，以控制任务执行的频率
	    vTaskDelay(GIMBAL_CONTROL_TIME);
	
#if YAW_TURN
        yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
        yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if YAW2_TURN
        yaw2_can_set_current = -gimbal_control.gimbal_yaw2_motor.given_current;
#else
        yaw2_can_set_current = gimbal_control.gimbal_yaw2_motor.given_current;
#endif
		
#if PITCH_TURN
        pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif
				CAN_cmd_gimbal_3(yaw2_can_set_current);		
        gimbal_publish_debug_snapshot();
#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);// 可选：跟踪任务的堆栈高水位标记
#endif
	
	}	

}	

int a=0;
int c=1;
void GetgimbalMode(void)
{
	static gimbal_behaviour_e last_behaviour_for_set = GIMBAL_ZERO_FORCE;
    static uint16_t init_time = 0;
    static uint16_t init_stop_time = 0;
    if (gimbal_behaviour != GIMBAL_INIT || dm_fault_latched)
    {
        init_time = 0;
        init_stop_time = 0;
    }
				 gimbal_behaviour_e previous_behaviour = last_behaviour_for_set;

    /* CALI ownership ends explicitly: END waits at zero for validated saving.
     * step=0 must never reuse a previous raw calibration command. */
    if (gimbal_cali_status == 1U)
    {
        gimbal_behaviour = (gimbal_control.gimbal_cali.step >= GIMBAL_CALI_START_STEP &&
                            gimbal_control.gimbal_cali.step < GIMBAL_CALI_END_STEP) ?
                            GIMBAL_CALI : GIMBAL_ZERO_FORCE;
        return;
    }
    if (!gimbal_cali_valid || gimbal_cali_status == 3U)
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
        last_behaviour_for_set = GIMBAL_ZERO_FORCE;
        return;
    }

    if (gimbal_behaviour == GIMBAL_INIT)
    {
        bool_t init_zero_done;
        init_time++;
        
        if (fabs(gimbal_control.gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_YAW_ZERO_ANGLE_ERROR &&
            fabs(gimbal_control.gimbal_yaw2_motor.relative_angle - INIT_YAW_SET) < GIMBAL_YAW_ZERO_ANGLE_ERROR &&
            fabs(gimbal_control.gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR &&
            !toe_is_error(BOARD_GYRO_TOE) && dm_gyro_online)
        {         
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
            if (init_stop_time == GIMBAL_INIT_STOP_TIME)
            {
                yaw_ins_zero_angle = *(gimbal_control.gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
                yaw_ins_zero_valid = 1;
            }
        }
        else
        {            
            init_stop_time = 0;
        }
        init_zero_done = (init_stop_time >= GIMBAL_INIT_STOP_TIME);

        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[RC_Right]) && !RC_data_is_error())
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
			a = 1;
        }
        if (!init_zero_done)
        {
            gimbal_behaviour = GIMBAL_ZERO_FORCE;
            return;
        }
    }
	
	if(UseVision())
	{
		gimbal_behaviour = AUTO_AIM;  
	}
	 
//1左 0右
   else if (switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[0])&&!switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[1]) &&!UseVision())//右边下拨且左边不下拨，防止与校准冲突
    {
			if(AUTO==1)//外八开启
			{
				gimbal_behaviour =	GIMBAL_VISION;			
			}
			
			else 
			{
				gimbal_behaviour =	GIMBAL_ZERO_FORCE;			
			}
    }
	//左下右下：无力模式
   if (switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[RC_Right])&&switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[1]))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;

    }
	//左下右中：初始模式
	else if(switch_is_mid(gimbal_control.gimbal_rc_ctrl->rc.s[RC_Right])&&switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[1]))
	 {
				gimbal_behaviour=GIMBAL_ABSOLUTE_ANGLE;
	 }	
//	//左下右中：绝对角度
//	else if(switch_is_mid(gimbal_control.gimbal_rc_ctrl->rc.s[RC_Right])&&switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[1]))
//	{
//		gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
//	}
	//左中右中：绝对角度
		else if (switch_is_mid(gimbal_control.gimbal_rc_ctrl->rc.s[RC_Right]) && switch_is_mid(gimbal_control.gimbal_rc_ctrl->rc.s[1]) && !UseVision())
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
	//左上右中：绝对角度
    else if (switch_is_mid(gimbal_control.gimbal_rc_ctrl->rc.s[RC_Right]) && switch_is_up(gimbal_control.gimbal_rc_ctrl->rc.s[1])&&!UseVision())
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
//		else if (switch_is_mid(gimbal_control.gimbal_rc_ctrl->rc.s[RC_Right]) && switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[1])&&!UseVision())
//    {
//        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
//    }
	//右上：绝对角度
    else if(switch_is_up(gimbal_control.gimbal_rc_ctrl->rc.s[RC_Right])&&!UseVision())
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }


	
	//遥控器错误处理    
    /* CAN remote control is received from the navigation board. */
    if (RC_data_is_error())
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

    {
        static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
        }
        last_gimbal_behaviour = gimbal_behaviour;
    }
 
	 // sync setpoints when entering absolute mode to avoid jumping to the last saved pose
    if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE && previous_behaviour != GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_yaw2_motor.absolute_angle_set = gimbal_control.gimbal_yaw2_motor.absolute_angle;
        gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.absolute_angle;
    }

    last_behaviour_for_set = gimbal_behaviour;

}	

//云台初始化函数
static void gimbal_init(gimbal_control_t *init)
{
    /*PID DATA*/
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    static const fp32 Yaw2_speed_pid[3] = {YAW2_SPEED_PID_KP, YAW2_SPEED_PID_KI, YAW2_SPEED_PID_KD};
    static const fp32 Yaw_speed2_pid[3] = {YAW_SPEED2_PID_KP, YAW_SPEED2_PID_KI, YAW_SPEED2_PID_KD};
		static const fp32 Yaw_speed_pid_auto[3] = {YAW_SPEED_PID_KP_AUTO,YAW_SPEED_PID_KI_AUTO,YAW_SPEED_PID_KD_AUTO};
 
    gimbal_total_pid_clear(init);
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
		init->gimbal_yaw2_motor.gimbal_motor_measure = get_yaw2_gimbal_motor_measure_point();
    init->gimbal_yaw_motor.offset_ecd = GIMBAL_YAW_OFFSET_ECD;
    if (!gimbal_cali_valid)
        init->gimbal_yaw2_motor.offset_ecd = GIMBAL_YAW2_OFFSET_ECD;

	
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
		PID_init(&yaw_pid_angle, POSITION_PID, yawANglePID, 20000, 0);  //1.8  0  1.5       
    PID_init(&yaw_pid_speed, POSITION_PID, yawSpeedPID,20000, 0); 
		PID_init(&yaw2_pid_angle, POSITION_PID, yaw2ANglePID, 20000, 0);  //1.8  0  1.5       
    PID_init(&yaw2_pid_speed, POSITION_PID, yaw2SpeedPID,20000, 0); 	 
//		PID_init(&yaw2_motor,PID_POSITION,yaw2MotorPID,10000,0);
//	PID_init(&yaw2_ecd,PID_POSITION,yaw2ECDPID,10,0);
//	PID_init(&yaw2_ecd,POSITION_PID,yaw2ECDPID,20000,0);

	//获取遥控器数据
	CANSendData.RC = get_remote_control_point();
	
    init->gimbal_rc_ctrl = get_remote_control_point();

    init->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_yaw2_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
	
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_yaw2_motor.gimbal_motor_mode = init->gimbal_yaw2_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
        /* Preserve the old effective 8500/0/10 gains, rather than jumping to 20000. */
		PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
		PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid2, PID_POSITION, Yaw_speed2_pid, YAW_SPEED2_PID_MAX_OUT, YAW_SPEED2_PID_MAX_IOUT);
	//yaw导航
		gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid_auto,YAW_GYRO_ABSOLUTE_PID_MAX_OUT,YAW_GYRO_ABSOLUTE_PID_MAX_IOUT,YAW_GYRO_ABSOLUTE_PID_KP_AUTO,YAW_GYRO_ABSOLUTE_PID_KI_AUTO,YAW_GYRO_ABSOLUTE_PID_KD_AUTO);
		PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid_auto, PID_POSITION, Yaw_speed_pid_auto, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

//	PID_init(&init->gimbal_pitch_motor.gimbal_motor_lock_pid2,PID_POSITION, Yaw_lock_pid2, YAW_GYRO_LOCK_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);

		gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

		gimbal_PID_init(&init->gimbal_yaw2_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW2_GYRO_ABSOLUTE_PID_KP, YAW2_GYRO_ABSOLUTE_PID_KI, YAW2_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->gimbal_yaw2_motor.gimbal_motor_relative_angle_pid, YAW2_ENCODE_RELATIVE_PID_MAX_OUT, YAW2_ENCODE_RELATIVE_PID_MAX_IOUT, YAW2_ENCODE_RELATIVE_PID_KP, YAW2_ENCODE_RELATIVE_PID_KI, YAW2_ENCODE_RELATIVE_PID_KD);    
		PID_init(&init->gimbal_yaw2_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw2_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);


    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
		init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;

		init->gimbal_yaw2_motor.absolute_angle_set = init->gimbal_yaw2_motor.absolute_angle;
    init->gimbal_yaw2_motor.relative_angle_set = init->gimbal_yaw2_motor.relative_angle;
    init->gimbal_yaw2_motor.motor_gyro_set = init->gimbal_yaw2_motor.motor_gyro;

}

//更新云台数据函数
//该函数从传感器（IMU 电机编码器）中读取原始数据，经过坐标变换和符号处理，转换为所需的绝对角度，相对角度和陀螺仪角速度
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{

    if (feedback_update == NULL)
    {
        return;
    }
		CANSendData.RC = get_remote_control_point();
				
	AUTO_Mode();
//直接读取IMU输出的俯仰角(INS_PITCH_ADDRESS_OFFSET指向欧拉角中的俯仰角),该角度相对于世界坐标系	
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
//相对角度通过这个函数计算编码器与偏移的差,将电机编码器值转换为机械角度，并减去初始化时记录的偏移量，得到电机相对于初始位置的转角
#if PITCH_TURN
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                    feedback_update->gimbal_pitch_motor.offset_ecd);
#else

    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                   feedback_update->gimbal_pitch_motor.offset_ecd);
#endif
//俯仰电机的角速度为IMU的Y轴角速度
    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
//两个yaw轴电机的绝对角度都为IMU输出的偏航角(航向角)
    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	if (yaw_ins_zero_valid)
	{
		feedback_update->gimbal_yaw_motor.absolute_angle = rad_format(feedback_update->gimbal_yaw_motor.absolute_angle - yaw_ins_zero_angle);
	}
	
	feedback_update->gimbal_yaw2_motor.absolute_angle = feedback_update->gimbal_yaw_motor.absolute_angle;


#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(
        feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd, feedback_update->gimbal_yaw_motor.offset_ecd);
#else
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(
        feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd, feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
#if YAW2_TURN
    feedback_update->gimbal_yaw2_motor.relative_angle = -motor_ecd_to_angle_change(
        feedback_update->gimbal_yaw2_motor.gimbal_motor_measure->ecd, feedback_update->gimbal_yaw2_motor.offset_ecd);
#else
    feedback_update->gimbal_yaw2_motor.relative_angle = motor_ecd_to_angle_change(
        feedback_update->gimbal_yaw2_motor.gimbal_motor_measure->ecd, feedback_update->gimbal_yaw2_motor.offset_ecd);
#endif

//机体坐标系转换为云台坐标系
    {
        float gyro[3];
        uint32_t age;
        dm_gyro_online = imu_get_gyro_snapshot(gyro, &age);
        dm_gyro_age_ms = age;
        feedback_update->gimbal_yaw_motor.motor_gyro =
            dm_gyro_online ? DM_IMU_YAW_GYRO_SIGN * gyro[2] : 0.0f;
    }
    feedback_update->gimbal_yaw2_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)) - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));

}
 
//云台无力模式函数
void Gimbal_Zero_Force(void)
{
    big_speed_test_enable = 0U;
    big_speed_test_set = 0.0f;
    big_speed_test_was_enabled = 0U;
    gimbal_control.gimbal_yaw_motor.motor_gyro_set = 0.0f;
    PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid);
   gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
	 gimbal_control.gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
	 gimbal_control.gimbal_yaw2_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;

	 add_yaw_angle = 0.0f;
	 add_pitch_angle = 0.0f;
	 add_yaw2_angle = 0.0f;

	 gimbal_control.gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
	 gimbal_control.gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
	 gimbal_control.gimbal_yaw2_motor.raw_cmd_current = add_yaw2_angle;

//将原始命令电流值赋给电流设置值,将电流设置值转换为整型后赋给最终给定电流
	 gimbal_control.gimbal_yaw_motor.current_set = gimbal_control.gimbal_yaw_motor.raw_cmd_current;
	 gimbal_control.gimbal_yaw_motor.given_current  = (int16_t)(gimbal_control.gimbal_yaw_motor.current_set);
	
	 gimbal_control.gimbal_yaw2_motor.current_set = gimbal_control.gimbal_yaw2_motor.raw_cmd_current;
	 gimbal_control.gimbal_yaw2_motor.given_current  = (int16_t)(gimbal_control.gimbal_yaw2_motor.current_set);
	
	 gimbal_control.gimbal_pitch_motor.current_set = gimbal_control.gimbal_yaw_motor.raw_cmd_current;
	 gimbal_control.gimbal_pitch_motor.given_current  = (int16_t)(gimbal_control.gimbal_yaw_motor.current_set);
	
}	
	

//云台初始化函数
 void Gimbal_Init(void)
{		
    /* Calibration can change the encoder zero. Start from the measured
     * relative angle instead of chasing a target from the previous mode. */
    if (gimbal_control.gimbal_pitch_motor.gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_control.gimbal_pitch_motor.relative_angle_set =
            gimbal_control.gimbal_pitch_motor.relative_angle;
    }
	if (gimbal_control.gimbal_yaw_motor.gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_control.gimbal_yaw_motor.relative_angle_set = gimbal_control.gimbal_yaw_motor.relative_angle;
	}
	if (gimbal_control.gimbal_yaw2_motor.gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_control.gimbal_yaw2_motor.relative_angle_set = gimbal_control.gimbal_yaw2_motor.relative_angle;
	}
		gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;  
    gimbal_control.gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
		gimbal_control.gimbal_yaw2_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;  
		
	if (fabs(INIT_PITCH_SET - gimbal_control.gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        add_init_pitch_angle= (INIT_PITCH_SET - gimbal_control.gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        add_init_yaw_angle = (INIT_YAW_SET - gimbal_control.gimbal_yaw_motor.relative_angle_set) * GIMBAL_INIT_YAW_SPEED;

				add_init_yaw2_angle = (INIT_YAW_SET - gimbal_control.gimbal_yaw2_motor.relative_angle_set) * GIMBAL_INIT_YAW_SPEED;

    }
    else
    {
        add_init_pitch_angle = (INIT_PITCH_SET - gimbal_control.gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        add_init_yaw_angle = (INIT_YAW_SET - gimbal_control.gimbal_yaw_motor.relative_angle_set) * GIMBAL_INIT_YAW_SPEED;		
        add_init_yaw2_angle = (INIT_YAW_SET - gimbal_control.gimbal_yaw2_motor.relative_angle_set) * GIMBAL_INIT_YAW_SPEED;
    }
		
    gimbal_control.gimbal_yaw_motor.relative_angle_set += add_init_yaw_angle;
//		gimbal_control.gimbal_pitch_motor.relative_angle_set += add_init_pitch_angle;
		fp32 pitch_min;
    fp32 pitch_max;
    fp32 pitch_set;

    pitch_min =
        gimbal_control.gimbal_pitch_motor.min_relative_angle;
    pitch_max =
        gimbal_control.gimbal_pitch_motor.max_relative_angle;

    /* 参数无效时，本次初始化不输出驱动力 */
    if (!(pitch_min < pitch_max))
    {
        Gimbal_Zero_Force();
        gimbal_total_pid_clear(&gimbal_control);
        return;
    }

    pitch_set =
        gimbal_control.gimbal_pitch_motor.relative_angle_set
        + add_init_pitch_angle;

    if (pitch_set > pitch_max)
    {
        pitch_set = pitch_max;
    }
    else if (pitch_set < pitch_min)
    {
        pitch_set = pitch_min;
    }

    gimbal_control.gimbal_pitch_motor.relative_angle_set =
        pitch_set;
		gimbal_control.gimbal_yaw2_motor.relative_angle_set += add_init_yaw2_angle;
	
		gimbal_control.gimbal_yaw_motor.motor_gyro_set= gimbal_PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid,gimbal_control.gimbal_yaw_motor.relative_angle, gimbal_control.gimbal_yaw_motor.relative_angle_set, gimbal_control.gimbal_yaw_motor.motor_gyro);
    gimbal_control.gimbal_yaw_motor.current_set = PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, gimbal_control.gimbal_yaw_motor.motor_gyro, gimbal_control.gimbal_yaw_motor.motor_gyro_set);
//    gimbal_control.gimbal_yaw_motor.current_set = PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, gimbal_control.gimbal_yaw_motor.motor_gyro, gimbal_control.gimbal_yaw_motor.motor_gyro_set);
		gimbal_control.gimbal_yaw_motor.given_current = (int16_t)(gimbal_control.gimbal_yaw_motor.current_set);
		
		gimbal_control.gimbal_pitch_motor.motor_gyro_set= gimbal_PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid,gimbal_control.gimbal_pitch_motor.relative_angle, gimbal_control.gimbal_pitch_motor.relative_angle_set, gimbal_control.gimbal_pitch_motor.motor_gyro);
    gimbal_control.gimbal_pitch_motor.current_set = PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid, gimbal_control.gimbal_pitch_motor.motor_gyro, gimbal_control.gimbal_pitch_motor.motor_gyro_set);
    gimbal_control.gimbal_pitch_motor.given_current = (int16_t)(gimbal_control.gimbal_pitch_motor.current_set);
	
		gimbal_control.gimbal_yaw2_motor.motor_gyro_set= gimbal_PID_calc(&gimbal_control.gimbal_yaw2_motor.gimbal_motor_relative_angle_pid,gimbal_control.gimbal_yaw2_motor.relative_angle, gimbal_control.gimbal_yaw2_motor.relative_angle_set, gimbal_control.gimbal_yaw2_motor.motor_gyro);
    gimbal_control.gimbal_yaw2_motor.current_set = PID_calc(&gimbal_control.gimbal_yaw2_motor.gimbal_motor_gyro_pid, gimbal_control.gimbal_yaw2_motor.motor_gyro, gimbal_control.gimbal_yaw2_motor.motor_gyro_set);
    gimbal_control.gimbal_yaw2_motor.given_current = (int16_t)(gimbal_control.gimbal_yaw2_motor.current_set);
	
}


//云台校准函数
/* Pitch and limited small-Yaw calibration. Big-Yaw never receives drive here.
 * A low-speed detector cannot distinguish a hard stop from excessive friction;
 * validate both travel and geometry before accepting the recorded endpoints. */
volatile uint8_t gimbal_cali_status = 0U; /* 0 needs cal, 1 running, 2 done, 3 error, 4 loaded */
volatile uint8_t gimbal_cali_error = 0U;  /* 1 interlock, 2 timeout, 3 feedback, 4 geometry, 5 abort; 6 reserved (old speed abort removed) */
volatile uint8_t gimbal_cali_error_step = 0U; /* Preserve the first failing stage. */
volatile uint8_t gimbal_cali_valid = 0U;
volatile uint8_t gimbal_cali_abort = 0U;
volatile int16_t gimbal_cali_pitch_command = 8000;
volatile int16_t gimbal_cali_yaw2_command = 8000;
volatile uint32_t gimbal_cali_step_elapsed_ms = 0U;
volatile uint32_t gimbal_cali_still_ms = 0U;
volatile uint32_t gimbal_cali_feedback_age_ms = 0xFFFFFFFFU;
volatile int16_t gimbal_cali_motor_rpm = 0;
volatile int32_t gimbal_cali_yaw2_span_counts = 0;

typedef struct
{
    uint8_t step;
    uint16_t last_ecd;
    uint16_t quiet_ecd;
    int32_t travel;
    uint32_t start_ms;
    uint32_t quiet_ms;
} limited_cali_runtime_t;
static limited_cali_runtime_t limited_cali;

static int32_t cali_count_delta(int32_t current, int32_t previous)
{
    int32_t delta = current - previous;
    while (delta > 4096) delta -= 8192;
    while (delta < -4096) delta += 8192;
    return delta;
}

static uint16_t cali_wrap_count(int32_t count)
{
    count %= 8192;
    if (count < 0) count += 8192;
    return (uint16_t)count;
}

static fp32 cali_relative_angle(uint16_t ecd, uint16_t offset, int32_t direction)
{
    return (fp32)(direction * cali_count_delta(ecd, offset)) * MOTOR_ECD_TO_RAD;
}

static bool_t cali_valid_range(fp32 upper, fp32 lower)
{
    /* Comparisons also reject NaN and infinity. The operating zero must fit. */
    return upper > 0.0f && upper < PI && lower < 0.0f && lower > -PI &&
           (upper - lower) > 0.02f;
}

static void gimbal_cali_clear_desired(void)
{
    add_pitch_angle = add_yaw_angle = add_yaw2_angle = 0.0f;
    gimbal_control.gimbal_pitch_motor.raw_cmd_current = 0.0f;
    gimbal_control.gimbal_pitch_motor.current_set = 0.0f;
    gimbal_control.gimbal_pitch_motor.given_current = 0;
    gimbal_control.gimbal_yaw_motor.raw_cmd_current = 0.0f;
    gimbal_control.gimbal_yaw_motor.current_set = 0.0f;
    gimbal_control.gimbal_yaw_motor.given_current = 0;
    gimbal_control.gimbal_yaw2_motor.raw_cmd_current = 0.0f;
    gimbal_control.gimbal_yaw2_motor.current_set = 0.0f;
    gimbal_control.gimbal_yaw2_motor.given_current = 0;
}

static void gimbal_cali_zero_outputs(void)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    gimbal_cali_clear_desired();
    pitch_can_set_current = yaw_can_set_current = yaw2_can_set_current = 0;
    __set_PRIMASK(irq);
}

static void gimbal_cali_fail(uint8_t error)
{
    gimbal_cali_zero_outputs();
    gimbal_cali_valid = 0U;
    if (gimbal_cali_status != 3U)
    {
        gimbal_cali_error = error;
        gimbal_cali_error_step = gimbal_control.gimbal_cali.step;
    }
    gimbal_cali_status = 3U;
    gimbal_control.gimbal_cali.step = 0U;
    limited_cali.step = 0U;
    gimbal_behaviour = GIMBAL_ZERO_FORCE;
    gimbal_total_pid_clear(&gimbal_control);
}

static bool_t gimbal_cali_switches_down(void)
{
    return !RC_data_is_error() &&
        switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[0]) &&
        switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[1]);
}

void Gimbal_Cali(void)
{
    motor_measure_t feedback;
    uint32_t now = HAL_GetTick();
    uint32_t age;
    uint8_t step = gimbal_control.gimbal_cali.step;
    uint8_t pitch = step == GIMBAL_CALI_PITCH_MAX_STEP || step == GIMBAL_CALI_PITCH_MIN_STEP;
    uint8_t yaw2 = step == GIMBAL_CALI_YAW_MAX_STEP || step == GIMBAL_CALI_YAW_MIN_STEP;
    int32_t change;
    int16_t command;
    bool_t quiet;

    /* Clear inactive desired outputs; CAN publishes once after control calculation. */
    gimbal_cali_clear_desired();
    gimbal_control.gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_control.gimbal_yaw2_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    if (gimbal_cali_status != 1U || (!pitch && !yaw2)) return;
    if (gimbal_cali_abort) { gimbal_cali_fail(5U); return; }
    if (!gimbal_cali_switches_down()) { gimbal_cali_fail(1U); return; }
    if (dm_fault_latched || !dm_gyro_online || toe_is_error(BOARD_GYRO_TOE))
    { gimbal_cali_fail(3U); return; }
    if (!gimbal_get_motor_snapshot(pitch ? 0U : 2U, &feedback, &age))
    { gimbal_cali_fail(3U); return; }
    gimbal_cali_feedback_age_ms = age;
    gimbal_cali_motor_rpm = feedback.speed_rpm;
    if (age > GIMBAL_CALI_FEEDBACK_MAX_MS || feedback.ecd >= 8192U)
    { gimbal_cali_fail(3U); return; }

    command = pitch ? gimbal_cali_pitch_command : gimbal_cali_yaw2_command;
    if (command <= 0 || command > (pitch ? GIMBAL_CALI_PITCH_COMMAND_MAX : GIMBAL_CALI_YAW2_COMMAND_MAX))
    { gimbal_cali_fail(1U); return; }
    if (limited_cali.step != step)
    {
        limited_cali.step = step;
        limited_cali.start_ms = now;
        limited_cali.quiet_ms = now;
        limited_cali.quiet_ecd = feedback.ecd;
        limited_cali.travel = 0;
        limited_cali.last_ecd = feedback.ecd;
        if (step == GIMBAL_CALI_YAW_MIN_STEP)
            limited_cali.last_ecd = gimbal_control.gimbal_cali.max_yaw_ecd;
    }
    change = cali_count_delta(feedback.ecd, limited_cali.last_ecd);
    limited_cali.travel += change;
    limited_cali.last_ecd = feedback.ecd;
    gimbal_cali_step_elapsed_ms = now - limited_cali.start_ms;
    if (gimbal_cali_step_elapsed_ms >= GIMBAL_CALI_STEP_TIMEOUT_MS)
    { gimbal_cali_fail(2U); return; }

    /* Use relative motor feedback for small-Yaw. Its C-board gyro can move
     * because of free big-Yaw reaction and is not an independent stop sensor. */
    change = cali_count_delta(feedback.ecd, limited_cali.quiet_ecd);
    quiet = feedback.speed_rpm >= -1 && feedback.speed_rpm <= 1 &&
            change >= -GIMBAL_CALI_STILL_COUNTS && change <= GIMBAL_CALI_STILL_COUNTS;
    if (pitch) quiet = quiet && fabsf(gimbal_control.gimbal_pitch_motor.motor_gyro) < GIMBAL_CALI_GYRO_LIMIT;
    if (!quiet || gimbal_cali_step_elapsed_ms < GIMBAL_CALI_SETTLE_MS)
    {
        limited_cali.quiet_ms = now;
        limited_cali.quiet_ecd = feedback.ecd;
    }
    gimbal_cali_still_ms = now - limited_cali.quiet_ms;
    if (gimbal_cali_still_ms >= GIMBAL_CALI_STOP_MS)
    {
        if (step == GIMBAL_CALI_PITCH_MAX_STEP)
        {
            gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
            gimbal_control.gimbal_cali.max_pitch_ecd = feedback.ecd;
        }
        else if (step == GIMBAL_CALI_PITCH_MIN_STEP)
        {
            gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
            gimbal_control.gimbal_cali.min_pitch_ecd = feedback.ecd;
            if (!(gimbal_control.gimbal_cali.max_pitch - gimbal_control.gimbal_cali.min_pitch >
                  2.0f * GIMBAL_CALI_REDUNDANT_ANGLE + 0.02f))
            { gimbal_cali_fail(4U); return; }
        }
        else if (step == GIMBAL_CALI_YAW_MAX_STEP)
        {
            /* Legacy field names now refer ONLY to the limited small-Yaw. */
            gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw2_motor.absolute_angle;
            gimbal_control.gimbal_cali.max_yaw_ecd = feedback.ecd;
        }
        else
        {
            gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw2_motor.absolute_angle;
            gimbal_control.gimbal_cali.min_yaw_ecd = feedback.ecd;
            gimbal_cali_yaw2_span_counts = (YAW2_TURN ? 1 : -1) * limited_cali.travel;
        }
        gimbal_control.gimbal_cali.step = step + 1U;
        limited_cali.step = 0U;
        gimbal_cali_zero_outputs();
        return; /* Keep every axis at zero on a transition and at END. */
    }

    if (step == GIMBAL_CALI_PITCH_MIN_STEP || step == GIMBAL_CALI_YAW_MIN_STEP) command = -command;
    if (pitch)
    {
        gimbal_control.gimbal_pitch_motor.raw_cmd_current = command;
        gimbal_control.gimbal_pitch_motor.current_set = command;
        gimbal_control.gimbal_pitch_motor.given_current = command;
    }
    else
    {
        gimbal_control.gimbal_yaw2_motor.raw_cmd_current = command;
        gimbal_control.gimbal_yaw2_motor.current_set = command;
        gimbal_control.gimbal_yaw2_motor.given_current = command;
    }
}

void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset,
                         const fp32 max_yaw, const fp32 min_yaw,
                         const fp32 max_pitch, const fp32 min_pitch)
{
    /* This API/Flash slot now stores SMALL-Yaw parameters. Big-Yaw is fixed. */
    if (yaw_offset >= 8192U || pitch_offset >= 8192U ||
        !cali_valid_range(max_yaw, min_yaw) || !cali_valid_range(max_pitch, min_pitch))
    {
        gimbal_cali_valid = 0U;
        gimbal_cali_status = 0U;
        gimbal_cali_error = 4U;
        return;
    }
    gimbal_control.gimbal_yaw_motor.offset_ecd = GIMBAL_YAW_OFFSET_ECD;
    /* These express a wrapped angle, not physical stops of continuous big-Yaw. */
    gimbal_control.gimbal_yaw_motor.max_relative_angle = PI;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = -PI;
    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
    gimbal_control.gimbal_yaw2_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw2_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw2_motor.min_relative_angle = min_yaw;
    gimbal_cali_valid = 1U;
    gimbal_cali_status = 4U;
    gimbal_cali_error = 0U;
}

bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset,
                           fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    motor_measure_t feedback;
    uint32_t age;
    uint16_t new_yaw, new_pitch;
    fp32 new_max_yaw, new_min_yaw, new_max_pitch, new_min_pitch;
    /* A pending callback is not a new RC request. Keep task-side faults latched. */
    if (gimbal_cali_status == 3U) return 0;
    if (gimbal_cali_status != 1U)
    {
        if (gimbal_control.gimbal_pitch_motor.gimbal_motor_measure == NULL ||
            !gimbal_cali_switches_down() || dm_fault_latched || !dm_gyro_online ||
            toe_is_error(BOARD_GYRO_TOE) ||
            !gimbal_get_motor_snapshot(0U, &feedback, &age) || age > GIMBAL_CALI_FEEDBACK_MAX_MS ||
            !gimbal_get_motor_snapshot(2U, &feedback, &age) || age > GIMBAL_CALI_FEEDBACK_MAX_MS)
        { gimbal_cali_fail(1U); return 0; }
        gimbal_cali_zero_outputs();
        gimbal_total_pid_clear(&gimbal_control);
        limited_cali.step = 0U;
        gimbal_cali_valid = 0U;
        gimbal_cali_error = 0U;
        gimbal_cali_error_step = 0U;
        gimbal_cali_abort = 0U;
        gimbal_cali_still_ms = gimbal_cali_step_elapsed_ms = 0U;
        gimbal_cali_yaw2_span_counts = 0;
        gimbal_cali_status = 1U;
        gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
        return 0;
    }
    if (gimbal_control.gimbal_cali.step != GIMBAL_CALI_END_STEP) return 0;
    if (!calc_gimbal_cali(&gimbal_control.gimbal_cali, &new_yaw, &new_pitch,
                         &new_max_yaw, &new_min_yaw, &new_max_pitch, &new_min_pitch))
    { gimbal_cali_fail(4U); return 0; }
    gimbal_cali_zero_outputs();
    *yaw_offset = new_yaw;
    *pitch_offset = new_pitch;
    *max_yaw = new_max_yaw;
    *min_yaw = new_min_yaw;
    *max_pitch = new_max_pitch;
    *min_pitch = new_min_pitch;
    set_cali_gimbal_hook(new_yaw, new_pitch, new_max_yaw, new_min_yaw, new_max_pitch, new_min_pitch);
    gimbal_control.gimbal_cali.step = 0U;
    gimbal_cali_status = 2U;
    return 1; /* Only success allows calibrate_task to write Flash. */
}

static bool_t calc_gimbal_cali(const gimbal_step_cali_t *cali, uint16_t *yaw_offset,
                              uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw,
                              fp32 *max_pitch, fp32 *min_pitch)
{
    int32_t span = gimbal_cali_yaw2_span_counts;
    int32_t yaw_direction = YAW2_TURN ? -1 : 1;
    int32_t pitch_direction = PITCH_TURN ? -1 : 1;
    uint16_t pitch_zero_a, pitch_zero_b;
    fp32 yaw_span, imu_span;
    if (span <= 0 || span >= 8192 || cali == NULL) return 0;
    yaw_span = (fp32)span * MOTOR_ECD_TO_RAD;
    imu_span = cali->max_pitch - cali->min_pitch;
    if (!(yaw_span > 2.0f * GIMBAL_CALI_REDUNDANT_ANGLE + 0.02f && yaw_span < 2.0f * PI - 0.02f) ||
        !(imu_span > 2.0f * GIMBAL_CALI_REDUNDANT_ANGLE + 0.02f && imu_span < PI)) return 0;
    if (cali->max_yaw_ecd >= 8192U || cali->min_yaw_ecd >= 8192U ||
        cali->max_pitch_ecd >= 8192U || cali->min_pitch_ecd >= 8192U) return 0;
    *yaw_offset = cali_wrap_count((int32_t)cali->max_yaw_ecd - yaw_direction * (span / 2));
    /* Estimate the Pitch encoder zero from each measured absolute Pitch angle.
     * This retains the original direct-drive angular model and sign convention. */
    pitch_zero_a = cali_wrap_count((int32_t)cali->max_pitch_ecd - pitch_direction *
                                  (int32_t)(cali->max_pitch / MOTOR_ECD_TO_RAD));
    pitch_zero_b = cali_wrap_count((int32_t)cali->min_pitch_ecd - pitch_direction *
                                  (int32_t)(cali->min_pitch / MOTOR_ECD_TO_RAD));
    if (fabsf((fp32)cali_count_delta(pitch_zero_a, pitch_zero_b)) * MOTOR_ECD_TO_RAD >
        GIMBAL_CALI_PITCH_MODEL_MAX_ERROR) return 0;
    *pitch_offset = cali_wrap_count((int32_t)pitch_zero_a - cali_count_delta(pitch_zero_a, pitch_zero_b) / 2);
    *max_yaw = cali_relative_angle(cali->max_yaw_ecd, *yaw_offset, yaw_direction) - GIMBAL_CALI_REDUNDANT_ANGLE;
    *min_yaw = cali_relative_angle(cali->min_yaw_ecd, *yaw_offset, yaw_direction) + GIMBAL_CALI_REDUNDANT_ANGLE;
    *max_pitch = cali_relative_angle(cali->max_pitch_ecd, *pitch_offset, pitch_direction) - GIMBAL_CALI_REDUNDANT_ANGLE;
    *min_pitch = cali_relative_angle(cali->min_pitch_ecd, *pitch_offset, pitch_direction) + GIMBAL_CALI_REDUNDANT_ANGLE;
    /* Reject a bad direction, inadequate travel, or an inverted shrunken range. */
    return cali_valid_range(*max_yaw, *min_yaw) && cali_valid_range(*max_pitch, *min_pitch) &&
           fabsf((*max_yaw - *min_yaw) - (yaw_span - 2.0f * GIMBAL_CALI_REDUNDANT_ANGLE)) < 0.01f;
}
 

uint16_t b;
fp32 diff=0.0f;
fp32 yaw_relative_angle=0.0f;
//static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
//{
//if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
//{
//	return;
//}
//	 static int16_t  pitch_channel = 0;
// rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
////    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
////    {
////        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
////        *yaw = 0.0f;
////    }
////    else
////    { 
//	*pitch =(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
//	*yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
////    }
//}
/* Disable the additive yaw speed term until motor/feedback polarity is
 * verified. With opposite polarity it becomes positive feedback and causes
 * left-right oscillation after mode entry. */
fp32 add_absolute=0;
//云台绝对角度控制函数
void Gimbal_Absolute_Angle(void)
{
//	fp32 add_yaw_angle = 0.0f;
	fp32 add_pitch_angle = 0.0f;
	fp32 add_yaw2_angle = 0.0f;
	if (gimbal_control.gimbal_yaw2_motor.gimbal_motor_mode != GIMBAL_MOTOR_GYRO)
	{
		gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
		gimbal_control.gimbal_yaw2_motor.absolute_angle_set = gimbal_control.gimbal_yaw2_motor.absolute_angle;
		gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.absolute_angle;
	}
	
	//目的是启用角度闭环控制
    gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    gimbal_control.gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    gimbal_control.gimbal_yaw2_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;

	static int16_t yaw_channel = 0, pitch_channel = 0,b=0;
	static float Turn_back=0;
    rc_deadband_limit(gimbal_control.gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control.gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

	if(CANSendData.RC->key.v & GIMBAL_ROTATION_ANG&&b==0)
	{
	    Turn_back=0.025f;
		  b=1;
	}
//   yaw.pitch的角度增量(将遥控器通道值和鼠标输入值乘以灵敏度系数，并加上回中速度)
//    add_yaw_angle = yaw_channel * YAW_RC_SEN +Turn_back;
    add_pitch_angle = pitch_channel * PITCH_RC_SEN ;
    add_yaw2_angle = yaw_channel * YAW_RC_SEN +Turn_back;
	
	b=0;
	Turn_back=0;
		
	static fp32 angle_set_yaw,angle_set_pitch,angle_set_yaw2;
//	 更新yaw2.pitch轴的绝对角度设定值
//    angle_set_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle_set;
//    gimbal_control.gimbal_yaw_motor.absolute_angle_set = rad_format(angle_set_yaw + add_yaw_angle);
			
		angle_set_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle_set;
    gimbal_control.gimbal_pitch_motor.absolute_angle_set = rad_format(angle_set_pitch + add_pitch_angle );
	
		angle_set_yaw2 = gimbal_control.gimbal_yaw2_motor.absolute_angle_set;
    gimbal_control.gimbal_yaw2_motor.absolute_angle_set = rad_format(angle_set_yaw2 + add_yaw2_angle);
	//确保pitch轴在最大和最小相对角度范围内
    if (gimbal_control.gimbal_pitch_motor.absolute_angle_set > gimbal_control.gimbal_pitch_motor.max_relative_angle)
    {
        gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.max_relative_angle;
    }
    else if (gimbal_control.gimbal_pitch_motor.absolute_angle_set < gimbal_control.gimbal_pitch_motor.min_relative_angle)
    {
        gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.min_relative_angle;
    }
////确保yaw2在一定范围内
//	if(gimbal_control.gimbal_yaw2_motor.absolute_angle_set>1.0f)
//	{
//		gimbal_control.gimbal_yaw2_motor.absolute_angle_set=1.0f;
//	}
//	else if(gimbal_control.gimbal_yaw2_motor.absolute_angle_set<-1.0f)
//	{
//		gimbal_control.gimbal_yaw2_motor.absolute_angle_set=-1.0f;
//	}	
////对yaw轴进行双环PID控制		
//	gimbal_control.gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, gimbal_control.gimbal_yaw_motor.absolute_angle, gimbal_control.gimbal_yaw_motor.absolute_angle_set, gimbal_control.gimbal_yaw_motor.motor_gyro);//外环：位置环
//    gimbal_control.gimbal_yaw_motor.current_set = PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, gimbal_control.gimbal_yaw_motor.motor_gyro, gimbal_control.gimbal_yaw_motor.motor_gyro_set);//内环：速度环
//    gimbal_control.gimbal_yaw_motor.given_current = (int16_t)(gimbal_control.gimbal_yaw_motor.current_set);
//对pitch轴进行双环PID控制			
		gimbal_control.gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, gimbal_control.gimbal_pitch_motor.absolute_angle, gimbal_control.gimbal_pitch_motor.absolute_angle_set, gimbal_control.gimbal_pitch_motor.motor_gyro);
    gimbal_control.gimbal_pitch_motor.current_set = PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid, gimbal_control.gimbal_pitch_motor.motor_gyro, gimbal_control.gimbal_pitch_motor.motor_gyro_set);
    gimbal_control.gimbal_pitch_motor.given_current = (int16_t)(gimbal_control.gimbal_pitch_motor.current_set);
//对yaw2轴进行双环PID控制	
		gimbal_control.gimbal_yaw2_motor.motor_gyro_set = gimbal_PID_calc(&gimbal_control.gimbal_yaw2_motor.gimbal_motor_absolute_angle_pid, gimbal_control.gimbal_yaw2_motor.absolute_angle, gimbal_control.gimbal_yaw2_motor.absolute_angle_set, gimbal_control.gimbal_yaw2_motor.motor_gyro);//外环：位置环
    gimbal_control.gimbal_yaw2_motor.current_set = PID_calc(&gimbal_control.gimbal_yaw2_motor.gimbal_motor_gyro_pid, gimbal_control.gimbal_yaw2_motor.motor_gyro, gimbal_control.gimbal_yaw2_motor.motor_gyro_set);//内环：速度环
    gimbal_control.gimbal_yaw2_motor.given_current = (int16_t)(gimbal_control.gimbal_yaw2_motor.current_set);

////用相对角度消除yaw轴差距
//	diff=gimbal_control.gimbal_yaw_motor.relative_angle-gimbal_control.gimbal_yaw2_motor.relative_angle;
////	yaw_relative_angle=gimbal_control.gimbal_yaw_motor.relative_angle+diff;	
//	if(YAW2_MIN_ANGLE<gimbal_control.gimbal_yaw2_motor.absolute_angle&&gimbal_control.gimbal_yaw2_motor.absolute_angle<YAW2_MAX_ANGLE)
//	{		
//		gimbal_control.gimbal_yaw_motor.motor_gyro_set= gimbal_PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid,gimbal_control.gimbal_yaw_motor.relative_angle,diff ,0);//外环：相对角度
//		gimbal_control.gimbal_yaw_motor.current_set = PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, gimbal_control.gimbal_yaw_motor.motor_gyro, gimbal_control.gimbal_yaw_motor.motor_gyro_set);//内环：角速度环 
//		gimbal_control.gimbal_yaw_motor.given_current = (int16_t)(gimbal_control.gimbal_yaw_motor.current_set);
//			 
//	}
//对yaw轴进行双环PID控制(跟随小yaw)	
    big_yaw_follow_control();
	
    
}	

fp32 yuyu;
//导航
void Gimbal_Vision(void)
{
	static fp32 angle_set;
	fp32 add_pitch_angle = 0.0f;
	
	if(Vision_state )
	{
		return;
	}
		{  
			//vision_stable_count = 0;
			sing_data=36864;					
			gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
			gimbal_control.gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
		
		  //自动俯仰控制（抬头低头功能）
		 //低头 
			if(stage_remain_time.stage_remain_time<420&&stage_remain_time.stage_remain_time>=180)//前哨站未被摧毁
			{	
				if(PA_direction)
				{
						add_pitch_angle -= add_Pitch * PITCH_RC_SEN;
					yuyu-= add_Pitch * PITCH_RC_SEN;
						if(gimbal_control.gimbal_pitch_motor.relative_angle_set >= 0.0f)
						{
								PA_direction = 0;
						} 
				}
				//抬头
				else if(PA_direction == 0)
				{
						add_pitch_angle += add_Pitch * PITCH_RC_SEN;
						if(gimbal_control.gimbal_pitch_motor.relative_angle_set <= -0.55f)
						{
								PA_direction = 1;						
						}
				}
			}	
			if(stage_remain_time.stage_remain_time>0&&stage_remain_time.stage_remain_time<180)//前哨站被摧毁
			{	if(PA_direction)
				{
						add_pitch_angle -= add_Pitch * PITCH_RC_SEN;
					yuyu-= add_Pitch * PITCH_RC_SEN;
						if(gimbal_control.gimbal_pitch_motor.relative_angle_set >= 0.3f)
						{
								PA_direction = 0;
						} 
				}
				//抬头
				else if(PA_direction == 0)
				{
						add_pitch_angle += add_Pitch * PITCH_RC_SEN;
						if(gimbal_control.gimbal_pitch_motor.relative_angle_set <= -0.4f)
						{
								PA_direction = 1;						
						}
				}
			}
//			if(Refree_data.gimbal_state == 1)
//			{
				AutoSpeed = 400;
//			}
//			else
//			{
//				AutoSpeed = 0;
//			}									
			//add_yaw_angle = AutoSpeed * YAW_RC_SEN - gimbal_control.gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
			add_yaw_angle = AutoSpeed * 0 - gimbal_control.gimbal_rc_ctrl->mouse.x * 0;
			angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set;
			gimbal_control.gimbal_yaw_motor.absolute_angle_set = rad_format(angle_set + add_yaw_angle);
			
			gimbal_control.gimbal_pitch_motor.relative_angle_set += add_pitch_angle;
			
			gimbal_control.gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, gimbal_control.gimbal_yaw_motor.absolute_angle, gimbal_control.gimbal_yaw_motor.absolute_angle_set, gimbal_control.gimbal_yaw_motor.motor_gyro);
			gimbal_control.gimbal_yaw_motor.current_set = PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, gimbal_control.gimbal_yaw_motor.motor_gyro, gimbal_control.gimbal_yaw_motor.motor_gyro_set);
			gimbal_control.gimbal_yaw_motor.given_current = (int16_t)(gimbal_control.gimbal_yaw_motor.current_set);
			
//			gimbal_control.gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid_auto,gimbal_control.gimbal_yaw_motor.absolute_angle,gimbal_control.gimbal_yaw_motor.absolute_angle_set,gimbal_control.gimbal_yaw_motor.motor_gyro);
//			gimbal_control.gimbal_yaw_motor.current_set = PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid_auto,gimbal_control.gimbal_yaw_motor.motor_gyro,gimbal_control.gimbal_yaw_motor.motor_gyro_set);
//			gimbal_control.gimbal_yaw_motor.given_current = (int16_t)(gimbal_control.gimbal_yaw_motor.current_set);
			
			gimbal_control.gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid, gimbal_control.gimbal_pitch_motor.relative_angle, gimbal_control.gimbal_pitch_motor.relative_angle_set, gimbal_control.gimbal_pitch_motor.motor_gyro);
			gimbal_control.gimbal_pitch_motor.current_set = PID_calc(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid, gimbal_control.gimbal_pitch_motor.motor_gyro, gimbal_control.gimbal_pitch_motor.motor_gyro_set);
			gimbal_control.gimbal_pitch_motor.given_current = (int16_t)(gimbal_control.gimbal_pitch_motor.current_set);
				
			yaw2_ecd.out=PID_calc(&yaw2_ecd,gimbal_control.gimbal_yaw2_motor.gimbal_motor_measure->ecd,1084);
			yaw2_motor.out=PID_calc(&yaw2_motor,gimbal_control.gimbal_yaw2_motor.gimbal_motor_measure->speed_rpm,yaw2_ecd.out);
			gimbal_control.gimbal_yaw2_motor.current_set = yaw2_motor.out;
			gimbal_control.gimbal_yaw2_motor.given_current = (int16_t)gimbal_control.gimbal_yaw2_motor.current_set;
//	gimbal_control.gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, gimbal_control.gimbal_yaw_motor.absolute_angle, gimbal_control.gimbal_yaw2_motor.absolute_angle_set, gimbal_control.gimbal_yaw_motor.motor_gyro);//外环：位置环
//    gimbal_control.gimbal_yaw_motor.current_set = PID_calc(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, gimbal_control.gimbal_yaw_motor.motor_gyro, gimbal_control.gimbal_yaw_motor.motor_gyro_set);//内环：速度环
 //   gimbal_control.gimbal_yaw_motor.giv+en_current = (int16_t)(gimbal_control.gimbal_yaw_motor.current_set);


		}			
								
}
fp32 add_yaw1=200;
fp32 add_yaw2=0;
//自瞄
 void Auto_Aim(void)
{	
			sing_data = 0;
			gimbal_control.gimbal_yaw2_motor.gimbal_motor_mode = GIMBAL_MOTOR_VISION;
			gimbal_control.gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_VISION;	
			gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_VISION;
//			gimbal_control.gimbal_pitch_motor.given_current = (int16_t)(gimbal_control.gimbal_pitch_motor.vision_current_set);
			gimbal_control.gimbal_pitch_motor.given_current = pitch_can_send;
			gimbal_control.gimbal_yaw2_motor.given_current = yaw2_can_send+gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->speed_rpm* add_yaw2;
			gimbal_control.gimbal_yaw_motor.given_current = yaw_can_send+gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->speed_rpm* add_yaw1;

	
	 
	
}

//编码器值转换为弧度函数 
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

//云台控制模式切换时的过渡处理
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)//保持当前电流值，避免电流突变
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)//将目标角度设置为当前角度，避免角度跳变
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
	else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_VISION && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_VISION)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
		else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

	
	if (gimbal_mode_change->gimbal_yaw2_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw2_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)//保持当前电流值，避免电流突变
    {
        gimbal_mode_change->gimbal_yaw2_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw2_motor.current_set = gimbal_mode_change->gimbal_yaw2_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw2_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw2_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)//将目标角度设置为当前角度，避免角度跳变
    {
        gimbal_mode_change->gimbal_yaw2_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw2_motor.absolute_angle;
    }
	else if (gimbal_mode_change->gimbal_yaw2_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_VISION && gimbal_mode_change->gimbal_yaw2_motor.gimbal_motor_mode == GIMBAL_MOTOR_VISION)
    {
        gimbal_mode_change->gimbal_yaw2_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw2_motor.absolute_angle;
    }
		else if (gimbal_mode_change->gimbal_yaw2_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw2_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw2_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw2_motor.absolute_angle;
    }	
    gimbal_mode_change->gimbal_yaw2_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw2_motor.gimbal_motor_mode;
		
	
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
	else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_VISION && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_VISION)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
		else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
//        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}


#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif

//设置校准参数(它接收一些参数，并将这些参数设置到全局结构体gimbal_control中对应的成员变量上)


//执行校准命令(云台校准命令的回调函数)




bool_t UseVision(void)
{
	if(Vision_state )
	{
	return 1;
	}
		else
		{
		return 0;
		}
}
//计算校准参数(根据记录的最大和最小编码器值以及对应的角度，计算出编码器的中位点（零点）和相对角度范围)

void AUTO_Mode(void)
{
		if(CANSendData.RC->rc.ch[0] > 500&&CANSendData.RC->rc.ch[2] <-500)
		{
			AUTO=1;
		}

			else if((CANSendData.RC->rc.ch[0] <-500&&CANSendData.RC->rc.ch[2] >500)||(CANSendData.RC->rc.s[0] != 2))
		{
			AUTO=0;
		}
}

//云台PID初始化函数
void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

//云台PID计算函数
fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

//云台清除PID数据函数
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

//返回yaw轴电机数据的指针
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const gimbal_motor_t *get_yaw2_motor_point(void)
{
    return &gimbal_control.gimbal_yaw2_motor;
}
//返回pitch轴电机数据的指针
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}
//返回云台数据结构的指针
static void big_yaw_output_limit(void)
{
    float limit = big_follow_current_limit;
    float current = gimbal_control.gimbal_yaw_motor.current_set;
    if (!(limit > 0.0f && limit <= YAW_SPEED_PID_MAX_OUT)) limit = 0.0f;
    /* Vision has a separate given_current path; limit the actual command too. */
    if (!(current >= -YAW_SPEED_PID_MAX_OUT && current <= YAW_SPEED_PID_MAX_OUT))
    {
        current = 0.0f;
        gimbal_control.gimbal_yaw_motor.given_current = 0;
        PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid);
    }
    if (current > limit) current = limit;
    if (current < -limit) current = -limit;
    gimbal_control.gimbal_yaw_motor.current_set = current;
    if (gimbal_control.gimbal_yaw_motor.given_current > limit)
        gimbal_control.gimbal_yaw_motor.given_current = (int16_t)limit;
    if (gimbal_control.gimbal_yaw_motor.given_current < -limit)
        gimbal_control.gimbal_yaw_motor.given_current = (int16_t)-limit;
}

static void big_yaw_follow_control(void)
{
    static uint32_t test_start;
    float offset = gimbal_control.gimbal_yaw2_motor.relative_angle;
    float deadband = big_follow_deadband;
    float kp = big_follow_kp;
    float sign = big_follow_sign;
    float limit = big_follow_speed_limit;
    float speed;
    if (!(deadband >= 0.0f && deadband <= 0.5f) ||
        !(kp >= 0.0f && kp <= 20.0f) ||
        !(limit >= 0.0f && limit <= 5.0f) ||
        !(sign == 1.0f || sign == -1.0f) ||
        !(offset >= -PI && offset <= PI))
    {
        speed = 0.0f;
    }
    else
    {
        if (offset > deadband) offset -= deadband;
        else if (offset < -deadband) offset += deadband;
        else offset = 0.0f;
        speed = sign * kp * offset;
    }
    if (big_speed_test_enable)
    {
        if (!big_speed_test_was_enabled) test_start = HAL_GetTick();
        /* One bounded test pulse. Remain at zero until test mode is exited. */
        speed = (uint32_t)(HAL_GetTick() - test_start) < 2000U ? big_speed_test_set : 0.0f;
    }
    big_speed_test_was_enabled = big_speed_test_enable;
    if (!(limit >= 0.0f && limit <= 5.0f)) limit = 0.0f;
    if (!(speed >= -100.0f && speed <= 100.0f)) speed = 0.0f;
    if (speed > limit) speed = limit;
    if (speed < -limit) speed = -limit;
    gimbal_control.gimbal_yaw_motor.motor_gyro_set = speed;
    gimbal_control.gimbal_yaw_motor.current_set = PID_calc(
        &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid,
        gimbal_control.gimbal_yaw_motor.motor_gyro, speed);
    if (!(gimbal_control.gimbal_yaw_motor.current_set >= -YAW_SPEED_PID_MAX_OUT &&
          gimbal_control.gimbal_yaw_motor.current_set <= YAW_SPEED_PID_MAX_OUT))
    {
        gimbal_control.gimbal_yaw_motor.current_set = 0.0f;
        PID_clear(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid);
    }
    gimbal_control.gimbal_yaw_motor.given_current =
        (int16_t)gimbal_control.gimbal_yaw_motor.current_set;
}

/* Publish once after the control cycle has mapped final CAN command values.
 * These are software send requests, not a capture of the physical CAN bus. */
static void gimbal_publish_debug_snapshot(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    gimbal_debug_values[0] = (float)now * 0.001f;
    gimbal_debug_values[1] = (float)gimbal_behaviour;
    gimbal_debug_values[2] = gimbal_control.gimbal_pitch_motor.relative_angle_set;
    gimbal_debug_values[3] = gimbal_control.gimbal_pitch_motor.relative_angle;
    gimbal_debug_values[4] = gimbal_control.gimbal_pitch_motor.absolute_angle;
    gimbal_debug_values[5] = gimbal_control.gimbal_pitch_motor.motor_gyro_set;
    gimbal_debug_values[6] = gimbal_control.gimbal_pitch_motor.motor_gyro;
    gimbal_debug_values[7] = (float)pitch_can_set_current;
    gimbal_debug_values[8] = gimbal_control.gimbal_yaw2_motor.relative_angle_set;
    gimbal_debug_values[9] = gimbal_control.gimbal_yaw2_motor.relative_angle;
    gimbal_debug_values[10] = gimbal_control.gimbal_yaw2_motor.motor_gyro_set;
    gimbal_debug_values[11] = gimbal_control.gimbal_yaw2_motor.motor_gyro;
    gimbal_debug_values[12] = (float)yaw2_can_set_current;
    gimbal_debug_values[13] = gimbal_control.gimbal_yaw_motor.relative_angle_set;
    gimbal_debug_values[14] = gimbal_control.gimbal_yaw_motor.relative_angle;
    gimbal_debug_values[15] = gimbal_control.gimbal_yaw_motor.motor_gyro_set;
    gimbal_debug_values[16] = gimbal_control.gimbal_yaw_motor.motor_gyro;
    gimbal_debug_values[17] = (float)yaw_can_set_current;
    gimbal_debug_values[18] = (float)dm_gyro_online;
    gimbal_debug_values[19] = (float)dm_gyro_age_ms;
    __set_PRIMASK(irq_state);
}

void gimbal_get_debug_snapshot(float values[GIMBAL_DEBUG_CHANNELS])
{
    uint32_t i = 0U;
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    while (i < GIMBAL_DEBUG_CHANNELS)
    {
        values[i] = gimbal_debug_values[i];
        i += 1U;
    }
    __set_PRIMASK(irq_state);
}

gimbal_control_t *get_gimlal_data(void)
{
    return &gimbal_control;
}



