#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
extern uint16_t AUTO;
//pitch speed close-loop PID params, max out and max iout
//pitch speed close-loop PID params, max out and max iout

//弧度转角度 rad 
//角度转弧度
#define TRANSITION_START_DEG 60.0f
#define TRANSITION_MIDDLE_DEG 80.0f

//角度转换为弧度
#define DEG_TO_RAD (PI/180.0f)
#define YAW2_MAX_ANGLE (60.0f*DEG_TO_RAD)
#define YAW2_MIN_ANGLE 0.0f /* retain previous effective definition */
//真的pitch速度环
#define PITCH_SPEED_PID_KP        8500.0f
#define PITCH_SPEED_PID_KI        0.0f
#define PITCH_SPEED_PID_KD        20.0f
#define PITCH_SPEED_PID_MAX_OUT   16000.0f
#define PITCH_SPEED_PID_MAX_IOUT  3000.0f
//真的Pitch位置环
#define PITCH_GYRO_ABSOLUTE_PID_KP 20.0f//25
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.5f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//真的yaw速度环(跟随)yaw speed close-loop PID params, max out and max iout
#define YAW_SPEED_PID_KP        13000.0f /* preserve the old effective gain */
#define YAW_SPEED_PID_KI        0.0f
#define YAW_SPEED_PID_KD        10.0f//20.0f//20.0f
#define YAW_SPEED_PID_MAX_OUT   25000.0f
#define YAW_SPEED_PID_MAX_IOUT  0.0f
//真的yaw速度环(锁死)
#define YAW_SPEED2_PID_KP        5500//5000.0f
#define YAW_SPEED2_PID_KI        0.0f
#define YAW_SPEED2_PID_KD        50.0f//20.0f
#define YAW_SPEED2_PID_MAX_OUT   16000.0f
#define YAW_SPEED2_PID_MAX_IOUT  0.0f
//真的yaw位置环
#define YAW_GYRO_ABSOLUTE_PID_KP      	25.0f// 7.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.001f//0.1f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT  10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f
//真的yaw编码器值yaw encode angle close-loop PID params, max out and max iout
#define YAW_ENCODE_RELATIVE_PID_KP        8.3f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.1f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT  15.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f
//yaw导航
#define YAW_GYRO_ABSOLUTE_PID_KP_AUTO     7.0f
#define YAW_GYRO_ABSOLUTE_PID_KI_AUTO     0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD_AUTO     0.01f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT	   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT    0.0f

#define YAW_SPEED_PID_KP_AUTO        8500.0f//5000.0f
#define YAW_SPEED_PID_KI_AUTO        0.0f
#define YAW_SPEED_PID_KD_AUTO        10.0f//20.0f//20.0f
//真的yaw2速度环控制
#define YAW2_SPEED_PID_KP 6000.0f//7000.0f
#define YAW2_SPEED_PID_KI 0.0f 
#define YAW2_SPEED_PID_KD 20.0f
#define YAW2_SPEED_PID_MAX_OUT 20000.0f
#define YAW2_SPEED_PID_MAX_IOUT 0.0f
//真的yaw2角度环控制
#define YAW2_GYRO_ABSOLUTE_PID_KP        6.2f//22.0f
#define YAW2_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW2_GYRO_ABSOLUTE_PID_KD        0.05f
#define YAW2_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW2_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f
//真的yaw2编码器值yaw encode angle close-loop PID params, max out and max iout
#define YAW2_ENCODE_RELATIVE_PID_KP        8.0f//8.3f
#define YAW2_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW2_ENCODE_RELATIVE_PID_KD        0.1f
#define YAW2_ENCODE_RELATIVE_PID_MAX_OUT   15.0f
#define YAW2_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f


#define YAW_GYRO_LOCK_PID_MAX_OUT   40000.0f


//pitch encode angle close-loop PID params, max out and max iout
#define PITCH_ENCODE_RELATIVE_PID_KP 15.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.3f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw encode angle close-loop PID params, max out and max iout
//#define YAW_ENCODE_RELATIVE_PID_KP        8.0f
//#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
//#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
//#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
//#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f



//烧饼模式
 #define  AUTO_MODE 1

#define GIMBAL_TASK_INIT_TIME 2

#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3


#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
#define GIMBAL_SPIN KEY_PRESSED_OFFSET_G
#define GIMBAL_TWO_MOVE KEY_PRESSED_OFFSET_B
#define TURN_SPEED    0.04f

#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R

#define RC_DEADBAND   20


#define YAW_RC_SEN    -0.000002f
#define PITCH_RC_SEN  -0.000002f //0.005
#define POS_RC_SEN  0.000001

#define YAW_MOUSE_SEN   0.00015f
#define PITCH_MOUSE_SEN 0.00015f
extern int16_t yaw_can_set_current , pitch_can_set_current,yaw2_can_set_current ;
//#define YAW_MOUSE_SEN   0.00005f
//#define PITCH_MOUSE_SEN 0.00015f

#define GIMBAL_CONTROL_TIME 1

//test mode, 0 close, 1 open
#define GIMBAL_TEST_MODE 0 

#define PITCH_TURN  1
#define YAW_TURN    0
#define YAW2_TURN    0


#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191

#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_YAW_ZERO_ANGLE_ERROR 0.02f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f

#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f
#define GIMBAL_YAW_OFFSET_ECD   7773
#define GIMBAL_YAW2_OFFSET_ECD  1118



//YAW2电机PID
#define YAW2_ANGLE_PID_KP 8.5f
#define YAW2_ANGLE_PID_KI 0.0f
#define YAW2_ANGLE_PID_KD 0.9f

//#define YAW2_SPEED_PID_KP 9.0f
//#define YAW2_SPEED_PID_KI 0.0f
//#define YAW2_SPEED_PID_KD 1.0f


#define YAW2_ANG_PID_MAX_OUT 20000.0f
#define YAW2_ANG_PID_MAX_IOUT 0.0f
/* One definition only; keep the formerly effective 20000 command limit. */
#define YAW2_SPEED_PID_MAX_IOUT 0.0f

//xingzeng
#define RPM_TO_RAD 0.104719755f  // 转速转换：RPM → rad/s (2π/60)
/* YAW2 angle limits are defined above around the mechanical centre. */


#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

/* Independent bounded calibration for Pitch and LIMITED small-Yaw only. */
#define GIMBAL_CALI_FEEDBACK_MAX_MS 50U
#define GIMBAL_CALI_STEP_TIMEOUT_MS 12000U
#define GIMBAL_CALI_SETTLE_MS 300U
#define GIMBAL_CALI_STOP_MS 2000U
#define GIMBAL_CALI_STILL_COUNTS 3
#define GIMBAL_CALI_PITCH_MODEL_MAX_ERROR 0.05f
#define GIMBAL_CALI_PITCH_COMMAND_MAX 8000
#define GIMBAL_CALI_YAW2_COMMAND_MAX 8000
extern volatile uint8_t gimbal_cali_status, gimbal_cali_error, gimbal_cali_valid, gimbal_cali_abort;
extern volatile uint8_t gimbal_cali_error_step;
extern volatile int16_t gimbal_cali_pitch_command, gimbal_cali_yaw2_command, gimbal_cali_motor_rpm;
extern volatile uint32_t gimbal_cali_step_elapsed_ms, gimbal_cali_still_ms, gimbal_cali_feedback_age_ms;
extern volatile int32_t gimbal_cali_yaw2_span_counts;



#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

#define YAW2_REGULAR  5102.0f
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

//dm_imu
#define DM_IMU_YAW_GYRO_SIGN      1.0f

extern volatile float big_follow_kp;
extern volatile float big_follow_sign;
extern volatile float big_follow_deadband;
extern volatile float big_follow_speed_limit;
extern volatile float big_follow_current_limit;
extern volatile uint8_t big_speed_test_enable;
extern volatile float big_speed_test_set;
extern volatile uint8_t dm_gyro_online;
extern volatile uint32_t dm_gyro_age_ms;
extern volatile uint8_t dm_fault_latched;

#define GIMBAL_DEBUG_CHANNELS 20U
void gimbal_get_debug_snapshot(float values[GIMBAL_DEBUG_CHANNELS]);


typedef struct 
{
	int test;
}SystemPIDCalc_t;
typedef struct
{
	const RC_ctrl_t *RC;
//		gimbal_control_t *gimbal_data;
}CAN_Send_t;


typedef enum
{
  GIMBAL_MOTOR_RAW = 0,
  GIMBAL_MOTOR_GYRO,
  GIMBAL_MOTOR_ENCONDE,
  GIMBAL_MOTOR_VISION,
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
	gimbal_PID_t gimbal_motor_absolute_angle_pid_auto;
		pid_type_def gimbal_motor_gyro_pid_auto;
		pid_type_def gimbal_motor_gyro_pid2;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
	  pid_type_def gimbal_motor_lock_pid;
		pid_type_def gimbal_motor_lock_pid2;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
	
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
		 fp32 vision_current_set;
    int16_t vision_given_current;
    fp32 vision_absolute_angle_set; //rad
	 float slave_angle;

} gimbal_motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
	gimbal_motor_t gimbal_yaw2_motor;
    gimbal_step_cali_t gimbal_cali;
	uint8_t gimbal_mode_data;
	fp32 GimbalFollowChassisAngle; 
	

} gimbal_control_t;

/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);
extern const gimbal_motor_t *get_yaw2_motor_point(void);
/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */

extern void gimbal_task(void const *pvParameters);

/**
  * @brief          gimbal cali calculate, return motor offset encode, max and min relative angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max relative angle
  * @param[out]     min_yaw:yaw min relative angle
  * @param[out]     max_yaw:pitch max relative angle
  * @param[out]     min_yaw:pitch min relative angle
  * @retval         none
  */
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/**
  * @brief          gimbal cali data, set motor offset encode, max and min relative angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max relative angle
  * @param[in]      min_yaw:yaw min relative angle
  * @param[in]      max_yaw:pitch max relative angle
  * @param[in]      min_yaw:pitch min relative angle
  * @retval         none
  */
	
extern void GetgimbalMode(void);
void Gimbal_Zero_Force(void);
void Gimbal_Init(void);
void Gimbal_Cali(void);
void Gimbal_Absolute_Angle(void);
void Gimbal_Double_Yaw(void);
void Gimbal_fllow_Chassis(void);
void Gimbal_Mottonless(void);
void Gimbal_Vision(void);
void Auto_Aim(void);
void Gimbal_yaw2_lock(void);

extern void gimbal_behaviour_set(void);


extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern void gimbal_motor_Vision_angle_control(gimbal_motor_t *gimbal_motor);
extern gimbal_control_t *get_gimlal_data(void);
void AUTO_Mode(void);
extern  fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
extern  void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
#endif
