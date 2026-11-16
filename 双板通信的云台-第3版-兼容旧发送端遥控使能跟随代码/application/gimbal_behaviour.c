#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "Vision.h"

//int gimbal_mode = 0;
int reset_yaw = 1, reset_num = 1;
int Vision_Timing = 0;
fp32 Err_angle = 0;
int Vision_GetTiming = 0;
int YAW2_limit = 0;
int YAW2_lastlimit = 0;
int GIMBAL_YAWCtrl = 0;
gimbal_control_t gimbal_control_behaviour;

 #define gimbal_warn_buzzer_on() buzzer_on(31, 20000)
 #define gimbal_warn_buzzer_off() buzzer_off()

#define int_abs(x) ((x) > 0 ? (x) : (-x))

//#define rc_deadband_limit(input, output, dealine)        \
//    {                                                    \
//        if ((input) > (dealine) || (input) < -(dealine)) \
//        {                                                \
//            (output) = (input);                          \
//        }                                                \
//        else                                             \
//        {                                                \
//            (output) = 0;                                \
//        }                                                \
//    }
//#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
//    {                                                                                \
//        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
//        {                                                                            \
//            (cmd_time)++;                                                            \
//            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
//            {                                                                        \
//                (cmd_time) = 0;                                                      \
//                (angle_set) = (angle);                                               \
//                (ecd_set) = (ecd);                                                   \
//                (step)++;                                                            \
//            }                                                                        \
//        }                                                                            \
//    }





extern void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);


 gimbal_behaviour_e mode = GIMBAL_ZERO_FORCE;

static void gimbal_vision_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);



bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_control_behaviour.gimbal_mode_data == GIMBAL_INIT || gimbal_control_behaviour.gimbal_mode_data == GIMBAL_CALI || gimbal_control_behaviour.gimbal_mode_data == GIMBAL_MOTIONLESS || gimbal_control_behaviour.gimbal_mode_data == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}



bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_control_behaviour.gimbal_mode_data == GIMBAL_INIT || gimbal_control_behaviour.gimbal_mode_data == GIMBAL_CALI || gimbal_control_behaviour.gimbal_mode_data == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}




