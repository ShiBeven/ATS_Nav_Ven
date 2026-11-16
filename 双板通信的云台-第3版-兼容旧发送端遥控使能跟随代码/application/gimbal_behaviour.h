#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "struct_typedef.h"

#include "gimbal_task.h"
//yaw,pitch控制通道以及状态开关通道
#define YawChannel 2
#define PitchChannel 3
#define ModeChannel 0

extern int sing_data;
typedef enum
{
  GIMBAL_ZERO_FORCE = 0, 
  GIMBAL_INIT,           
  GIMBAL_CALI,           
  GIMBAL_ABSOLUTE_ANGLE, 
  GIMBAL_YAW2_LOCK,	
  GIMBAL_FLLOW_CHASSIS, 
  GIMBAL_MOTIONLESS,
  GIMBAL_VISION,
  AUTO_AIM,
  GIMBAL_DOUBLE_YAW,
} gimbal_behaviour_e;
#define GIMBAL_ROTATION_ANG KEY_PRESSED_OFFSET_C

extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);

extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);

extern bool_t gimbal_cmd_to_chassis_stop(void);
static void gimbal_auto_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
extern bool_t gimbal_cmd_to_shoot_stop(void);
extern int reset_yaw;
extern  gimbal_behaviour_e gimbal_behaviour;
#endif
