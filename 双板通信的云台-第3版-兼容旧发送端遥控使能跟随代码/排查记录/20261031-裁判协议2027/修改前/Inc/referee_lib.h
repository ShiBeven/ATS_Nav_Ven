#ifndef REFEREE_LIB_H
#define REFEREE_LIB_H
#include "main.h"
#include "struct_typedef.h"



extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer, fp32 *powermax);
extern void  get_chassis_power_and_buffer_OLD(fp32 *power, fp32 *buffer);

extern uint8_t get_robot_id(void);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);

extern uint8_t get_robot_level(void);

extern uint16_t get_robot_max_HP(void);
extern uint16_t get_robot_remain_HP(void);

extern uint16_t ID1_cooling_rate(void);
extern uint16_t ID1_cooling_limit(void);
extern uint16_t ID1_speed_limit(void);
extern uint16_t ID1CoolingHeat(void);

extern uint16_t ID2_cooling_rate(void);
extern uint16_t ID2_cooling_limit(void);
extern uint16_t ID2_speed_limit(void);
extern uint16_t ID2CoolingHeat(void);

extern uint8_t robot_buff(void);

extern uint16_t Get_Fire2_Speed(void);
extern uint16_t Get_Fire1_Speed(void);

extern uint16_t Chassis_Power_Limit(void);

extern int MyColour(void);
#endif
