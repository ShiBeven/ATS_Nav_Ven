#ifndef REFEREE_LIB_H
#define REFEREE_LIB_H
#include "struct_typedef.h"
/* 2027 exposes buffer energy and a power limit, not measured chassis power,
 * two independent 17mm shooter limits, or a legacy one-byte rune buff. */
uint8_t get_robot_id(void);
uint8_t get_robot_level(void);
uint16_t get_robot_max_HP(void);
uint16_t get_robot_remain_HP(void);
uint16_t get_chassis_buffer_energy(void);
uint16_t Chassis_Power_Limit(void);
uint16_t get_shooter_cooling_value(void);
uint16_t get_shooter_heat_limit(void);
uint16_t get_shooter_17mm_heat(void);
uint16_t get_shooter_42mm_heat(void);
fp32 get_bullet_speed_limit(void);
fp32 get_bullet_initial_speed(void);
uint8_t get_bullet_launching_frequency(void);
int MyColour(void);
#endif
