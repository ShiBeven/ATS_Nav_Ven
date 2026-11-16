#include "referee.h"
#include "referee_lib.h"
uint8_t get_robot_id(void) { return robot_state.robot_id; }
uint8_t get_robot_level(void) { return robot_state.robot_level; }
uint16_t get_robot_max_HP(void) { return robot_state.maximum_HP; }
uint16_t get_robot_remain_HP(void) { return robot_state.current_HP; }
uint16_t get_chassis_buffer_energy(void) { return power_heat_data.buffer_energy; }
uint16_t Chassis_Power_Limit(void) { return robot_state.chassis_power_limit; }
uint16_t get_shooter_cooling_value(void) { return robot_state.shooter_barrel_cooling_value; }
uint16_t get_shooter_heat_limit(void) { return robot_state.shooter_barrel_heat_limit; }
uint16_t get_shooter_17mm_heat(void) { return power_heat_data.shooter_17mm_barrel_heat; }
uint16_t get_shooter_42mm_heat(void) { return power_heat_data.shooter_42mm_barrel_heat; }
fp32 get_bullet_speed_limit(void) { return robot_state.bullet_speed_limit; }
fp32 get_bullet_initial_speed(void) { return shoot_data.initial_speed; }
uint8_t get_bullet_launching_frequency(void) { return shoot_data.launching_frequency; }
int MyColour(void) {
    uint8_t color = referee_team_color(robot_state.robot_id);
    return color <= 1U ? (int)color : -1;
}
