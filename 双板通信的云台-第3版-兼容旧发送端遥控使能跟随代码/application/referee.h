#ifndef REFEREE_H
#define REFEREE_H
#include "main.h"
#include "protocol.h"

/* Serial wire layouts: RoboMaster 2027 V1.0.0, chapter 1 and appendix 2.
 * All reserved fields have unique C names; their bytes remain on the wire. */
typedef enum {
    RED_HEAVY = 1, RED_RESERVED_2 = 2,
    RED_STANDARD_1 = 3, RED_STANDARD_2 = 4, RED_STANDARD_3 = 5,
    RED_AERIAL = 6, RED_SENTRY = 7, RED_DART = 8, RED_RADAR = 9,
    RED_BASE = 10, R_OUTPOST = 11,
    BLUE_HEAVY = 101, BLUE_RESERVED_2 = 102,
    BLUE_STANDARD_1 = 103, BLUE_STANDARD_2 = 104, BLUE_STANDARD_3 = 105,
    BLUE_AERIAL = 106, BLUE_SENTRY = 107, BLUE_DART = 108, BLUE_RADAR = 109,
    BLUE_BASE = 110, B_OUTPOST = 111
} robot_id_t;
typedef enum {
    PROGRESS_UNSTART = 0, PROGRESS_PREPARE, PROGRESS_SELFCHECK,
    PROGRESS_5sCOUNTDOWN, PROGRESS_BATTLE, PROGRESS_CALCULATING
} game_progress_t;

typedef __packed struct { /* 0x0001, 11 bytes */
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;
typedef __packed struct { uint8_t winner; } game_result_t;
typedef __packed struct { /* 0x0003, 24 bytes */
    uint16_t ally_1_robot_HP, reserved, ally_3_robot_HP, ally_4_robot_HP;
    int16_t damage_difference;
    uint16_t ally_7_robot_HP, outpost_R_HP, ally_base_HP;
    uint16_t outpost_B_HP, enemy_base_HP, ally_control_degree, enemy_control_degree;
} game_robot_HP_t;
typedef __packed struct { uint32_t event_data, event_data_2; } event_data_t;
typedef __packed struct {
    uint8_t level, offending_robot_id, count;
} referee_warning_t;
typedef __packed struct {
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;
typedef __packed struct { /* 0x0201, 17 bytes, outputs at offset 16 */
    uint8_t robot_id, robot_level;
    uint16_t current_HP, maximum_HP;
    uint16_t shooter_barrel_cooling_value, shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    float bullet_speed_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
    uint8_t reserved_power_management : 5;
} robot_status_t;
typedef __packed struct { /* 0x0202: first eight bytes are reserved, NOT power */
    uint16_t reserved_1, reserved_2;
    float reserved;
    uint16_t buffer_energy, shooter_17mm_barrel_heat, shooter_42mm_barrel_heat;
} power_heat_data_t;
typedef __packed struct { float x, y, angle; } robot_pos_t; /* angle: degrees */
typedef __packed struct { /* 0x0204, cooling gain is uint16_t */
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff, vulnerability_buff;
    uint16_t attack_buff;
} buff_t;
typedef __packed struct { /* 0x0206, 7 bytes */
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
    uint16_t hit_sequence;
    int16_t hit_position_x, hit_position_y;
} hurt_data_t;
typedef __packed struct {
    uint8_t bullet_type, shooter_number, launching_frequency; /* frequency: Hz */
    float initial_speed; /* m/s */
} shoot_data_t;
typedef __packed struct {
    uint16_t projectile_allowance_17mm, projectile_allowance_42mm;
    uint16_t remaining_gold_coin, projectile_allowance_fortress;
} projectile_allowance_t;
typedef __packed struct { uint32_t rfid_status; uint16_t rfid_status_2; } rfid_status_t;
typedef __packed struct {
    uint8_t dart_launch_opening_status, reserved;
    uint16_t target_change_time, latest_launch_cmd_time;
} dart_client_cmd_t;
typedef __packed struct {
    float heavyengineer_x, heavyengineer_y, reserved_1, reserved_2;
    float standard_3_x, standard_3_y, standard_4_x, standard_4_y;
    float reserved_3, reserved_4;
} ground_robot_position_t;
typedef __packed struct { uint16_t mark_progress; } radar_mark_data_t;
typedef __packed struct {
    uint32_t sentry_info;
    uint16_t sentry_info_2;
    uint64_t sentry_info_3;
} sentry_info_t;
typedef __packed struct {
    uint8_t radar_info, radar_info_2;
    uint16_t fortify_center_x, fortify_center_y, fortify_radius;
} radar_info_t;
typedef __packed struct { /* 0x0301, actual payload = 6 + user_data length */
    uint16_t data_cmd_id, sender_id, receiver_id;
    uint8_t user_data[REFEREE_INTERACTION_MAX_DATA];
} robot_interaction_data_t;
typedef __packed struct { uint8_t delete_type, layer; } interaction_layer_delete_t;
typedef __packed struct {
    uint8_t figure_name[3];
    uint32_t operate_tpye : 3;
    uint32_t figure_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} interaction_figure_t;
typedef __packed struct { interaction_figure_t interaction_figure[2]; } interaction_figure_2_t;
typedef __packed struct { interaction_figure_t interaction_figure[5]; } interaction_figure_3_t;
typedef __packed struct { interaction_figure_t interaction_figure[7]; } interaction_figure_4_t;
typedef __packed struct {
    interaction_figure_t graphic_data;
    uint8_t data[30];
} interaction_character_t;
typedef __packed struct { uint32_t sentry_cmd; } sentry_cmd_t;
typedef __packed struct {
    uint8_t radar_cmd, password_cmd, password_1, password_2, password_3;
    uint8_t password_4, password_5, password_6, fortify_cmd;
    uint16_t fortify_center_x, fortify_center_y;
} radar_cmd_t;
typedef __packed struct {
    float target_position_x, target_position_y;
    uint8_t cmd_keyboard, target_robot_id;
    uint16_t cmd_source;
} map_command_t;
typedef __packed struct {
    uint16_t opponent_heavyengineer_position_x, opponent_heavyengineer_position_y;
    uint16_t reserved_1, reserved_2;
    uint16_t opponent_infantry_3_position_x, opponent_infantry_3_position_y;
    uint16_t opponent_infantry_4_position_x, opponent_infantry_4_position_y;
    uint16_t reserved_3, reserved_4;
    uint16_t opponent_sentry_position_x, opponent_sentry_position_y;
    uint16_t ally_heavy_position_x, ally_heavy_position_y, reserved_5, reserved_6;
    uint16_t ally_infantry_3_position_x, ally_infantry_3_position_y;
    uint16_t ally_infantry_4_position_x, ally_infantry_4_position_y;
    uint16_t reserved_7, reserved_8, ally_sentry_position_x, ally_sentry_position_y;
} map_robot_data_t;
typedef __packed struct {
    uint8_t intention;
    uint16_t start_position_x, start_position_y;
    int8_t delta_x[49], delta_y[49];
    uint16_t sender_id;
} map_data_t;
typedef __packed struct {
    uint16_t sender_id, receiver_id;
    uint8_t user_data[30]; /* 15 UTF-16 code units, little endian */
} custom_info_t;
/* Image-transmission link, not the 115200-baud power-management User port. */
typedef __packed struct { uint8_t data[300]; } robot_custom_data_2_t; /* 0x0310 TX */
typedef __packed struct { uint8_t data[60]; } robot_custom_data_t; /* 0x0311 RX */

#define REF_ASSERT_SIZE(type, bytes) typedef char ref_size_##type[(sizeof(type) == (bytes)) ? 1 : -1]
REF_ASSERT_SIZE(frame_header_struct_t, 5);
REF_ASSERT_SIZE(game_status_t, 11); REF_ASSERT_SIZE(game_result_t, 1);
REF_ASSERT_SIZE(game_robot_HP_t, 24); REF_ASSERT_SIZE(event_data_t, 8);
REF_ASSERT_SIZE(referee_warning_t, 3); REF_ASSERT_SIZE(dart_info_t, 3);
REF_ASSERT_SIZE(robot_status_t, 17); REF_ASSERT_SIZE(power_heat_data_t, 14);
REF_ASSERT_SIZE(robot_pos_t, 12); REF_ASSERT_SIZE(buff_t, 7);
REF_ASSERT_SIZE(hurt_data_t, 7); REF_ASSERT_SIZE(shoot_data_t, 7);
REF_ASSERT_SIZE(projectile_allowance_t, 8); REF_ASSERT_SIZE(rfid_status_t, 6);
REF_ASSERT_SIZE(dart_client_cmd_t, 6); REF_ASSERT_SIZE(ground_robot_position_t, 40);
REF_ASSERT_SIZE(radar_mark_data_t, 2); REF_ASSERT_SIZE(sentry_info_t, 14);
REF_ASSERT_SIZE(radar_info_t, 8); REF_ASSERT_SIZE(robot_interaction_data_t, 118);
REF_ASSERT_SIZE(interaction_layer_delete_t, 2); REF_ASSERT_SIZE(interaction_figure_t, 15);
REF_ASSERT_SIZE(interaction_figure_2_t, 30); REF_ASSERT_SIZE(interaction_figure_3_t, 75);
REF_ASSERT_SIZE(interaction_figure_4_t, 105); REF_ASSERT_SIZE(interaction_character_t, 45);
REF_ASSERT_SIZE(sentry_cmd_t, 4); REF_ASSERT_SIZE(radar_cmd_t, 13);
REF_ASSERT_SIZE(map_command_t, 12); REF_ASSERT_SIZE(map_robot_data_t, 48);
REF_ASSERT_SIZE(map_data_t, 105); REF_ASSERT_SIZE(custom_info_t, 34);
REF_ASSERT_SIZE(robot_custom_data_2_t, 300); REF_ASSERT_SIZE(robot_custom_data_t, 60);
#undef REF_ASSERT_SIZE

extern frame_header_struct_t referee_receive_header, referee_send_header;
extern game_status_t game_state;
extern game_result_t game_result;
extern game_robot_HP_t game_robot_HP;
extern event_data_t field_event;
extern referee_warning_t referee_warning;
extern dart_info_t dart_info;
extern robot_status_t robot_state;
extern power_heat_data_t power_heat_data;
extern robot_pos_t game_robot_pos;
extern buff_t buff;
extern hurt_data_t robot_hurt;
extern shoot_data_t shoot_data;
extern projectile_allowance_t projectile_allowance;
extern rfid_status_t robot_rfid;
extern dart_client_cmd_t dart_client_cmd;
extern ground_robot_position_t ground_robot_position;
extern radar_mark_data_t radar_mark_data;
extern sentry_info_t SentryInfo;
extern radar_info_t RadarInfo;
extern robot_interaction_data_t robot_interaction_data;
extern uint16_t robot_interaction_user_data_length;
extern interaction_layer_delete_t interaction_layer_delete;
extern interaction_figure_t interaction_figure;
extern interaction_figure_2_t interaction_figure_2;
extern interaction_figure_3_t interaction_figure_3;
extern interaction_figure_4_t interaction_figure_4;
extern interaction_character_t interaction_character;
extern sentry_cmd_t sentry_cmd;
extern radar_cmd_t radar_cmd;
extern map_command_t map_command;
extern map_robot_data_t map_robot_data;
extern map_data_t map_data;
extern custom_info_t custom_info;
extern robot_custom_data_2_t robot_custom_data_2;
extern robot_custom_data_t robot_custom_data;
extern volatile uint32_t referee_rx_valid_frames;
extern volatile uint32_t referee_last_rx_tick;
extern volatile uint16_t referee_last_cmd_id;
void init_referee_struct_data(void);
uint8_t referee_data_solve(const uint8_t *frame, uint16_t frame_length);
/* Returns 0=red, 1=blue, 0xFF=unknown/reserved. */
uint8_t referee_team_color(uint8_t robot_id);
#endif
