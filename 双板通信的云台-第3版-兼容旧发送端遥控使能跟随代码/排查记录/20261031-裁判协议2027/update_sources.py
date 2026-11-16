"""Migrate this checkout's referee interface to the supplied 2027 V1.0.0 PDF."""
from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parents[2]
BACKUP = Path(__file__).resolve().parent / "修改前"
FILES = {}
FILES["application/protocol/protocol.h"] = r'''
#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H
#include "struct_typedef.h"

/* RoboMaster 2027 V1.0.0 (2026-10-29), section 1.1. */
#define REFEREE_PROTOCOL_VERSION "2027-V1.0.0-20261029"
#define HEADER_SOF 0xA5U
#define REF_PROTOCOL_MAX_DATA_SIZE 300U
#define REF_PROTOCOL_FRAME_MAX_SIZE 309U
#define REF_PROTOCOL_HEADER_SIZE sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE 2U
#define REF_PROTOCOL_CRC16_SIZE 2U
#define REF_HEADER_CRC_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CMD_SIZE)
#define REFEREE_SERVER_ID 0x8080U
#define REFEREE_INTERACTION_MAX_DATA 112U

typedef enum {
    GAME_STATE_CMD_ID = 0x0001, GAME_RESULT_CMD_ID = 0x0002,
    GAME_ROBOT_HP_CMD_ID = 0x0003, FIELD_EVENTS_CMD_ID = 0x0101,
    REFEREE_WARNING_CMD_ID = 0x0104, DART_INFO_CMD_ID = 0x0105,
    ROBOT_STATE_CMD_ID = 0x0201, POWER_HEAT_DATA_CMD_ID = 0x0202,
    ROBOT_POS_CMD_ID = 0x0203, BUFF_MUSK_CMD_ID = 0x0204,
    ROBOT_HURT_CMD_ID = 0x0206, SHOOT_DATA_CMD_ID = 0x0207,
    BULLET_REMAINING_CMD_ID = 0x0208, ROBOT_RFID_STATE = 0x0209,
    DART_CLIENT_CMD_ID = 0x020A, GROUND_ROBOT_POSITION_CMD_ID = 0x020B,
    RADAR_MARK_DATA_CMD_ID = 0x020C, SENTRY_INFO_CMD_ID = 0x020D,
    RADAR_INFO_CMD_ID = 0x020E, ROBOT_INTERACTION_DATA_CMD_ID = 0x0301,
    STUDENT_INTERACTIVE_DATA_CMD_ID = 0x0301,
    MAP_COMMAND_CMD_ID = 0x0303, MAP_ROBOT_DATA_CMD_ID = 0x0305,
    MAP_PATH_CMD_ID = 0x0307, CUSTOM_INFO_CMD_ID = 0x0308,
    ROBOT_CUSTOM_DATA_TX_CMD_ID = 0x0310,
    ROBOT_CUSTOM_DATA_RX_CMD_ID = 0x0311
} referee_cmd_id_t;

#pragma pack(push, 1)
typedef __packed struct {
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
} frame_header_struct_t;
#pragma pack(pop)

typedef enum {
    STEP_HEADER_SOF = 0, STEP_LENGTH_LOW, STEP_LENGTH_HIGH,
    STEP_FRAME_SEQ, STEP_HEADER_CRC8, STEP_DATA_CRC16
} unpack_step_e;
typedef struct {
    frame_header_struct_t *p_header;
    uint16_t data_len;
    uint8_t protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e unpack_step;
    uint16_t index;
} unpack_data_t;
#endif
'''

FILES["application/referee.h"] = r'''
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
'''

FILES["application/referee.c"] = r'''
#include "referee.h"
#include "CRC8_CRC16.h"
#include <string.h>

frame_header_struct_t referee_receive_header, referee_send_header;
game_status_t game_state;
game_result_t game_result;
game_robot_HP_t game_robot_HP;
event_data_t field_event;
referee_warning_t referee_warning;
dart_info_t dart_info;
robot_status_t robot_state;
power_heat_data_t power_heat_data;
robot_pos_t game_robot_pos;
buff_t buff;
hurt_data_t robot_hurt;
shoot_data_t shoot_data;
projectile_allowance_t projectile_allowance;
rfid_status_t robot_rfid;
dart_client_cmd_t dart_client_cmd;
ground_robot_position_t ground_robot_position;
radar_mark_data_t radar_mark_data;
sentry_info_t SentryInfo;
radar_info_t RadarInfo;
robot_interaction_data_t robot_interaction_data;
uint16_t robot_interaction_user_data_length;
interaction_layer_delete_t interaction_layer_delete;
interaction_figure_t interaction_figure;
interaction_figure_2_t interaction_figure_2;
interaction_figure_3_t interaction_figure_3;
interaction_figure_4_t interaction_figure_4;
interaction_character_t interaction_character;
sentry_cmd_t sentry_cmd;
radar_cmd_t radar_cmd;
map_command_t map_command;
map_robot_data_t map_robot_data;
map_data_t map_data;
custom_info_t custom_info;
robot_custom_data_2_t robot_custom_data_2;
robot_custom_data_t robot_custom_data;
volatile uint32_t referee_rx_valid_frames;
volatile uint32_t referee_last_rx_tick;
volatile uint16_t referee_last_cmd_id;

void init_referee_struct_data(void)
{
#define CLEAR(object) memset(&(object), 0, sizeof(object))
    CLEAR(referee_receive_header); CLEAR(referee_send_header);
    CLEAR(game_state); CLEAR(game_result); CLEAR(game_robot_HP);
    CLEAR(field_event); CLEAR(referee_warning); CLEAR(dart_info);
    CLEAR(robot_state); CLEAR(power_heat_data); CLEAR(game_robot_pos);
    CLEAR(buff); CLEAR(robot_hurt); CLEAR(shoot_data); CLEAR(projectile_allowance);
    CLEAR(robot_rfid); CLEAR(dart_client_cmd); CLEAR(ground_robot_position);
    CLEAR(radar_mark_data); CLEAR(SentryInfo); CLEAR(RadarInfo);
    CLEAR(robot_interaction_data); CLEAR(interaction_layer_delete);
    CLEAR(interaction_figure); CLEAR(interaction_figure_2);
    CLEAR(interaction_figure_3); CLEAR(interaction_figure_4);
    CLEAR(interaction_character); CLEAR(sentry_cmd); CLEAR(radar_cmd);
    CLEAR(map_command); CLEAR(map_robot_data); CLEAR(map_data); CLEAR(custom_info);
    CLEAR(robot_custom_data_2); CLEAR(robot_custom_data);
#undef CLEAR
    robot_interaction_user_data_length = 0U;
    referee_rx_valid_frames = 0U;
    referee_last_rx_tick = 0U;
    referee_last_cmd_id = 0U;
}

uint8_t referee_team_color(uint8_t id)
{
    if (id >= 1U && id <= 11U && id != 2U) return 0U;
    if (id >= 101U && id <= 111U && id != 102U) return 1U;
    return 0xFFU;
}

static uint16_t referee_u16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint8_t referee_interaction_solve(const uint8_t *data, uint16_t length)
{
    uint16_t sub_id, user_length, expected = 0U;
    void *destination = NULL;
    if (length < 6U || length > sizeof(robot_interaction_data)) return 0U;
    sub_id = referee_u16(data);
    user_length = length - 6U;
    if (sub_id >= 0x0200U && sub_id <= 0x02FFU) {
        expected = user_length;
    } else {
        switch (sub_id) {
#define INTERACTION_CASE(id, object) case id: destination = &(object); expected = sizeof(object); break
        INTERACTION_CASE(0x0100U, interaction_layer_delete);
        INTERACTION_CASE(0x0101U, interaction_figure);
        INTERACTION_CASE(0x0102U, interaction_figure_2);
        INTERACTION_CASE(0x0103U, interaction_figure_3);
        INTERACTION_CASE(0x0104U, interaction_figure_4);
        INTERACTION_CASE(0x0110U, interaction_character);
        INTERACTION_CASE(0x0120U, sentry_cmd);
        INTERACTION_CASE(0x0121U, radar_cmd);
#undef INTERACTION_CASE
        default: return 0U;
        }
    }
    if (user_length != expected) return 0U;
    /* This packet is variable length: never copy sizeof(maximum) blindly. */
    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data));
    memcpy(&robot_interaction_data, data, length);
    robot_interaction_user_data_length = user_length;
    if (destination != NULL) memcpy(destination, data + 6U, user_length);
    return 1U;
}

uint8_t referee_data_solve(const uint8_t *frame, uint16_t frame_length)
{
    uint16_t length, command, expected = 0U;
    void *destination = NULL;
    if (frame == NULL || frame_length < REF_HEADER_CRC_CMDID_LEN ||
        frame_length > REF_PROTOCOL_FRAME_MAX_SIZE || frame[0] != HEADER_SOF) return 0U;
    length = referee_u16(frame + 1U);
    if ((uint32_t)length + REF_HEADER_CRC_CMDID_LEN != frame_length) return 0U;
    if (!verify_CRC8_check_sum((uint8_t *)frame, REF_PROTOCOL_HEADER_SIZE) ||
        !verify_CRC16_check_sum((uint8_t *)frame, frame_length)) return 0U;
    command = referee_u16(frame + REF_PROTOCOL_HEADER_SIZE);
    if (command == ROBOT_INTERACTION_DATA_CMD_ID) {
        if (!referee_interaction_solve(frame + REF_HEADER_CMDID_LEN, length)) return 0U;
    } else {
        switch (command) {
#define DATA_CASE(id, object) case id: destination = &(object); expected = sizeof(object); break
        DATA_CASE(GAME_STATE_CMD_ID, game_state);
        DATA_CASE(GAME_RESULT_CMD_ID, game_result);
        DATA_CASE(GAME_ROBOT_HP_CMD_ID, game_robot_HP);
        DATA_CASE(FIELD_EVENTS_CMD_ID, field_event);
        DATA_CASE(REFEREE_WARNING_CMD_ID, referee_warning);
        DATA_CASE(DART_INFO_CMD_ID, dart_info);
        DATA_CASE(ROBOT_STATE_CMD_ID, robot_state);
        DATA_CASE(POWER_HEAT_DATA_CMD_ID, power_heat_data);
        DATA_CASE(ROBOT_POS_CMD_ID, game_robot_pos);
        DATA_CASE(BUFF_MUSK_CMD_ID, buff);
        DATA_CASE(ROBOT_HURT_CMD_ID, robot_hurt);
        DATA_CASE(SHOOT_DATA_CMD_ID, shoot_data);
        DATA_CASE(BULLET_REMAINING_CMD_ID, projectile_allowance);
        DATA_CASE(ROBOT_RFID_STATE, robot_rfid);
        DATA_CASE(DART_CLIENT_CMD_ID, dart_client_cmd);
        DATA_CASE(GROUND_ROBOT_POSITION_CMD_ID, ground_robot_position);
        DATA_CASE(RADAR_MARK_DATA_CMD_ID, radar_mark_data);
        DATA_CASE(SENTRY_INFO_CMD_ID, SentryInfo);
        DATA_CASE(RADAR_INFO_CMD_ID, RadarInfo);
        DATA_CASE(MAP_COMMAND_CMD_ID, map_command);
        DATA_CASE(MAP_ROBOT_DATA_CMD_ID, map_robot_data);
        DATA_CASE(MAP_PATH_CMD_ID, map_data);
        DATA_CASE(CUSTOM_INFO_CMD_ID, custom_info);
        DATA_CASE(ROBOT_CUSTOM_DATA_TX_CMD_ID, robot_custom_data_2);
        DATA_CASE(ROBOT_CUSTOM_DATA_RX_CMD_ID, robot_custom_data);
#undef DATA_CASE
        default: return 0U;
        }
        if (length != expected) return 0U;
        memcpy(destination, frame + REF_HEADER_CMDID_LEN, length);
    }
    memcpy(&referee_receive_header, frame, REF_PROTOCOL_HEADER_SIZE);
    referee_last_cmd_id = command;
    referee_last_rx_tick = HAL_GetTick();
    referee_rx_valid_frames++;
    return 1U;
}
'''

FILES["application/referee_usart_task.h"] = r'''
#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H
#include "main.h"
#include "CAN_receive.h"
#include "referee.h"

#define USART_RX_BUF_LENGHT 512U
#define REFEREE_FIFO_BUF_LENGTH 2048U
#define SENTRY_CMD_POSTURE_SHIFT 21U
#define SENTRY_CMD_POSTURE_MASK (7UL << SENTRY_CMD_POSTURE_SHIFT)
#define SENTRY_CMD_ACTIVATE_RUNE (1UL << 24)

typedef frame_header_struct_t frame_header_t;
typedef __packed struct {
    uint16_t data_cmd_id, sender_id, receiver_id;
    sentry_cmd_t SENTRY;
} data_header_t;
typedef __packed struct {
    frame_header_t framehead;
    uint16_t CMD_ID;
    data_header_t datahead;
} sentry_fh;

typedef struct {
    uint16_t exchanged_bullets;
    uint8_t remote_bullet_cnt, remote_heal_cnt;
    uint8_t can_free_respawn, can_pay_respawn;
    uint16_t respawn_gold_cost;
    uint8_t is_out_of_combat;
    uint16_t team_17mm_remaining;
    uint8_t current_posture, can_activate_rune, is_enhanced_posture;
    uint8_t attack_remaining_time, defend_remaining_time, move_remaining_time;
    uint8_t enhanced_attack_remaining_time, enhanced_defend_remaining_time;
    uint8_t enhanced_move_remaining_time;
} sentry_info_extracted_t;

/* 0x0101, 2027 table 1-9. R/B refer to the field's original red/blue side,
 * not this robot's ally/enemy color. */
typedef struct {
    uint8_t supply_zone_activated, small_rune_status, big_rune_status;
    uint8_t r_fortress_gain, b_fortress_gain, r_base_gain, b_base_gain;
    uint8_t allied_high_ground_gain, r_central_highland_gain, b_central_highland_gain;
    uint8_t r_outpost_gain, b_outpost_gain;
    uint8_t r_outpost_control, b_outpost_control, r_base_control, b_base_control;
    uint16_t enemy_dart_last_hit_time;
    uint8_t enemy_dart_last_hit_target;
    uint8_t r_highland_activation, r_highland_control;
    uint8_t b_highland_activation, b_highland_control;
    uint8_t central_activation, central_control;
} event_data_parsed_t;

extern uint8_t Vision_ID;
extern sentry_info_extracted_t info;
extern event_data_parsed_t event_info;
/* Defaults to zero. Enabling sets the 2027 rune request at bit 24. */
extern volatile uint8_t sentry_activate_rune;
void robot_id(void);
void referee_usart_task(void const *argument);
void send_test(sentry_fh *fh);
void get_sentry_info(sentry_info_extracted_t *result);
void get_event_info(event_data_parsed_t *result);
#endif
'''

FILES["application/referee_usart_task.c"] = r'''
#include "referee_usart_task.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "usb_task.h"
#include <string.h>

extern UART_HandleTypeDef huart6;
sentry_fh draw_data;
sentry_info_extracted_t info;
event_data_parsed_t event_info;
uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
uint8_t Vision_ID = 0xFFU;
uint8_t UI_Seq1;
uint8_t hhg_test[19];
volatile uint8_t sentry_activate_rune;
static void referee_unpack_fifo_data(void);

void referee_usart_task(void const *argument)
{
    uint32_t last_send = 0U;
    (void)argument;
    init_referee_struct_data();
    memset(&referee_unpack_obj, 0, sizeof(referee_unpack_obj));
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    /* Start this port only after its FIFO and dedicated RX buffers exist. */
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
    while (1) {
        uint32_t now;
        referee_unpack_fifo_data();
        robot_id();
        get_sentry_info(&info);
        get_event_info(&event_info);
        now = HAL_GetTick();
        if ((uint32_t)(now - last_send) >= 100U) {
            last_send = now;
            send_test(&draw_data); /* 10 Hz < 0x0301's 30 Hz limit */
        }
        osDelay(10);
    }
}

void robot_id(void)
{
    uint8_t team = referee_team_color(robot_state.robot_id);
    /* Preserve the existing target-color convention: red ally -> blue target. */
    Vision_ID = (team <= 1U) ? (uint8_t)(1U - team) : 0xFFU;
}

void send_test(sentry_fh *fh)
{
    uint8_t posture;
    uint32_t command = 0U;
    sentry_info_extracted_t status;
    if (fh == NULL || (robot_state.robot_id != RED_SENTRY &&
                       robot_state.robot_id != BLUE_SENTRY)) return;
    memset(fh, 0, sizeof(*fh));
    fh->framehead.SOF = HEADER_SOF;
    fh->framehead.data_length = sizeof(data_header_t);
    fh->framehead.seq = UI_Seq1++;
    fh->CMD_ID = ROBOT_INTERACTION_DATA_CMD_ID;
    fh->datahead.data_cmd_id = 0x0120U;
    fh->datahead.sender_id = robot_state.robot_id;
    fh->datahead.receiver_id = REFEREE_SERVER_ID;
    get_sentry_info(&status);
    if (status.can_free_respawn) command |= 1UL;
    posture = (uint8_t)RECEIVE_ROBOT_CMD_DATA.data.speed_vector.mode;
    if (posture < 1U || posture > 6U) posture = 3U;
    /* 2027: posture widened from 2 to 3 bits, rune moved to bit 24. */
    command |= ((uint32_t)posture << SENTRY_CMD_POSTURE_SHIFT);
    if (sentry_activate_rune) command |= SENTRY_CMD_ACTIVATE_RUNE;
    fh->datahead.SENTRY.sentry_cmd = command;
    /* Serialize whole packed objects, never take packed scalar addresses. */
    memcpy(hhg_test, fh, sizeof(*fh));
    append_CRC8_check_sum(hhg_test, REF_PROTOCOL_HEADER_SIZE);
    fh->framehead.CRC8 = hhg_test[4];
    append_CRC16_check_sum(hhg_test, sizeof(hhg_test));
    (void)HAL_UART_Transmit(&huart6, hhg_test, sizeof(hhg_test), 20U);
}

void get_sentry_info(sentry_info_extracted_t *result)
{
    uint32_t first;
    uint16_t second;
    uint64_t third;
    if (result == NULL) return;
    first = SentryInfo.sentry_info;
    second = SentryInfo.sentry_info_2;
    third = SentryInfo.sentry_info_3;
    result->exchanged_bullets = (uint16_t)(first & 0x7FFU);
    result->remote_bullet_cnt = (uint8_t)((first >> 11) & 0x0FU);
    result->remote_heal_cnt = (uint8_t)((first >> 15) & 0x0FU);
    result->can_free_respawn = (uint8_t)((first >> 19) & 1U);
    result->can_pay_respawn = (uint8_t)((first >> 20) & 1U);
    result->respawn_gold_cost = (uint16_t)((first >> 21) & 0x3FFU);
    result->is_out_of_combat = (uint8_t)(second & 1U);
    result->team_17mm_remaining = (uint16_t)((second >> 1) & 0x7FFU);
    result->current_posture = (uint8_t)((second >> 12) & 3U);
    result->can_activate_rune = (uint8_t)((second >> 14) & 1U);
    result->is_enhanced_posture = (uint8_t)((second >> 15) & 1U);
    result->attack_remaining_time = (uint8_t)(third & 0xFFU);
    result->defend_remaining_time = (uint8_t)((third >> 8) & 0xFFU);
    result->move_remaining_time = (uint8_t)((third >> 16) & 0xFFU);
    result->enhanced_attack_remaining_time = (uint8_t)((third >> 32) & 0xFFU);
    result->enhanced_defend_remaining_time = (uint8_t)((third >> 40) & 0xFFU);
    result->enhanced_move_remaining_time = (uint8_t)((third >> 48) & 0xFFU);
}

void get_event_info(event_data_parsed_t *result)
{
    uint32_t first, second;
    if (result == NULL) return;
    first = field_event.event_data;
    second = field_event.event_data_2;
    result->supply_zone_activated = (uint8_t)(first & 1U);
    result->small_rune_status = (uint8_t)((first >> 1) & 3U);
    result->big_rune_status = (uint8_t)((first >> 3) & 3U);
    result->r_fortress_gain = (uint8_t)((first >> 5) & 7U);
    result->b_fortress_gain = (uint8_t)((first >> 8) & 7U);
    result->r_base_gain = (uint8_t)((first >> 11) & 7U);
    result->b_base_gain = (uint8_t)((first >> 14) & 7U);
    result->allied_high_ground_gain = (uint8_t)((first >> 17) & 1U);
    result->r_central_highland_gain = (uint8_t)((first >> 18) & 3U);
    result->b_central_highland_gain = (uint8_t)((first >> 20) & 3U);
    result->r_outpost_gain = (uint8_t)((first >> 22) & 7U);
    result->b_outpost_gain = (uint8_t)((first >> 25) & 7U);
    result->r_outpost_control = (uint8_t)((first >> 28) & 3U);
    result->b_outpost_control = (uint8_t)((first >> 30) & 3U);
    result->r_base_control = (uint8_t)(second & 3U);
    result->b_base_control = (uint8_t)((second >> 2) & 3U);
    result->enemy_dart_last_hit_time = (uint16_t)((second >> 4) & 0x1FFU);
    result->enemy_dart_last_hit_target = (uint8_t)((second >> 13) & 7U);
    result->r_highland_activation = (uint8_t)((second >> 16) & 3U);
    result->r_highland_control = (uint8_t)((second >> 18) & 3U);
    result->b_highland_activation = (uint8_t)((second >> 20) & 3U);
    result->b_highland_control = (uint8_t)((second >> 22) & 3U);
    result->central_activation = (uint8_t)((second >> 24) & 3U);
    result->central_control = (uint8_t)((second >> 26) & 3U);
}

static void referee_unpack_fifo_data(void)
{
    uint8_t byte;
    unpack_data_t *packet = &referee_unpack_obj;
    while (fifo_s_used(&referee_fifo)) {
        byte = (uint8_t)fifo_s_get(&referee_fifo);
        switch (packet->unpack_step) {
        case STEP_HEADER_SOF:
            packet->index = 0U;
            if (byte == HEADER_SOF) {
                packet->protocol_packet[packet->index++] = byte;
                packet->unpack_step = STEP_LENGTH_LOW;
            }
            break;
        case STEP_LENGTH_LOW:
            packet->data_len = byte;
            packet->protocol_packet[packet->index++] = byte;
            packet->unpack_step = STEP_LENGTH_HIGH;
            break;
        case STEP_LENGTH_HIGH:
            packet->data_len |= (uint16_t)((uint16_t)byte << 8);
            packet->protocol_packet[packet->index++] = byte;
            if (packet->data_len <= REF_PROTOCOL_MAX_DATA_SIZE) {
                packet->unpack_step = STEP_FRAME_SEQ;
            } else {
                packet->unpack_step = STEP_HEADER_SOF;
                packet->index = 0U;
            }
            break;
        case STEP_FRAME_SEQ:
            packet->protocol_packet[packet->index++] = byte;
            packet->unpack_step = STEP_HEADER_CRC8;
            break;
        case STEP_HEADER_CRC8:
            packet->protocol_packet[packet->index++] = byte;
            if (verify_CRC8_check_sum(packet->protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
                packet->unpack_step = STEP_DATA_CRC16;
            } else {
                packet->unpack_step = STEP_HEADER_SOF;
                packet->index = 0U;
            }
            break;
        case STEP_DATA_CRC16:
            packet->protocol_packet[packet->index++] = byte;
            if (packet->index == packet->data_len + REF_HEADER_CRC_CMDID_LEN) {
                uint16_t total = packet->index;
                packet->unpack_step = STEP_HEADER_SOF;
                packet->index = 0U;
                (void)referee_data_solve(packet->protocol_packet, total);
            }
            break;
        default:
            packet->unpack_step = STEP_HEADER_SOF;
            packet->index = 0U;
            break;
        }
    }
}

void USART6_IRQHandler(void)
{
    if (USART6->SR & UART_FLAG_IDLE) {
        uint16_t received;
        uint8_t completed;
        __HAL_UART_CLEAR_PEFLAG(&huart6);
        __HAL_DMA_DISABLE(huart6.hdmarx);
        received = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        completed = ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) ? 0U : 1U;
        __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
        if (completed == 0U) huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
        else huart6.hdmarx->Instance->CR &= ~DMA_SxCR_CT;
        __HAL_DMA_ENABLE(huart6.hdmarx);
        fifo_s_puts(&referee_fifo, (char *)usart6_buf[completed], received);
    }
}
'''

FILES["Inc/referee_lib.h"] = r'''
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
'''
FILES["Src/referee_lib.c"] = r'''
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
'''

# Minimal edits to interfaces which feed or transport official referee fields.
usb = (ROOT / "application/usb_task.h").read_bytes()
old_enum = b'enum mode { Move = 0, Attack = 1, Defend = 2 } mode;'
new_enum = b'enum mode { Attack = 1, Defend = 2, Move = 3, EnhancedAttack = 4, EnhancedDefend = 5, EnhancedMove = 6 } mode;'
assert usb.count(old_enum) == 1
FILES["application/usb_task.h"] = usb.replace(old_enum, new_enum)
usart = (ROOT / "Src/usart.c").read_bytes()
assert usart.count(b'MY_UART6_DMA_Init();') == 1
FILES["Src/usart.c"] = usart.replace(b'MY_UART6_DMA_Init();', b'/* Referee task starts USART6 DMA after initializing its FIFO. */')
can = (ROOT / "application/CAN_receive.h").read_bytes()
assert can.count(b'uint8_t current_Hp;') == 1
FILES["application/CAN_receive.h"] = can.replace(b'uint8_t current_Hp;', b'uint16_t current_Hp;')

for name in FILES:
    current = (ROOT / name).read_bytes()
    assert current == (BACKUP / name).read_bytes(), f"Source changed since backup: {name}"
for name, data in FILES.items():
    if isinstance(data, str):
        data = (data.lstrip("\n").rstrip() + "\n").replace("\n", "\r\n").encode("utf-8")
    (ROOT / name).write_bytes(data)
result = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in FILES}
(BACKUP.parent / "修改文件哈希.json").write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
print("Updated", len(FILES), "files")
