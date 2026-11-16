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
