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
