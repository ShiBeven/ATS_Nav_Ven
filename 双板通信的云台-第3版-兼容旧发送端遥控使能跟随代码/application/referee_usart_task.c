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
