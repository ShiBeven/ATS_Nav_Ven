#include "remote_control.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"
#include "Vision.h"
#include "detect_task.h"



#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
//extern  gimbal_behaviour_e gimbal_behaviour;
gimbal_control_t gimbal_control_rc;

static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
RC_ctrl_t rc_ctrl;
/* Last valid frame from either local DBUS or CAN2 forwarded RC. */
volatile uint32_t rc_last_can_tick = 0;

static int16_t rc_get_i16_be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

void remote_control_update_basic(const uint8_t *data)
{
    rc_ctrl.rc.ch[0] = rc_get_i16_be(&data[0]);
    rc_ctrl.rc.ch[1] = rc_get_i16_be(&data[2]);
    rc_ctrl.rc.ch[2] = rc_get_i16_be(&data[4]);
    rc_ctrl.rc.s[1] = data[6];
    rc_ctrl.rc.s[0] = data[7];
    rc_last_can_tick = HAL_GetTick();
}

void remote_control_update_aux(const uint8_t *data)
{
    rc_ctrl.rc.ch[4] = rc_get_i16_be(&data[0]);
    rc_ctrl.key.v = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
    rc_last_can_tick = HAL_GetTick();
}

void remote_control_update_mouse(const uint8_t *data)
{
    rc_ctrl.mouse.x = rc_get_i16_be(&data[0]);
    rc_ctrl.mouse.y = rc_get_i16_be(&data[2]);
    rc_ctrl.mouse.z = rc_get_i16_be(&data[4]);
    rc_ctrl.mouse.press_l = data[6];
    rc_ctrl.mouse.press_r = data[7];
    rc_last_can_tick = HAL_GetTick();
}

void remote_control_can_timeout_check(void)
{
    if (rc_last_can_tick == 0 || (HAL_GetTick() - rc_last_can_tick) > 100u)
    {
        memset(&rc_ctrl, 0, sizeof(rc_ctrl));
        rc_ctrl.rc.s[0] = RC_SW_DOWN;
        rc_ctrl.rc.s[1] = RC_SW_DOWN;
    }
}
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    memset(&rc_ctrl, 0, sizeof(rc_ctrl));
    rc_last_can_tick = 0;
    /* The RC receiver is connected to USART3. Start the double-buffer DMA
       before the scheduler starts so the USART3 IDLE IRQ can decode frames. */
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

uint8_t RC_data_is_error(void)
{
    remote_control_can_timeout_check();
    return (rc_last_can_tick == 0 || (HAL_GetTick() - rc_last_can_tick) > 100u);
    /* CAN 遥控模式下，安全状态由 100 ms 超时检查统一维护。 */
/*
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
*/
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}


void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                rc_last_can_tick = HAL_GetTick();
                detect_hook(DBUS_TOE);
                sbus_to_usart1(sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                rc_last_can_tick = HAL_GetTick();
                detect_hook(DBUS_TOE);
                sbus_to_usart1(sbus_rx_buf[1]);
            }
        }
    }

}

static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 20);
}

 fp32 TOP_CHASSIS=0.0f;
//bool_t UseVision(void)
//{
//    if(rc_ctrl.mouse.press_r || gimbal_control_rc.gimbal_mode_data == GIMBAL_VISION)
//    {
//        return 1;
//    }
//    else 
//    {
//        return 0;
//    }

//			
//}

//bool_t UseVision(void)
//{
//    if(Follow_Vision)
//    {
//        return 1;
//    }
//    else 
//    {
//        return 0;
//    }

//}



