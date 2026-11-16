/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
	
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "CAN_receive.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"

/*USART6*/


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

extern UART_HandleTypeDef huart6;
sentry_fh draw_data = {0};     // 自动初始化为全 0
sentry_info_extracted_t info={0};
event_data_parsed_t event_info={0};
// 机器人控制指令数据
extern ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA; 
uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
//extern custom_flag;
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */


char shoot_arr[5]="shoot";
char flrb_arr[4]="FRBL";
/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
		uint8_t updata=0;
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);

    while(1)
    {
			robot_id();
			updata++;
	  referee_unpack_fifo_data();
			send_test(&draw_data);//周期发送哨兵指令
			get_sentry_info(&info);
			get_event_info(&event_info);
			osDelay(100);//
			
	}
}
	

uint8_t Vision_ID,red_outpost,blue_outpost;
void robot_id(void)
{
    switch(robot_state.robot_id)
    {
        // 这些 ID 的 Vision_ID 设为 1
        case 1: case 2: case 3: case 4: case 5: case 7: case 101:
            Vision_ID = 1;
            break;

        // 这些 ID 的 Vision_ID 设为 0
        case 102: case 103: case 104: case 105: case 107:
            Vision_ID = 0;
            break;

        default:
            // 其他 ID 处理方式（如果需要）
            Vision_ID = -1; // 可根据需求调整默认值
            break;
    }

}

 uint8_t UI_Seq1=0;
int robot_ID;

uint8_t hhg_test[20];

static uint16_t bullet_exchange_total=0;
static uint16_t remote_bullet_counter=0;
static uint16_t retome_hp_counter=0;
void send_test(sentry_fh *fh)
{
	fh->framehead.SOF = 0xA5;
	fh->framehead.data_length = 10;
	fh->framehead.seq = UI_Seq1++;
	
	memcpy(&hhg_test[0],&fh->framehead,4);
	append_CRC8_check_sum(hhg_test,5);
	
	draw_data.CMD_ID = 0x0301;
	memcpy(&hhg_test[5],(const uint8_t *)&fh->CMD_ID,2);
	
	fh->datahead.data_cmd_id = 0x0120;
	fh->datahead.sender_id = robot_state.robot_id;
	fh->datahead.receiver_id = 0x8080;
	fh->datahead.SENTRY.sentry_cmd = 0;
	//bit 0:确认复活（1：确认复活）
	sentry_info_extracted_t info;
	get_sentry_info(&info);
	if(info.can_free_respawn)
	{
		fh->datahead.SENTRY.sentry_cmd |= (1 << 0);
	}	
//	   
//	//bit 1:确认兑换立即复活（0：不确认）
//	fh->datahead.SENTRY.sentry_cmd |= (1 << 1);
//	//bit 2-12：兑换发弹量（例如500发）
//	bullet_exchange_total+=500;
//	fh->datahead.SENTRY.sentry_cmd |= ((bullet_exchange_total&0x7FF)<<2);
//	//bit 13-16:远程兑换发弹量的请求次数(例如1次)
//	remote_bullet_counter++;
//	fh->datahead.SENTRY.sentry_cmd |=((remote_bullet_counter&0xF)<<13);
//	//bit 17-20:远程兑换血量的请求次数(例如0次)
//	retome_hp_counter++;
//	fh->datahead.SENTRY.sentry_cmd |=((retome_hp_counter&0xF)<<17);
	uint8_t posture=3;
	if(RECEIVE_ROBOT_CMD_DATA.data.speed_vector.mode ==1)
	{
		posture=1;//移动
	}	
	if(RECEIVE_ROBOT_CMD_DATA.data.speed_vector.mode==2)
	{
		posture=2;//进攻
	}	
	if(RECEIVE_ROBOT_CMD_DATA.data.speed_vector.mode==3)
	{
		posture=3;//防御
	}	
	//bit 21-22:姿态指令(1:进攻姿态,2:防御姿态,3:移动姿态,默认为3;修改此值可改变哨兵姿态)
	fh->datahead.SENTRY.sentry_cmd |=((posture&0x3)<<21);
//	fh->datahead.SENTRY.sentry_cmd |=((1&0x3)<<21);

	//bit 23:哨兵机器人是否确认使能量机关进入正在激活状态,1为确认.默认为0
//	fh->datahead.SENTRY.sentry_cmd |=(1<<23);  
  //bit 24-31:保留位，默认为0

//	memcpy(&hhg_test[7],&draw_data.datahead,17);
	memcpy(&hhg_test[7],&fh->datahead,sizeof(data_header_t));
	
	append_CRC16_check_sum(hhg_test,19);
	HAL_UART_Transmit(&huart6, hhg_test, 19, HAL_MAX_DELAY);
}

/**
  * @brief  提取 0x020D 哨兵自主决策信息同步的所有字段
  * @param  info: 输出参数，存放提取后的数据
  * @retval None
  */
void get_sentry_info(sentry_info_extracted_t *info)
{
    if (info == NULL) return;

    // 提取 sentry_info 字段
    info->exchanged_bullets  = (SentryInfo.sentry_info >> 0)  & 0x7FF;  // bit 0-10
    info->remote_bullet_cnt  = (SentryInfo.sentry_info >> 11) & 0x0F;   // bit 11-14
    info->remote_heal_cnt    = (SentryInfo.sentry_info >> 15) & 0x0F;   // bit 15-18
    info->can_free_respawn   = (SentryInfo.sentry_info >> 19) & 0x01;   // bit 19
    info->can_pay_respawn    = (SentryInfo.sentry_info >> 20) & 0x01;   // bit 20
    info->respawn_gold_cost  = (SentryInfo.sentry_info >> 21) & 0x3FF;  // bit 21-30

    // 提取 sentry_info_2 字段
    info->is_out_of_combat   = (SentryInfo.sentry_info_2 >> 0)  & 0x01;   // bit 0
    info->team_17mm_remaining= (SentryInfo.sentry_info_2 >> 1)  & 0x7FF;  // bit 1-11
    info->current_posture    = (SentryInfo.sentry_info_2 >> 12) & 0x03;   // bit 12-13
    info->can_activate_rune  = (SentryInfo.sentry_info_2 >> 14) & 0x01;   // bit 14
}
/**
 * @brief 解析场地事件数据 (0x0101)
 * @param info 输出参数，存放解析后的字段
 * @note 调用前需确保 field_event 已被裁判系统更新
 */
void get_event_info(event_data_parsed_t *info)
{
    if (info == NULL) return;

//    uint32_t raw = field_event.event_data;  // field_event 是全局变量

    // 提取各个字段
    info->supply_zone_occupied     = (field_event.event_data>> 0)  & 0x01;   // bit0
    // bits1-2 保留，跳过
    info->small_rune_status        = (field_event.event_data >> 3)  & 0x03;   // bits3-4
    info->big_rune_status          = (field_event.event_data >> 5)  & 0x03;   // bits5-6
    info->central_highland_owner   = (field_event.event_data >> 7)  & 0x03;   // bits7-8
    info->trapezoidal_highland_owned = (field_event.event_data >> 9) & 0x03;   // bits9-10 (实际只需判1)
    info->enemy_dart_last_hit_time = (field_event.event_data >> 11) & 0x1FF;  // bits11-19
    info->enemy_dart_last_hit_target = (field_event.event_data >> 20) & 0x07; // bits20-22
    info->center_gain_owner        = (field_event.event_data >> 23) & 0x03;   // bits23-24
    info->fort_gain_owner          = (field_event.event_data >> 25) & 0x03;   // bits25-26
    info->outpost_gain_owner       = (field_event.event_data >> 27) & 0x03;   // bits27-28
    info->base_gain_occupied       = (field_event.event_data   >> 29) & 0x01;   // bit29
}
/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step) 
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
             referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}


void USART6_IRQHandler(void)
{
    static volatile uint8_t res;

	if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);

        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
        }
    }
}


