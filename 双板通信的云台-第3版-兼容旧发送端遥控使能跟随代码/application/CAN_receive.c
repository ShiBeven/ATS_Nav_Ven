#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_rng.h"
#include "detect_task.h"
#include "usb_task.h"
#include "gimbal_task.h"
#include <string.h>
#include "dm_imu.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

volatile float chassis_yaw_rx_rad;
volatile uint32_t chassis_yaw_sender_tick;
volatile uint32_t chassis_yaw_rx_ms;
volatile uint8_t chassis_yaw_seen;

refree1_ins_t yaw_acc;
#define get_motor_measure(ptr, data)                                 \
  {                                                                  \
    (ptr)->last_ecd = (ptr)->ecd;                                    \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);             \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);       \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);   \
    (ptr)->temperate = (data)[6];                                    \
    if ((ptr)->ecd - (ptr)->last_ecd > 4096)                         \
      (ptr)->round_cnt--;                                            \
    else if ((ptr)->ecd - (ptr)->last_ecd < -4096)                   \
      (ptr)->round_cnt++;                                            \
    (ptr)->total_angle = ((ptr)->round_cnt + 1) * 8192 + (ptr)->ecd; \
  }

//#define get_referee_data(ptr, data)                        \
//  {                                                        \
//    (ptr)->Vision_ID = (uint8_t)((data)[0]); \
//    (ptr)->shooter_17mm_1_barrel_heat = (uint16_t)((data)[1] << 8 | (data)[2]);                     \
//		 (ptr)->gimbal_state = (uint8_t)((data)[3]);              \
//    (ptr)->shooter_barrel_heat_limit = (uint16_t)((data)[4] << 8 | (data)[5]);                     \
//  }
#define get_referee_data(ptr, data)                        \
  {                                                        \
    (ptr)->Vision_ID = (uint8_t)((data)[0]); \
    (ptr)->shooter_17mm_barrel_heat = (uint16_t)((data)[1] << 8 | (data)[2]);                     \
		(ptr)->gimbal_state = (uint8_t)((data)[3]);              \
    (ptr)->shooter_barrel_heat_limit = (uint16_t)((data)[4] << 8 | (data)[5]);                     \
  }
  
  
	#define get_referee1_data(ptr, data)                        \
  {                                                        \
				memcpy(&(ptr)->launching_frequency,data,4);	                     \
  }
	#define get_referee2_data(ptr, data)                        \
  {                                                        \
				memcpy(&(ptr)->groy,data,4);	                     \
  }	  
	#define get_progress_data(ptr, data)                        \
  {                                                        \
				(ptr)->current_Hp = (uint16_t)((data)[0]<<8|(data)[1]);\
				(ptr)->game_progress = (uint8_t)((data)[2]);\
				(ptr)->shooter_barrel_cooling_value=(uint16_t)((data)[3]<<8|(data)[4]);\
  }	  
  #define get_time_data(ptr,data)							\
  {															\
	  (ptr)->stage_remain_time=(uint16_t)((data)[0]<<8|(data)[1]);	\
  }
  
  
  
/**
 * @brief  uint鏉炵惌loat
 * @param  x_int   閺堚偓婢堆嶆嫹?
 * @param  x_min   閺堚偓鐏忓骏鎷?
 * @param  bits    娴ｅ秵鏆?
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min) 
{ 
int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + 
(float)b_min + 0.5f; 
//閸?0.5 娴ｅ灝鎮滄稉瀣絿閺佹潙褰夐幋鎰磽閼稿秳绨查崗?
 return b; 
} 

/**
 * @brief  float鏉炵憚int
 * @param  x_int   閺堚偓婢堆嶆嫹?
 * @param  x_min   閺堚偓鐏忓骏鎷?
 * @param  bits    娴ｅ秵鏆?
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
motor_measure_t MOTOR_YAW2;
motor_measure_t ShootMotor[3];
motor_measure_t GimbalMotor[3];
static volatile uint32_t gimbal_motor_rx_ms[3];
static volatile uint8_t gimbal_motor_rx_seen[3];

bool_t gimbal_get_motor_snapshot(uint8_t index, motor_measure_t *out, uint32_t *age_ms)
{
    uint32_t irq, stamp;
    uint8_t seen;
    if (index >= 3U || out == NULL || age_ms == NULL) return 0;
    irq = __get_PRIMASK();
    __disable_irq();
    *out = GimbalMotor[index];
    stamp = gimbal_motor_rx_ms[index];
    seen = gimbal_motor_rx_seen[index];
    __set_PRIMASK(irq);
    *age_ms = HAL_GetTick() - stamp;
    return seen != 0U;
}



refree_data_t Refree_data;
refree1_data_t Refree1_data;//瀵綊鈧?
game_state_t game_statee;//濮ｆ棁绂岄悩鑸碘偓浣界箻缁?
stage_remain_time_t stage_remain_time;


static CAN_TxHeaderTypeDef shoot_tx_message;
static CAN_TxHeaderTypeDef board_tx_message;

static uint8_t shoot_can_send_data[8];
static uint8_t gimbal_GM6020_send_data[8];

static uint8_t board_send_data[8];
int16_t nav_data[3];
uint8_t nav_send_data[8];
Motor_t MOTOR1_t, MOTOR2_t, MOTOR3_t;
aaaz_t aazz;
int i = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  CAN_RxHeaderTypeDef rx_header;
	static CAN_RxPacketTypeDef packet;

  uint8_t rx_data[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
      return;
  }
  if (hcan->Instance == CAN2)
  {
    switch (rx_header.StdId)
    {
			
			case CAN_CHASSIS_YAW_ID:
{
    float yaw_rad;
    uint32_t sender_tick;

    if (rx_header.IDE == CAN_ID_STD &&
        rx_header.RTR == CAN_RTR_DATA &&
        rx_header.DLC == 8U)
    {
        memcpy(&yaw_rad, &rx_data[0], sizeof(yaw_rad));
        memcpy(&sender_tick, &rx_data[4], sizeof(sender_tick));

        if (yaw_rad >= -3.141593f && yaw_rad <= 3.141593f)
        {
            chassis_yaw_rx_rad = yaw_rad;
            chassis_yaw_sender_tick = sender_tick;
            chassis_yaw_rx_ms = HAL_GetTick();
            chassis_yaw_seen = 1U;
        }
    }
    break;
}
			
			case DM_IMU_MST_ID:
            if (rx_header.IDE == CAN_ID_STD && rx_header.RTR == CAN_RTR_DATA &&
                rx_header.DLC == 8U)
            {
                IMU_UpdateData(rx_data);
            }
            break;
			case CAN_Board_ID:
				if (rx_header.DLC == 8) remote_control_update_basic(rx_data);
				break;
			case CAN_Board2_ID:
				if (rx_header.DLC == 8) remote_control_update_aux(rx_data);
				break;
			case CAN_RC_MOUSE_ID:
				if (rx_header.DLC == 8) remote_control_update_mouse(rx_data);
				break;
			case CAN_GM6020_YAW_ID:
				if (rx_header.IDE == CAN_ID_STD && rx_header.RTR == CAN_RTR_DATA && rx_header.DLC == 8U)
      {
          get_motor_measure(&GimbalMotor[1], rx_data);
          gimbal_motor_rx_ms[1] = HAL_GetTick();
          gimbal_motor_rx_seen[1] = 1U;
      }
				break;
			case CAN_Board4_ID:
				get_referee_data(&Refree_data, rx_data);
				break;
			case CAN_Board6_ID:
				get_referee1_data(&Refree1_data, rx_data);
				break;
			case CAN_Board8_ID:
				get_progress_data(&game_statee,rx_data);
				break;			
			case CAN_INS_ID:
		    get_referee2_data(&yaw_acc, rx_data);
				break;
			case CAN_Board9_ID:
				get_time_data(&stage_remain_time,rx_data);
				break;
				
					
    }
		
  }
  else
  {
    switch (rx_header.StdId)
    {		
	
    case CAN_GM6020_PITCH_ID:
      if (rx_header.IDE == CAN_ID_STD && rx_header.RTR == CAN_RTR_DATA && rx_header.DLC == 8U)
      {
          get_motor_measure(&GimbalMotor[0], rx_data);
          gimbal_motor_rx_ms[0] = HAL_GetTick();
          gimbal_motor_rx_seen[0] = 1U;
      }
      break;
		case CAN_GM6020_YAW2_ID:
      if (rx_header.IDE == CAN_ID_STD && rx_header.RTR == CAN_RTR_DATA && rx_header.DLC == 8U)
      {
          get_motor_measure(&GimbalMotor[2], rx_data);
          gimbal_motor_rx_ms[2] = HAL_GetTick();
          gimbal_motor_rx_seen[2] = 1U;
      }
      break;
    case CAN_GM3508_RIGHT_FIRE_ID:
      get_motor_measure(&ShootMotor[2], rx_data);
      break;
    case CAN_GM3508_LEFT_FIRE_ID:
      get_motor_measure(&ShootMotor[1], rx_data);
      break;
    case CAN_GM2006_AMMUNITION_BOOSTER_ID:
      get_motor_measure(&ShootMotor[0], rx_data);
      break;  
  }
  }
}

/* Independent complete frames prevent stale slots/header sharing between
 * the task and TIM2 IRQ. Keep the mailbox selection/write atomic too. */
static void gimbal_send_one_axis(CAN_HandleTypeDef *hcan, uint32_t id,
                                 uint8_t byte_offset, int16_t command)
{
    CAN_TxHeaderTypeDef header = {0};
    uint16_t word = (uint16_t)command;
    uint8_t data[8] = {0};
    uint32_t mailbox, irq_state;
    header.StdId = id;
    header.RTR = CAN_RTR_DATA;
    header.IDE = CAN_ID_STD;
    header.DLC = 8U;
    data[byte_offset] = (uint8_t)(word >> 8);
    data[byte_offset + 1U] = (uint8_t)word;
    irq_state = __get_PRIMASK();
    __disable_irq();
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0U)
        (void)HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
    __set_PRIMASK(irq_state);
}

void CAN_cmd_gimbal(int16_t pitch)
{
    gimbal_send_one_axis(&hcan1, 0x1FFU, 2U, pitch);
}

void CAN_cmd_gimbal_2(int16_t yaw)
{
    gimbal_send_one_axis(&hcan2, 0x1FFU, 0U, yaw);
}

void CAN_cmd_gimbal_3(int16_t yaw2)
{
    gimbal_send_one_axis(&hcan1, 0x2FFU, 4U, yaw2);
}







void CAN_CmdShoot(int16_t right, int16_t left, int16_t Trigger,int16_t A)
{
  uint32_t send_mail_box;
  shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
  shoot_tx_message.IDE = CAN_ID_STD;
  shoot_tx_message.RTR = CAN_RTR_DATA;
  shoot_tx_message.DLC = 0x08;
  shoot_can_send_data[0] = right >> 8;
  shoot_can_send_data[1] = right;
  shoot_can_send_data[2] = left >> 8;
  shoot_can_send_data[3] = left;
  shoot_can_send_data[4] = Trigger >> 8;
  shoot_can_send_data[5] = Trigger;
	shoot_can_send_data[6] = A >> 8;
  shoot_can_send_data[7] = A;

  HAL_CAN_AddTxMessage(&hcan1, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

void CAN_CmdBoard5(uint8_t vision_state)
{
  uint32_t send_mail_box;
  board_tx_message.StdId = CAN_Board5_ID;
  board_tx_message.IDE = CAN_ID_STD;
  board_tx_message.RTR = CAN_RTR_DATA;
  board_tx_message.DLC = 0x08;
  board_send_data[0] = vision_state;

	
  HAL_CAN_AddTxMessage(&hcan2, &board_tx_message, board_send_data, &send_mail_box);
}

void CAN_SendGimbalState(void)
{
  uint32_t send_mail_box;
  gimbal_control_t *gimbal_data = get_gimlal_data();
  if (gimbal_data == NULL || gimbal_data->gimbal_yaw_motor.gimbal_motor_measure == NULL)
  {
    return;
  }
  uint16_t offset_ecd = gimbal_data->gimbal_yaw_motor.offset_ecd;
  uint16_t yaw_ecd = gimbal_data->gimbal_yaw_motor.gimbal_motor_measure->ecd;
  board_tx_message.StdId = CAN_GIMBAL_STATE_ID;
  board_tx_message.IDE = CAN_ID_STD;
  board_tx_message.RTR = CAN_RTR_DATA;
  board_tx_message.DLC = 8;
  board_send_data[0] = (uint8_t)(offset_ecd >> 8);
  board_send_data[1] = (uint8_t)offset_ecd;
  board_send_data[2] = (uint8_t)(yaw_ecd >> 8);
  board_send_data[3] = (uint8_t)yaw_ecd;
  board_send_data[4] = 1;
  board_send_data[5] = 0;
  board_send_data[6] = 0;
  board_send_data[7] = 0;
  /* 0x121 鎶婁簯鍙扮湡瀹炵紪鐮佸櫒鍙嶉鐙珛鍑烘潵锛岄伩鍏嶅拰閬ユ帶杈呭姪甯ф贩鐢ㄣ€?*/
  (void)HAL_CAN_AddTxMessage(&hcan2, &board_tx_message, board_send_data, &send_mail_box);
}
static void rc_put_i16_be(uint8_t *data, int16_t value)
{
  data[0] = (uint8_t)((uint16_t)value >> 8);
  data[1] = (uint8_t)value;
}

static void CAN_SendRcFrame(uint16_t id, const uint8_t data[8])
{
  uint32_t send_mail_box;
  board_tx_message.StdId = id;
  board_tx_message.ExtId = 0;
  board_tx_message.IDE = CAN_ID_STD;
  board_tx_message.RTR = CAN_RTR_DATA;
  board_tx_message.DLC = 8;
  board_tx_message.TransmitGlobalTime = DISABLE;
  (void)HAL_CAN_AddTxMessage(&hcan2, &board_tx_message, (uint8_t *)data, &send_mail_box);
}

void CAN_SendRcBasic(const RC_ctrl_t *rc)
{
  uint8_t data[8] = {0};
  if (rc == NULL) return;
  rc_put_i16_be(&data[0], rc->rc.ch[0]);
  rc_put_i16_be(&data[2], rc->rc.ch[1]);
  rc_put_i16_be(&data[4], rc->rc.ch[2]);
  data[6] = (uint8_t)rc->rc.s[1]; data[7] = (uint8_t)rc->rc.s[0];
  CAN_SendRcFrame(CAN_Board_ID, data);
}

void CAN_SendRcAux(const RC_ctrl_t *rc)
{
  uint8_t data[8] = {0};
  if (rc == NULL) return;
  rc_put_i16_be(&data[0], rc->rc.ch[4]);
  data[2] = (uint8_t)(rc->key.v >> 8); data[3] = (uint8_t)rc->key.v;
  CAN_SendRcFrame(CAN_Board2_ID, data);
}

void CAN_SendRcMouse(const RC_ctrl_t *rc)
{
  uint8_t data[8] = {0};
  if (rc == NULL) return;
  rc_put_i16_be(&data[0], rc->mouse.x);
  rc_put_i16_be(&data[2], rc->mouse.y);
  rc_put_i16_be(&data[4], rc->mouse.z);
  data[6] = rc->mouse.press_l; data[7] = rc->mouse.press_r;
  CAN_SendRcFrame(CAN_RC_MOUSE_ID, data);
}

void CAN_CmdBoard3(ReceivedPacketTwist_s *Navigation_speed_set)
{
  uint32_t send_mail_box;
  board_tx_message.StdId = CAN_Board3_ID;
  board_tx_message.ExtId = 0;
  board_tx_message.IDE = CAN_ID_STD;
  board_tx_message.RTR = CAN_RTR_DATA;
  board_tx_message.DLC = 0x08;
  board_tx_message.TransmitGlobalTime = DISABLE;
	nav_data[0]=float_to_int16(Navigation_speed_set->vx, 2, -2, 32000, -32000);
	nav_data[1]=float_to_int16(Navigation_speed_set->vy,2, -2, 32000, -32000);
	nav_data[2]=float_to_int16(Navigation_speed_set->wz, 6, -6, 32000, -32000);

  nav_send_data[0] = nav_data[0]>> 8;
  nav_send_data[1] = nav_data[0];
  nav_send_data[2] = nav_data[1]>> 8;
	nav_send_data[3] = nav_data[1];
  nav_send_data[4] = nav_data[2]>> 8;
  nav_send_data[5] = nav_data[2];
  /* The chassis protocol currently uses only vx/vy/wz (bytes 0..5).
   * Always clear the reserved tail so an old frame cannot leak into it. */
  nav_send_data[6] = 0U;
  nav_send_data[7] = 0U;
  HAL_CAN_AddTxMessage(&hcan2, &board_tx_message, nav_send_data, &send_mail_box);
}


/*DM Motor*/

/**
 * @brief  閻㈠灚婧€閸涙垝鎶?
 * @param  hcan   CAN閻ㄥ嫬褰為敓?
 * @param  ID     閺佺増宓佺敮褏娈慖D
 * @param  data   閸涙垝鎶ら敓?
 */

/**
 * @brief  MIT濡€崇础閹貉傜瑓閹貉冨煑閿?
 * @param  hcan   CAN閻ㄥ嫬褰為敓?
 * @param  ID     閺佺増宓佺敮褏娈慖D
 * @param  _pos   娴ｅ秶鐤嗙紒娆忕暰
 * @param  _vel   闁喎瀹崇紒娆忕暰
 * @param  _KP    娴ｅ秶鐤嗗В鏂剧伐缁粯鏆?
 * @param  _KD    娴ｅ秶鐤嗛敓?閸掑棛閮撮敓?
 * @param  _torq  閿?閻晝绮扮€规熬鎷?
 */
void MIT_CtrlMotor(CAN_HandleTypeDef *hcan, uint16_t ID, float _pos, float _vel, float _KP, float _KD, float _torq)
{
  static CAN_TxPacketTypeDef packet;
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

  packet.hdr.StdId = ID;
  packet.hdr.IDE = CAN_ID_STD;
  packet.hdr.RTR = CAN_RTR_DATA;
  packet.hdr.DLC = 0x08;

  pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
  kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
  kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

  packet.payload[0] = (pos_tmp >> 8);
  packet.payload[1] = pos_tmp;
  packet.payload[2] = (vel_tmp >> 4);
  packet.payload[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  packet.payload[4] = kp_tmp;
  packet.payload[5] = (kd_tmp >> 4);
  packet.payload[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  packet.payload[7] = tor_tmp;

  // 閹垫儳鍩岀粚铏规畱閸欐垿鈧線鍋栫粻鎲嬬礉閹跺﹥鏆熼敓?閸欐垿鈧礁鍤敓?
  if (HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //
  {
    if (HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
}

/**
 * @brief  娴ｅ秶鐤嗛柅鐔峰濡€崇础閹貉傜瑓閹貉冨煑閿?
 * @param  hcan   CAN閻ㄥ嫬褰為敓?
 * @param  ID     閺佺増宓佺敮褏娈慖D
 * @param  _pos   娴ｅ秶鐤嗙紒娆忕暰
 * @param  _vel   闁喎瀹崇紒娆忕暰
 */
void PosSpeed_CtrlMotor(CAN_HandleTypeDef *hcan, uint16_t ID, float _pos, float _vel)
{
  static CAN_TxPacketTypeDef packet;
  uint8_t *pbuf, *vbuf;
  pbuf = (uint8_t *)&_pos;
  vbuf = (uint8_t *)&_vel;

  packet.hdr.StdId = ID;
  packet.hdr.IDE = CAN_ID_STD;
  packet.hdr.RTR = CAN_RTR_DATA;
  packet.hdr.DLC = 0x08;

  packet.payload[0] = *pbuf;
  ;
  packet.payload[1] = *(pbuf + 1);
  packet.payload[2] = *(pbuf + 2);
  packet.payload[3] = *(pbuf + 3);
  packet.payload[4] = *vbuf;
  packet.payload[5] = *(vbuf + 1);
  packet.payload[6] = *(vbuf + 2);
  packet.payload[7] = *(vbuf + 3);

  // 閹垫儳鍩岀粚铏规畱閸欐垿鈧線鍋栫粻鎲嬬礉閹跺﹥鏆熼敓?閸欐垿鈧礁鍤敓?
  if (HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //
  {
    if (HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
}

/**
 * @brief  闁喎瀹冲Ο鈥崇础閹貉傜瑓閹貉冨煑閿?
 * @param  hcan   CAN閻ㄥ嫬褰為敓?
 * @param  ID     閺佺増宓佺敮褏娈慖D
 * @param  _vel   闁喎瀹崇紒娆忕暰
 */
void Speed_CtrlMotor(CAN_HandleTypeDef *hcan, uint16_t ID, float _vel)
{
  static CAN_TxPacketTypeDef packet;
  uint8_t *vbuf;
  vbuf = (uint8_t *)&_vel;

  packet.hdr.StdId = ID;
  packet.hdr.IDE = CAN_ID_STD;
  packet.hdr.RTR = CAN_RTR_DATA;
  packet.hdr.DLC = 0x04;

  packet.payload[0] = *vbuf;
  packet.payload[1] = *(vbuf + 1);
  packet.payload[2] = *(vbuf + 2);
  packet.payload[3] = *(vbuf + 3);

  // 閹垫儳鍩岀粚铏规畱閸欐垿鈧線鍋栫粻鎲嬬礉閹跺﹥鏆熼敓?閸欐垿鈧礁鍤敓?
  if (HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //
  {
    if (HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
}

/**
 * @brief  閻㈠灚婧€閸涙垝鎶?
 * @param  hcan   CAN閻ㄥ嫬褰為敓?
 * @param  ID     閺佺増宓佺敮褏娈慖D
 * @param  data   閸涙垝鎶ら敓?
 */
void Enable_CtrlMotor(CAN_HandleTypeDef *hcan, uint8_t ID, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7)
{
  static CAN_TxPacketTypeDef packet;

  packet.hdr.StdId = ID;
  packet.hdr.IDE = CAN_ID_STD;
  packet.hdr.RTR = CAN_RTR_DATA;
  packet.hdr.DLC = 0x08;
  packet.payload[0] = (uint8_t)data0;
  packet.payload[1] = (uint8_t)data1;
  packet.payload[2] = (uint8_t)data2;
  packet.payload[3] = (uint8_t)data3;
  packet.payload[4] = (uint8_t)data4;
  packet.payload[5] = (uint8_t)data5;
  packet.payload[6] = (uint8_t)data6;
  packet.payload[7] = (uint8_t)data7;

  /*閹垫儳鍩岀粚铏规畱閸欐垿鈧線鍋栫粻鎲嬬礉閹跺﹥鏆熼敓?閸欐垿鈧礁鍤敓?*/
  if (HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //
  {
    if (HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
}

const motor_measure_t *get_yaw2_gimbal_motor_measure_point(void)
{
  return &GimbalMotor[2];
}
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &GimbalMotor[1];
}

const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &GimbalMotor[0];
}

const motor_measure_t *get_shoot_trigger_measure_point(void)
{
  return &ShootMotor[0];
}

const motor_measure_t *get_shoot_fire_left_point(void)
{
  return &ShootMotor[1];
}

const motor_measure_t *get_shoot_fire_right_point(void)
{
  return &ShootMotor[2];
}
//const motor_measure_t *get_yaw2_gimbal_motor_measure_point(void)
//{
//		return &MOTOR_YAW2;
//}


