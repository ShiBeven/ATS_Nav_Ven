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
#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H
#include "main.h"
#include "CAN_receive.h"
#include "referee.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024
#define UPDATE KEY_PRESSED_OFFSET_B
#define UI_SOF 0xA5
//typedef union {
//    uint32_t value;  // 用于打包数据传输
//    struct {
//        uint32_t enable       : 1;  // bit 0 - 启用哨兵
//        uint32_t patrol       : 1;  // bit 1 - 巡逻模式
//        uint32_t shoot        : 1;  // bit 2 - 射击允许
//        uint32_t follow       : 1;  // bit 3 - 跟随云台
//        uint32_t reserved     : 28; // bit 4~31 - 保留/扩展
//    } bits;
//} SentryCmd;

//// 数据头结构体
//typedef struct {
//    uint16_t data_cmd_id;
//    uint16_t sender_id;
//    uint16_t receiver_id;
//} DataHeader;

//// 帧头结构体
//typedef struct {
//    uint8_t SOF;
//    uint16_t data_length;
//    uint8_t seq;
//    uint8_t CRC8;
//} FrameHeader;

//// 总数据帧结构体
//typedef struct {
//    FrameHeader framehead;
//    uint16_t CMD_ID;
//    DataHeader datahead;
//    SentryCmd sentry_cmd;  // 这里用 union 结构
//} sentry_fh;
typedef struct __attribute__((packed)) {
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
} frame_header_t;

typedef struct __attribute__((packed)) {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
	  sentry_cmd_t SENTRY;
} data_header_t;

typedef struct __attribute__((packed)) {
    frame_header_t framehead;
    uint16_t CMD_ID;
    data_header_t datahead;
} sentry_fh;
//0x20D
// 提取 0x020D 哨兵自主决策信息的结构体
typedef struct {
    uint16_t exchanged_bullets;   // 已成功兑换的允许发弹量（bit0-10）
    uint8_t  remote_bullet_cnt;   // 成功远程兑换发弹次数（bit11-14）
    uint8_t  remote_heal_cnt;     // 成功远程兑换血量次数（bit15-18）
    uint8_t  can_free_respawn;    // 是否可以免费复活（bit19）
    uint8_t  can_pay_respawn;     // 是否可以兑换立即复活（bit20）
    uint16_t respawn_gold_cost;   // 兑换立即复活所需金币数（bit21-30）
    uint8_t  is_out_of_combat;    // 是否处于脱战状态（sentry_info_2 bit0）
    uint16_t team_17mm_remaining; // 队伍17mm允许发弹量剩余可兑换数（bit1-11）
    uint8_t  current_posture;     // 当前哨兵姿态 1进攻 2防御 3移动（bit12-13）
    uint8_t  can_activate_rune;   // 己方能量机关是否可激活（bit14）
} sentry_info_extracted_t;
//0x0101
//提取 0x0101 场地事件数据的结构体
// 解析 0x0101 场地事件数据的结构体
typedef struct {
    uint8_t  supply_zone_occupied;       // bit0: 己方补给区是否占领 (1为占领)
    uint8_t  small_rune_status;          // bits3-4: 小能量机关状态 0未激活,1已激活,2正在激活
    uint8_t  big_rune_status;            // bits5-6: 大能量机关状态 同上
    uint8_t  central_highland_owner;     // bits7-8: 中央高地占领方 1己方,2对方
    uint8_t  trapezoidal_highland_owned; // bits9-10: 梯形高地是否占领 (1为占领)
    uint16_t enemy_dart_last_hit_time;   // bits11-19: 对方飞镖最后一次击中时间 (0-420)
    uint8_t  enemy_dart_last_hit_target; // bits20-22: 击中目标 1前哨站,2基地固定,3随机固定,4随机移动,5末端移动
    uint8_t  center_gain_owner;          // bits23-24: 中心增益点占领方 0未占领,1己方,2对方,3双方 (仅RMUL)
    uint8_t  fort_gain_owner;            // bits25-26: 堡垒增益点占领方 同上
    uint8_t  outpost_gain_owner;         // bits27-28: 前哨站增益点占领方 0未占领,1己方,2对方
    uint8_t  base_gain_occupied;         // bit29: 基地增益点是否占领 (1为占领)
    // bits30-31 保留
} event_data_parsed_t;
//typedef  struct
//{
//  uint8_t SOF;//起始字节,固定0
//  uint16_t data_length;//帧数据长度
//  uint8_t seq;//包序号
//  uint8_t CRC8;//CRC8校验值
//} UI_Packhead;//帧头
//typedef __packed struct
//{
// uint16_t data_cmd_id;
// uint16_t sender_id;
// uint16_t receiver_id;
//}UI_Data_Operate;
//typedef struct
//{
//	 UI_Packhead framehead;		
//		uint16_t CMD_ID;
//		UI_Data_Operate datahead;
//    sentry_cmd_t SENTRY;
//		uint16_t CRC16;
//} sentry_fh;


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
extern	void robot_id(void);
void Sentry_FH_1(sentry_fh *draw);
void uart_send_test_frame(void);
extern void referee_usart_task(void const * argument);
extern uint8_t Vision_ID;
void send_test(sentry_fh *fh);
//void send_test(sentry_fh *fh,uint16_t ammo_x, uint8_t ammo_cnt, 
//             uint8_t hp_cnt, uint8_t posture, uint8_t energy_en);
void get_sentry_info(sentry_info_extracted_t *info);
void get_event_info(event_data_parsed_t *info);


#endif
