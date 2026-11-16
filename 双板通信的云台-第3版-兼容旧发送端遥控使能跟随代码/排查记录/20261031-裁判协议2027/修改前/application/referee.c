#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_status_t game_state;							/*0X0001*/
game_result_t game_result;							/*0X0002*/
game_robot_HP_t game_robot_HP;							/*0X0003*/
event_data_t field_event;								/*0X0101*/
//ext_supply_projectile_action_t supply_projectile_action_t;/*0X0102*/
//ext_supply_projectile_booking_t supply_projectile_booking_t;/*0X0103*/
referee_warning_t referee_warning;						/*0x0104*/
dart_info_t dart_info;									/*0X0105*/
robot_status_t robot_state;							    /*0X0201*/
power_heat_data_t power_heat_data;						/*0X0202*/
robot_pos_t game_robot_pos;								/*0X0203*/
buff_t buff;											/*0X0204*/
//air_support_data_t robot_energy;													/*0X0205*/
hurt_data_t robot_hurt;									/*0X0206*/
shoot_data_t shoot_data;								/*0X0207*/
projectile_allowance_t projectile_allowance;			/*0X0208*/
rfid_status_t robot_rfid;								/*0x0209*/
dart_client_cmd_t dart_client_cmd;						/*0X020A*/
ground_robot_position_t ground_robot_position;			/*0X020B*/
radar_mark_data_t radar_mark_data;						/*0X020C*/
sentry_info_t SentryInfo;								/*0X020D*/
radar_info_t RadarInfo;	                 				/*0X020E*/
robot_interaction_data_t robot_interaction_data;		/*0X0301*/
interaction_layer_delete_t interaction_layer_delete;	/*0X0100*/
interaction_figure_t interaction_figure;				/*0X0101*/
interaction_figure_2_t interaction_figure_2;			/*0X0102*/
interaction_figure_3_t interaction_figure_3;			/*0X0103*/
interaction_figure_4_t interaction_figure_4;			/*0X0104*/
//ext_client_custom_character_t ext_client_custom_character;/*0X0110*/
//哨兵自主决策指令
sentry_cmd_t sentry_cmd;								/*0X0120*/

radar_cmd_t radar_cmd;									/*0X0121*/
robot_interaction_data_t robot_interaction_data;		/*0X0301*/
custom_robot_data_t custom_robot_data;					/*0X0302*/
map_command_t map_command;								/*0X0303*/
//remote_control_t remote_control;														/*0X0304*/
map_robot_data_t map_robot_data;						/*0X0305*/
custom_client_data_t custom_client_data;				/*0X0306*/
//哨兵机器人或半自动控制方式的机器人可通过常规链路向对应的操作手选手端发送路径坐标数据，该路径会在小地图上显示
map_data_t map_data;									/*0X0307*/

custom_info_t custom_info;								/*0X0308*/
robot_custom_data_t robot_custom_data;                  /*0X0309*/
robot_custom_data_2_t robot_custom_data_2; 				/*0X0310*/
robot_custom_data_3_t robot_custom_data_3; 				/*0X0311*/

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));
    memset(&game_state, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));
    memset(&field_event, 0, sizeof(event_data_t));
    //memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
//    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));
    memset(&robot_state, 0, sizeof(robot_status_t));
    memset(&dart_info, 0, sizeof(dart_info_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&game_robot_pos, 0, sizeof(robot_pos_t));
    memset(&buff, 0, sizeof(buff_t));
    //memset(&robot_energy, 0, sizeof(air_support_data_t));
    memset(&robot_hurt, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
    memset(&robot_rfid, 0, sizeof(rfid_status_t));
    memset(&dart_client_cmd, 0, sizeof(dart_client_cmd_t));
    memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
    memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));
    memset(&SentryInfo, 0, sizeof(sentry_info_t));
    memset(&RadarInfo, 0, sizeof(radar_info_t));
    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
    memset(&interaction_layer_delete, 0, sizeof(interaction_layer_delete_t));
    memset(&interaction_figure, 0, sizeof(interaction_figure_t));
    memset(&interaction_figure_2, 0, sizeof(interaction_figure_2_t));
    memset(&interaction_figure_3, 0, sizeof(interaction_figure_3_t));
    memset(&interaction_figure_4, 0, sizeof(interaction_figure_4_t));
	
    memset(&sentry_cmd, 0, sizeof(sentry_cmd_t));
    memset(&radar_cmd, 0, sizeof(radar_cmd_t));
	
	memset(&map_command, 0, sizeof(map_command_t));
	memset(&map_robot_data, 0, sizeof(map_robot_data_t));
	memset(&map_data, 0, sizeof(map_data_t));
	memset(&custom_info, 0, sizeof(custom_info_t));

	
    memset(&custom_robot_data, 0, sizeof(custom_robot_data_t));
	memset(&robot_custom_data, 0, sizeof(robot_custom_data_t));
	memset(&robot_custom_data_2, 0, sizeof(robot_custom_data_t));
	memset(&robot_custom_data_3, 0, sizeof(robot_custom_data_t));
	memset(&custom_robot_data, 0, sizeof(custom_client_data_t));


}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
    case GAME_STATE_CMD_ID:
    {
        memcpy(&game_state, frame + index, sizeof(game_state));
    }
    break;
    case GAME_RESULT_CMD_ID:
    {
        memcpy(&game_result, frame + index, sizeof(game_result));
    }
    break;
    case GAME_ROBOT_HP_CMD_ID:
    {
        memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP));
    }
    break;
    case FIELD_EVENTS_CMD_ID:
    {
        memcpy(&field_event, frame + index, sizeof(field_event));
    }
    break;
//    case SUPPLY_PROJECTILE_ACTION_CMD_ID:
//    {
//        memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
//    }
//    break;
//    case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
//    {
//        memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
//    }
//    break;
    case REFEREE_WARNING_CMD_ID:
    {
        memcpy(&referee_warning, frame + index, sizeof(referee_warning));
    }
    break;

    case ROBOT_STATE_CMD_ID:
    {
        memcpy(&robot_state, frame + index, sizeof(robot_state));
    }
		break;

    case DART_INFO_CMD_ID:
    {
        memcpy(&dart_info, frame + index, sizeof(dart_info));
    }
    break;
    case POWER_HEAT_DATA_CMD_ID:
    {
        memcpy(&power_heat_data, frame + index, sizeof(power_heat_data));
    }
    break;
    case ROBOT_POS_CMD_ID:
    {
        memcpy(&game_robot_pos, frame + index, sizeof(game_robot_pos));
    }
    break;
    case BUFF_MUSK_CMD_ID:
    {
        memcpy(&buff, frame + index, sizeof(buff));
    }
    break;
//    case AERIAL_ROBOT_ENERGY_CMD_ID:
//    {
//        memcpy(&robot_energy, frame + index, sizeof(robot_energy));
//    }
//    break;
    case ROBOT_HURT_CMD_ID:
    {
        memcpy(&robot_hurt, frame + index, sizeof(robot_hurt));
    }
    break;
    case SHOOT_DATA_CMD_ID:
    {
        memcpy(&shoot_data, frame + index, sizeof(shoot_data));
    }
    break;
    case BULLET_REMAINING_CMD_ID:
    {
        memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance));
    }
    break;
    case ROBOT_RFID_STATE:
    {
        memcpy(&robot_rfid, frame + index, sizeof(robot_rfid));
    }
    break;
    case DART_CLIENT_CMD_ID:
    {
        memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd));
    }
    break;
    case GROUND_ROBOT_POSITION_CMD_ID:
    {
        memcpy(&ground_robot_position, frame + index, sizeof(ground_robot_position));
    }
    break;
    case RADAR_MARK_DATA_CMD_ID:
    {
        memcpy(&radar_mark_data, frame + index, sizeof(radar_mark_data));
    }
    break;
    case SENTRY_INFO_CMD_ID:
    {
        memcpy(&SentryInfo, frame + index, sizeof(SentryInfo));
    }
    break;

    case RADAR_INFO_CMD_ID:
    {
        memcpy(&RadarInfo, frame + index, sizeof(RadarInfo));
    }
    break;
    case ROBOT_INTERACTION_DATA_CMD_ID:
    {
        memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data));
    }
		  break;
		case CUSTOM_ROBOT_DATA_ID:
		{
        memcpy(&custom_robot_data, frame + index, sizeof(custom_robot_data));
    }
		  break;
		

    default:
    {
        break;
    }
    }
}




