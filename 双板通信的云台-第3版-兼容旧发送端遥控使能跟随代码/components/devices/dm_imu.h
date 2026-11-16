#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "main.h"


#define DM_IMU_CAN_ID 0x030U
#define DM_IMU_MST_ID 0x031U

#define ACCEL_CAN_MAX (235.2f)
#define ACCEL_CAN_MIN	(-235.2f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

#define CMD_READ 0
#define CMD_WRITE 1

typedef enum
{
	COM_USB=0,
	COM_RS485,
	COM_CAN,
	COM_VOFA

}imu_com_port_e;

typedef enum
{
	CAN_BAUD_1M=0,
	CAN_BAUD_500K,
	CAN_BAUD_400K,
	CAN_BAUD_250K,
	CAN_BAUD_200K,
	CAN_BAUD_100K,
	CAN_BAUD_50K,
	CAN_BAUD_25K
	
}imu_baudrate_e;

typedef enum 
{
		REBOOT_IMU = 0x00,
    ACCEL_DATA = 0x01,
    GYRO_DATA  = 0x02,
    EULER_DATA = 0x03,
    QUAT_DATA  = 0x04,
    SET_ZERO   = 0x05,
    ACCEL_CALI = 0x06,
    GYRO_CALI  = 0x07,
    CHANGE_COM = 0x09,
    SET_DELAY  = 0x0A,
    CHANGE_ACTIVE = 0x0B,
    SET_BAUD   = 0x0C,
    SET_CAN_ID = 0x0D,
    SET_MST_ID = 0x0E,
    SAVE_PARAM = 0xFE,
    RESTORE_SETTING = 0xFF
}reg_id_e;



typedef struct
{
	uint8_t can_id;
	uint8_t mst_id;
	
	CAN_HandleTypeDef *can_handle;
	
	float pitch;
	float roll;
	float yaw;

	volatile float gyro[3];
	float accel[3];
	
	float q[4];

	float cur_temp;
	volatile uint32_t last_gyro_rx_tick;
	volatile uint8_t gyro_valid;
}imu_t;

extern imu_t imu;

void IMU_UpdateData(uint8_t *pData);
void imu_init(uint8_t can_id,uint8_t mst_id,CAN_HandleTypeDef *can_handle);
void imu_write_reg(uint8_t reg_id,uint32_t data);
void imu_read_reg(uint8_t reg_id);
void imu_reboot(void);
void imu_accel_calibration(void);
void imu_gyro_calibration(void);
void imu_change_com_port(imu_com_port_e port);
void imu_set_active_mode_delay(uint32_t delay);
void imu_change_to_active(void);
void imu_change_to_request(void);
void imu_set_baud(imu_baudrate_e baud);
void imu_set_can_id(uint8_t can_id);
void imu_set_mst_id(uint8_t mst_id);
void imu_save_parameters(void);
void imu_restore_settings(void);
void imu_request_accel(void);
void imu_request_gyro(void);
void imu_request_euler(void);
void imu_request_quat(void);

/* Coherent interrupt/task snapshot. Returns zero before first frame or timeout. */
uint8_t imu_get_gyro_snapshot(float gyro[3], uint32_t *age_ms);

#endif
