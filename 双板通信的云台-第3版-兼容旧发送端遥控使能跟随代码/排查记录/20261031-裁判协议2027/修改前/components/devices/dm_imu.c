#include "dm_imu.h"
#include <string.h>

imu_t imu;

uint8_t imu_get_gyro_snapshot(float gyro[3], uint32_t *age_ms)
{
    uint32_t irq_state, tick, now, age;
    uint8_t valid;
    irq_state = __get_PRIMASK();
    __disable_irq();
    valid = imu.gyro_valid;
    tick = imu.last_gyro_rx_tick;
    now = HAL_GetTick();
    gyro[0] = imu.gyro[0];
    gyro[1] = imu.gyro[1];
    gyro[2] = imu.gyro[2];
    __set_PRIMASK(irq_state);
    age = valid ? (uint32_t)(now - tick) : 0xFFFFFFFFU;
    if (age_ms != NULL) *age_ms = age;
    return valid && age <= 20U &&
           gyro[2] >= GYRO_CAN_MIN && gyro[2] <= GYRO_CAN_MAX;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void imu_init(uint8_t can_id,uint8_t mst_id,CAN_HandleTypeDef *hcan)
{
	imu.can_id=can_id;
	imu.mst_id=mst_id;
	imu.can_handle=hcan;
	
	imu.gyro_valid = 0U;
  imu.last_gyro_rx_tick = 0U;
	
}


/*
		发送指令
*/
static HAL_StatusTypeDef imu_send_cmd(uint8_t reg_id,uint8_t read_write,uint32_t value)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox;
    uint8_t data[8] =
    {
        0xCC,
        reg_id,
        read_write,
        0xDD,
        0x00, 0x00, 0x00, 0x00
    };

    if (imu.can_handle == NULL)
    {
        return HAL_ERROR;
    }

    memcpy(&data[4], &value, sizeof(value));

    tx_header.StdId = imu.can_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    if (HAL_CAN_GetTxMailboxesFreeLevel(imu.can_handle) == 0U)
    {
        return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(imu.can_handle,&tx_header,data,&mailbox);
}


void imu_write_reg(uint8_t reg_id,uint32_t data)
{
	imu_send_cmd(reg_id,CMD_WRITE,data);
}

void imu_read_reg(uint8_t reg_id)
{
	imu_send_cmd(reg_id,CMD_READ,0);
}

void imu_reboot()
{
	imu_write_reg(REBOOT_IMU,0);
}

void imu_accel_calibration()
{
	imu_write_reg(ACCEL_CALI,0);
}

void imu_gyro_calibration()
{
	imu_write_reg(GYRO_CALI,0);
}


void imu_change_com_port(imu_com_port_e port)
{
	imu_write_reg(CHANGE_COM,(uint8_t)port);
}

void imu_set_active_mode_delay(uint32_t delay)
{
	imu_write_reg(SET_DELAY,delay);
}

//设置成主动模式
void imu_change_to_active()
{
	imu_write_reg(CHANGE_ACTIVE,1);
}

void imu_change_to_request()
{
	imu_write_reg(CHANGE_ACTIVE,0);
}

void imu_set_baud(imu_baudrate_e baud)
{
	imu_write_reg(SET_BAUD,(uint8_t)baud);
}

void imu_set_can_id(uint8_t can_id)
{
	imu_write_reg(SET_CAN_ID,can_id);
}

void imu_set_mst_id(uint8_t mst_id)
{
	imu_write_reg(SET_MST_ID,mst_id);
}

void imu_save_parameters()
{
	imu_write_reg(SAVE_PARAM,0);
}

void imu_restore_settings()
{
	imu_write_reg(RESTORE_SETTING,0);
}


void imu_request_accel()
{
	imu_read_reg(ACCEL_DATA);
}

void imu_request_gyro()
{
	imu_read_reg(GYRO_DATA);
}

void imu_request_euler()
{
	imu_read_reg(EULER_DATA);
}

void imu_request_quat()
{
	imu_read_reg(QUAT_DATA);
}



void IMU_UpdateAccel(uint8_t* pData)
{
	uint16_t accel[3];
	
	accel[0]=pData[3]<<8|pData[2];
	accel[1]=pData[5]<<8|pData[4];
	accel[2]=pData[7]<<8|pData[6];
	
	imu.accel[0]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	imu.accel[1]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	imu.accel[2]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	
}

void IMU_UpdateGyro(uint8_t *pData)
{
    uint16_t gyro[3];

    if (pData == NULL)
    {
        return;
    }

    gyro[0] = (uint16_t)pData[2] |
              ((uint16_t)pData[3] << 8);

    gyro[1] = (uint16_t)pData[4] |
              ((uint16_t)pData[5] << 8);

    gyro[2] = (uint16_t)pData[6] |
              ((uint16_t)pData[7] << 8);

    imu.gyro[0] =
        uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);

    imu.gyro[1] =
        uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);

    imu.gyro[2] =
        uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);

    imu.last_gyro_rx_tick = HAL_GetTick();
    imu.gyro_valid = 1U;
}


void IMU_UpdateEuler(uint8_t* pData)
{
	int euler[3];
	
	euler[0]=pData[3]<<8|pData[2];
	euler[1]=pData[5]<<8|pData[4];
	euler[2]=pData[7]<<8|pData[6];
	
	imu.pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
	imu.yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
	imu.roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);
}


void IMU_UpdateQuaternion(uint8_t* pData)
{
	int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
	int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
	int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
	int z = (pData[6]&0x3F)<<8|pData[7];
	
	imu.q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
}

void IMU_UpdateData(uint8_t* pData)
{
	if (pData == NULL) return;
	switch(pData[0])
	{
		case 1:
			IMU_UpdateAccel(pData);
			break;
		case 2:
			IMU_UpdateGyro(pData);
			break;
		case 3:
			IMU_UpdateEuler(pData);
			break;
		case 4:
			IMU_UpdateQuaternion(pData);
			break;
	}
}
