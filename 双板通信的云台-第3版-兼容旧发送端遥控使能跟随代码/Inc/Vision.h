#ifndef VISION_H
#define VISION_H

#include "main.h"
#include "user_lib.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "gimbal_behaviour.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "INS_task.h"

#include "my_pid.h"
#include "struct_typedef.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "SolveTrajectory.h"
#define VISION_HUART huart6
#define USE_NEWCODE 0
typedef union
{
    float f_data;
    uint8_t byte[4];
} FloatUChar_t;

#pragma pack(push, 1)

//typedef  struct
//{
//	uint8_t head;
//	uint8_t detect_color : 1;  // 0-red 1-blue
//  bool tracker : 1;
//  uint8_t reserved : 6;
//  float pitch;
//  float yaw;
//  float aim_x;
//  float aim_y;
//  float aim_z;
//	uint16_t check_num;
//} Vision_send_t;


//typedef  struct
//{
//  uint8_t header;
//	union 
//	{
//		uint8_t all;
//		struct{
//			bool tracking :1 ;
//			uint8_t id :3 ;
//			uint8_t armors_num :3 ;  // 2-balance 3-outpost 4-normal
//			uint8_t reserved : 1;
//			    }bit;
//		}data;
//  float x;
//  float y;
//  float z;
//  float yaw;
//  float vx;
//  float vy;
//  float vz;
//  float v_yaw;
//  float r1;
//  float r2;
//  float dz;
//  uint16_t checksum;
//}VisionRecvData_t;
// 预测器参数配置
typedef enum
{
  NOMAL = 0,
  Rotation,
} tracking_state;

typedef struct {
    float pos_filter_alpha;    // 位置滤波系数
    float vel_filter_base;     // 速度滤波基准系数
    float base_predict_time;   // 基础预测时间(s)
    float max_predict_time;    // 最大预测时间(s)
}PredictorConfig;
typedef struct {
    float x, y, z;
    float vx, vy, vz;
    uint32_t timestamp;
} HistoryEntry;
// 预测器状态
typedef struct {
    // 历史数据缓存
    struct {
        float x, y, z;
        float vx, vy, vz;
        uint32_t timestamp;
    } history[3];
    
    // 滤波状态
    float filtered_vx, filtered_vy, filtered_vz;
    float filtered_yaw;
    
    // 预测输出
    float predicted_x, predicted_y, predicted_z;
    float predicted_yaw;
		int state;
		
    uint8_t data_valid;
} MotionPredictor;
typedef union
{
    float f_data;
    uint8_t byte[4];
} TX;

typedef union
{
	  float f_data;
    uint8_t byte[4];
    
} RX;

typedef struct {
    float distance;
    float compensation;  //补偿
} PitchCompensationPoint;
float get_pitch_compensation(float distance);
 

extern void LanggoUartFrameIRQHandler(UART_HandleTypeDef *huart);
//extern VisionRecvData_t VisionRecvData;
extern void Vision_task(void const *pvParameters);
extern uint8_t Follow_Vision;
extern void Kalman_Filter(void);
extern float kalman_targetYaw, kalman_targetPitch;
extern void Vision_Kalman_Init(void);
void autoSolveTrajectory(float *pitch, float *yaw,float *aim_x, float *aim_y, float *aim_z);
//extern  VisionRecvData_t* AimDataUpdate(void);
extern void GetIMUData(const float Acc[3], const float Gyro[3], const float Qart[4]);
extern float aim_x , aim_y , aim_z ; // aim point 落点，传回上位机用于可视化

extern void Vision_init(void);
extern void Update_to_resolve(void);
// extern void GetIMUData(const float Acc[3], const float Gyro[3]);
extern void predictor_init(MotionPredictor* p, PredictorConfig cfg);
 float get_delta_ang_pi(float angle1, float angle2) ;
#pragma pack(pop)

#endif


