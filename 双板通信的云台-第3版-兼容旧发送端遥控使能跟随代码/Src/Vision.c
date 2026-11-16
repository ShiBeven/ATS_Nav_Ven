#include "Vision_Task.h"
#include "kalman.h"
#include "Vision.h"
#include "bsp_usart.h"
#include "stdlib.h"
#include "INS_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "math.h"
#include "CRC8_CRC16.h"
#include "CAN_receive.h"
#include <stdio.h>
#include "SolveTrajectory.h"
#include "math.h"
#include "usb_task.h"
#include "Filter.h"

//Vision_send_t Vision_send;
TX TXData;
TX TX_yaw_ecd, TX_pitch_ecd;
RX RxSet;
//gimbal_control_t gimbal_control;
//VisionRecvData_t InitZreo = {0};
//VisionRecvData_t VisionRecvData;
extern VisionData_t VisionData;
extern float add_angle;
int x = 0;
extKalman_t kalman_Yaw, kalman_Pitch;
float kalman_targetYaw, kalman_targetPitch;
float targetYaw, targetPitch;
uint8_t CRC_OK = 0;
void vision_send(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);
extern uint8_t Vision_state;
uint8_t Follow_Vision = 0;
uint8_t sv_buff[24];
extern RC_ctrl_t rc_ctrl;
float dis_diff_min;
float temp_dis_diff;
int idx;
float yaw_v = 0;
float bias_angle;
float yaw_speed=0;
 float predict_time_final=0;
uint8_t armor_choose = 0;
uint8_t suggest_fire = 0;
float allow_fire_ang_max = 0.f, allow_fire_ang_min = 0.f;
int  current_target =0;
float last_yaw_diff = .0f;  
float last_switch_time = 0;  
float FSH;
float FSH1;
 SolveTrajectoryParams st;
 tar_pos tar_position[4];
float t = 0.1f;
float aim_x = 0, aim_y = 0, aim_z = 0; // aim point 
// 1. 增强运动学估计器结构体
typedef struct {
    uint8_t header;          // 0xB5
    float target_yaw;
    float target_pitch;
    float pred_x, pred_y, pred_z,pred_yaw;
    uint8_t fire_cmd;
    uint16_t checksum;
} GimbalPacket;
PredictorConfig predictorConfigp;
MotionPredictor pred_st;
HistoryEntry history;
GimbalPacket gimbal_state;
tracking_state track_state;


#define YAW_MOTOR_RES_SPEED 1.5f//1.6f//
#define LARGE_ARMOR_WIDTH 0.25
#define SMALL_ARMOR_WIDTH 0.135
#define MAX_DETECT_DISTANCE 8
#define MAX_AREA 2

#define V_YAW 4.5

uint8_t Send_CheckSum(uint8_t *data, uint8_t len) //data????飬len????鳤??
{
    uint8_t i, crc = 0;

    for(i = 0; i < len; i++)
    {
        crc += *(data++);
    }
    crc = ~crc;
    return crc;
}

int see = 0;
int add = 0;


uint16_t test = 0;
uint8_t time = 0;
float LastPitchPrediction = 0., LastYawPrediction = 0.;


float get_delta_ang_pi(float angle1, float angle2) 
	{
    float delta = angle1 - angle2;
    
    // 将角度差值转换到 [-π, π] 范围
    while (delta > PI) delta -= 2 * PI;
    while (delta < -PI) delta += 2 * PI;
    
    return delta;
  }
int should_switch_target(float yaw_diff_1, float yaw_diff_2, float diff_angle,float  angle_of_advance)
	{
		
		if (sign(st.v_yaw)*yaw_diff_1 < diff_angle / 2 - angle_of_advance || angle_of_advance > diff_angle / 4) 
			{                     
				return 0;  // 
    } else if (sign(st.v_yaw)*yaw_diff_2 < diff_angle / 2 - angle_of_advance || angle_of_advance > diff_angle / 4){
        return 1;  // 
    }
    // ?
    if (current_target == 0 && (sign(st.v_yaw)*yaw_diff_1 < diff_angle / 2 - angle_of_advance|| angle_of_advance > diff_angle / 4)) {
        return 0;  // 
    }
    if (current_target == 1 && (sign(st.v_yaw)*yaw_diff_2 < diff_angle / 2 - angle_of_advance|| angle_of_advance > diff_angle / 4)) {
        return 1;  // 
    }
    	


    
    return -1;  // 
  }
// 初始化预测器
void predictor_init(MotionPredictor* p, PredictorConfig cfg) {
    memset(p, 0, sizeof(MotionPredictor));
    p->filtered_vx = 0;
    p->filtered_vy = 0;
    p->filtered_vz = 0;
    p->data_valid = 0;
	
	
}	
float median3(float a, float b, float c) {
    return (a > b) ? (b > c ? b : (a > c ? c : a)) 
                  : (a > c ? a : (b > c ? c : b));
}
uint32_t dt_ms;
float filtered_v_yaw ;
float dyaw;
 float yaw_buf[3] = {0};
void predictor_update(MotionPredictor* p, 
                     float x, float y, float z,
                     float vx, float vy, float vz,
                     float yaw,float v_yaw,
                     uint32_t timestamp) {
    // ===== 1. 数据有效性检查 =====
    static uint8_t init_counter = 0;
    if(p->history[0].timestamp == 0)
		{
			p->history[0].x=x;
			p->history[0].y=y;
			p->history[0].z=z;
			p->history[0].vx=vx;
			p->history[0].vy=vy;
			p->history[0].vz=x;
			p->history[0].timestamp=timestamp;
			init_counter++;
    }

		float dt;
		if(dt_ms!=0)
		{
		 dt=0.01;
		}
		
		

    // ===== 3. 速度动态滤波 =====
    // 计算速度变化率
    float dvx = vx - p->history[0].vx;
    float dvy = vy - p->history[0].vy;
    float dvz = vz - p->history[0].vz;
    float velocity_change = sqrtf(dvx*dvx + dvy*dvy + dvz*dvz);
    float velocity_magnitude = sqrtf(vx*vx + vy*vy + vz*vz);
    float change_ratio = velocity_change / (velocity_magnitude + 0.01f); // 避免除零
    
    // 动态调整滤波系数
    float alpha = 0.7f - 0.3f * fminf(change_ratio, 1.0f); // 变化越大滤波系数越小
    
    // 执行滤波
    p->filtered_vx = alpha * vx + (1-alpha) * p->filtered_vx;
    p->filtered_vy = alpha * vy + (1-alpha) * p->filtered_vy;
    p->filtered_vz = alpha * vz + (1-alpha) * p->filtered_vz;
		

    // ===== 4. 预测时间计算 =====
    float motion_intensity = fminf(velocity_magnitude / 4.0f, 1.0f); // 假设最大速度5m/s
    float predict_time = 0.012f + 0.30f * motion_intensity; // 150ms~500ms

    // ===== 5. 位置预测 =====
	 // ===== 5.5 状态判断 =====
 

		
		p->predicted_x = x + p->filtered_vx * predict_time;
    p->predicted_y = y + p->filtered_vy * predict_time; 
    p->predicted_z = z + p->filtered_vz * predict_time;

		
    // ===== 6. 偏航角预测 =====
    // 计算角速度（简化版）
    static float last_yaw = 0;
     dyaw = yaw - last_yaw;
    dyaw = fmodf(dyaw + PI, 2*PI) - PI; // 归一化到[-π, π]
//    float v_yaw = dyaw / dt;
    
    // 中值滤波
//    
    yaw_buf[2] = yaw_buf[1];
    yaw_buf[1] = yaw_buf[0];
    yaw_buf[0] = v_yaw;
//    filtered_v_yaw = median3(yaw_buf[0], yaw_buf[1], yaw_buf[2]);
       if(init_counter >= 3) 
				 {
						filtered_v_yaw = median3(yaw_buf[0], yaw_buf[1], yaw_buf[2]);
					
					// 自适应滤波：高速运动时增强滤波
					 float filter_strength = fmaxf(0.3f, 1.0f - fabsf(filtered_v_yaw)/PI);
					 filtered_v_yaw *= filter_strength;
					} else {
           filtered_v_yaw = v_yaw; // 初始化阶段直接使用原始值
         }
	  float yaw_speed_factor = 1.0f - fminf(fabsf(filtered_v_yaw)/PI, 0.8f);
    float yaw_predict_time = 0.05f * yaw_speed_factor; // 0.03s~0.15s
    p->predicted_yaw = yaw + filtered_v_yaw * yaw_predict_time;
    p->predicted_yaw = fmodf(p->predicted_yaw, 2*PI);
  		if(fabs(filtered_v_yaw)<V_YAW)
		  {
			  p->state=NOMAL;
			
			}
			else 
			{
			 p->state=Rotation;			
			}
    // ===== 7. 更新历史数据 =====
    memmove(&p->history[1], &p->history[0], 2*sizeof(p->history[0])); // 滑动窗口
//    memcpy(&p->history[0], &(typeof(p->history[0])), sizeof(p->history[0]));
    	p->history[0].x=x;
			p->history[0].y=y;
			p->history[0].z=z;
			p->history[0].vx=vx;
			p->history[0].vy=vy;
			p->history[0].vz=x;
			p->history[0].timestamp=timestamp;
      p->data_valid = 1;
    last_yaw = yaw;
}

void Vision_Kalman_Init(void)
{
}
//void GetIMUData(const float Acc[3], const float Gyro[3], const float Qart[4])
//{
//}

void Vision_init()
{
    st.k = 0;
    st.bullet_type =  BULLET_17;
	  st.state=0;
    st.current_v = 0;
    st.current_pitch = 0;
    st.current_yaw = 0;
    st.xw = 0;
    st.yw = 0;
    st.zw = 1.5;

    st.vxw = 0;
    st.vyw = 0;
    st.vzw = 0;
    st.v_yaw = 0;
    st.tar_yaw = 0;
    st.r1 = 0;
    st.r2 = 0;
    st.dz = 0;
    st.bias_time = 0;
    st.s_bias = 0;
    st.z_bias = 0;
    st.armor_id = ARMOR_INFANTRY3;
    st.armor_num = ARMOR_NUM_NORMAL;
		
}

//	void Update_CRC()
//	{
//		
//	st.mode = ReceivedPacketVision.mode;
//	st.k = 0.045;                                  // 空气阻力系数
//    st.bullet_type = BULLET_17;                    // 子弹类型
//    st.current_v = 22;                             // 子弹速度
//    st.current_pitch = VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle;  // 当前俯仰角
//    st.current_yaw = VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle;      // 当前偏航角
//	}









void Update_to_resolve()
{
	//st.mode = 0;
	st.mode = ReceivedPacketVision.mode;
    st.k = 0.045;                                  // 空气阻力系数
    st.bullet_type = BULLET_17;                    // 子弹类型
    st.current_v = 22;                             // 子弹速度
    st.current_pitch = VisionData.GetGimbalData->gimbal_pitch_motor.absolute_angle;  // 当前俯仰角
    st.current_yaw = VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle;      // 当前偏航角
    
    // 从视觉接收数据包更新参数
//    st.cap_timestamp = ReceivedPacketVision.cap_timestamp;  // 捕获时间戳
//    st.checksum = ReceivedPacketVision.checksum;            // 校验和
//    st.t_offset = ReceivedPacketVision.t_offset;            // 时间偏移
//    st.armor_id = ReceivedPacketVision.id;                  // 装甲板ID
//    st.state = ReceivedPacketVision.mode;                  // 状态
//    st.xw = ReceivedPacketVision.x;                         // x坐标
//    st.yw = ReceivedPacketVision.y;                         // y坐标
//    st.zw = ReceivedPacketVision.z;                         // z坐标
//    st.yaw = -ReceivedPacketVision.yaw;                      // 偏航角
//    st.vxw = ReceivedPacketVision.vx;                       // x方向速度
//    st.vyw = ReceivedPacketVision.vy;                       // y方向速度
//    st.vzw = ReceivedPacketVision.vz;                       // z方向速度
//    st.v_yaw = ReceivedPacketVision.yaw_vel;                // 偏航角速度
//    st.tar_yaw = ReceivedPacketVision.yaw;                  // 目标偏航角
//    st.r1 = ReceivedPacketVision.r1;                        // 半径1
//    st.r2 = ReceivedPacketVision.r2;                        // 半径2
//    st.dz = ReceivedPacketVision.dz;                        // 高度差
//    st.bias_time = acc;                                     // 偏置时间（0-200ms）
//    st.s_bias = 0.25084;                                    // 距离偏置
//    st.z_bias = 0.01687;                                    // 高度偏置
//    st.armor_num = ReceivedPacketVision.armors_num;         // 装甲板数量cketVision.armors_num;
		
}

float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
    if(t < 0)
    {
        return 0;
    }
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
t=0;
    return z;
}
//??????
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    for (i = 0; i < 25; i++)
    {
        angle_pitch = atan2f(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        if(z_actual == 0)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3f * (z - z_actual);
        z_temp = z_temp + dz;
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}
fp32 yaw_feed;
fp32 yaw_bsial=0;
float aim_yaw;
int16_t armor_board = 1;
float advance_hhh , different_hhh , different_angel_hhh;
extern Low_Pass_Filter_t v_yaw_Filter;
float yaw_add_angle = 0;
float pitch_yuan , pitch_buchang = 0;
float pitch_add_angle,add_high_pitch;
float cccccnm;
float all_distance;
float add_yaw;
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{         
	
	    if (Vision_state == 0) 
		{
				return;
    }
  float armor_x = 0.f, armor_y = 0.f, armor_z = 0.f;
	float est_x, est_y, est_z, est_yaw;;

   float predict_time_final = st.bias_time / 1000.0f + t;
		st.armor_num = 4;
    // Step 1: 目标中心预测
    float x_pred = st.xw + predict_time_final * st.vxw;
    float y_pred = st.yw + predict_time_final * st.vyw;
    float z_pred = st.zw + predict_time_final * st.vzw;
    float yaw_pred = st.tar_yaw + predict_time_final * v_yaw_Filter.out;
    float center_theta = atan2(y_pred, x_pred);

   
	uint8_t use_1 = 1;
    float diff_angle = 2 * PI / st.armor_num; //算出每个装甲板的角度差
		armor_choose = 0;
    for (size_t i = 0; i < st.armor_num; i++) //遍历四个装甲板
		{
      float armor_yaw = yaw_pred + i * diff_angle; //四个装甲板的角度
      float yaw_diff = get_delta_ang_pi(armor_yaw, center_theta); //装甲板角度和我与机器人中心的差值 
      if (fabsf(yaw_diff) < diff_angle / 2) //如果差值小于+-45°，即该装甲板在可击打范围内
				{
        armor_choose = 1; //标志位置1

        // Only 4 armors has 2 radius and height
        float r = st.r1;   //初始化
        armor_z = z_pred;  //初始化
        if (st.armor_num == 4) 
				{
          r = use_1 ? st.r1 : st.r2;  //根据装甲板的序号在两个不同的半径和高度之间交替切换
          armor_z = use_1 ? z_pred : (z_pred + st.dz);
        }

        // Robot state to armor
        armor_x = x_pred - r * cos(armor_yaw);  //通过机器人中心的坐标倒推出装甲板坐标
        armor_y = y_pred - r * sin(armor_yaw);

        // Calculate angle of advance

        float armor_z_next = z_pred; //确定下一块装甲板的高低和半径
        if (st.armor_num == 4) 
				{
          r = !use_1 ? st.r1 : st.r2;
          armor_z_next = !use_1 ? z_pred : (z_pred + st.dz);
        }
        float next_armor_yaw =  //预测下一块装甲板的角度，通过当前角度减去角度差
            armor_yaw -
            sign(v_yaw_Filter.out) * diff_angle;  // TODO: use future span to calculate
                                      // target delta angle
        float armor_x_next = x_pred - r * cos(next_armor_yaw); //确定下一块装甲板坐标
        float armor_y_next = y_pred - r * sin(next_armor_yaw);

        float yaw_motor_delta =  //算出如果要打下一块装甲板需要加的角度
            get_delta_ang_pi(atan2(armor_y, armor_x), atan2(armor_y_next, armor_x_next));
        float angle_of_advance =  //算出提前量
            fabsf(yaw_motor_delta) / YAW_MOTOR_RES_SPEED * fabsf(v_yaw_Filter.out) / 2;
						advance_hhh = angle_of_advance;
				different_hhh = yaw_diff;
				different_angel_hhh = diff_angle;
          
        if (sign(v_yaw_Filter.out) * yaw_diff < diff_angle / 2 - angle_of_advance ||angle_of_advance > diff_angle / 4) //判断选择当前装甲板
				{
          est_x = armor_x;
          est_y = armor_y;
          est_z = armor_z;
          est_yaw = armor_yaw;
					armor_board = 1;
        } else {
          est_x = armor_x_next;
          est_y = armor_y_next;
          est_z = armor_z_next;
          est_yaw = next_armor_yaw;
 					armor_board = 0;
        }
				break;
			}
        use_1 = !use_1;
		}

    *aim_x = est_x + st.vxw * predict_time_final;
    *aim_y = est_y + st.vyw * predict_time_final;
    *aim_z = est_z + st.vzw * predict_time_final;
		
		
		float dis_xy = sqrtf((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y));
		all_distance = dis_xy;
		if(st.v_yaw < 0.1 && st.v_yaw >-0.1)
		{
			add_yaw = 0;
		}
		if(st.v_yaw >= 0.1 && st.v_yaw < 1.0)
		{
			add_yaw = -0.02;
		}
		if(st.v_yaw >=1.0)
		{
			add_yaw = -0.05;
		}
		
		if(st.v_yaw <= -0.1 && st.v_yaw >-1.0)
		{
			add_yaw = 0.02;
		}
				if(st.v_yaw <= -1.0)
		{
			add_yaw = 0.05;
		}

		
		if(dis_xy>=2.5&&dis_xy<4)
		{
		pitch_add_angle=0.01;
		
		}
		if(dis_xy>=4)
		{
				pitch_add_angle=0.01;
		
		}
		if(dis_xy<2.5)
		{
			pitch_add_angle=-0.03;
		
		}
		
		
		if(st.zw>=0.0)
		{
			add_high_pitch = -0.01;
		}
	
    float pitch_temp = -pitchTrajectoryCompensation(dis_xy + st.s_bias, *aim_z - st.z_bias, st.current_v);
		pitch_yuan = pitch_temp;
    pitch_buchang = get_pitch_compensation(dis_xy);
    if (pitch_temp) 
			*pitch = pitch_temp + pitch_add_angle + add_high_pitch;
    if (*aim_x || *aim_y) 
			*yaw = atan2f(*aim_y, *aim_x) + yaw_add_angle+add_yaw;

//    if (Vision_state == 0) {
//        *pitch = 0;
//        // *yaw = 0; // 是否清零看你需求
//    }
		
		float armor_w = (st.armor_id == 1) ? LARGE_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
    float ax = est_x - 0.5f * armor_w * sinf(est_yaw);
		
    float ay = est_y + 0.5f * armor_w * cosf(est_yaw);
    float bx = est_x + 0.5f * armor_w * sinf(est_yaw);
    float by = est_y - 0.5f * armor_w * cosf(est_yaw);
    float angle_a = atan2f(ay, ax);
    float angle_b = atan2f(by, bx);
    float angle_c = atan2f(est_y, est_x);
    float allow_fire_ang_max = angle_c - angle_b;
    float allow_fire_ang_min = angle_c - angle_a;

    float control_delta_angle = get_delta_ang_pi(*yaw , VisionData.GetGimbalData->gimbal_yaw_motor.absolute_angle);
		cccccnm = control_delta_angle;
    shoot_control = (Vision_state == 1 && armor_choose &&
                    control_delta_angle < allow_fire_ang_max * 2&&
                    control_delta_angle > allow_fire_ang_min * 2);
}


		
   
float get_pitch_compensation(float distance) {
    // 补偿数据点
    const PitchCompensationPoint table[] = {
        {1.0f, -0.15f},
        {1.5f, -0.05f},
        {2.0f, -0.03f},
        {2.5f, -0.02f},
        {3.0f, -0.02f},
        {3.5f, -0.02f},
        {4.0f,  0.00f},
        {4.5f,  0.00f},
        {5.0f,  0.00f},
        {5.5f,  0.00f},
        {6.0f,  0.02f},
    };

    const int table_size = sizeof(table) / sizeof(table[0]);

    // 边界处理：小于最小值
    if (distance <= table[0].distance) {
        return table[0].compensation;
    }

    // 边界处理：大于最大值
    if (distance >= table[table_size - 1].distance) {
        return table[table_size - 1].compensation;
    }

    // 区间线性插值
    for (int i = 0; i < table_size - 1; ++i) {
        if (distance >= table[i].distance && distance <= table[i + 1].distance) {
            float x0 = table[i].distance;
            float y0 = table[i].compensation;
            float x1 = table[i + 1].distance;
            float y1 = table[i + 1].compensation;

            // 线性插值公式
            return y0 + (y1 - y0) * (distance - x0) / (x1 - x0);
        }
    }

    return 0.0f;  // 不应到达这里
}


int send_time = 0;
void calcuateBuffPostition(float xc, float yc, float zc, float theta,
                           int blade_id, float a, float b, float w,
                           uint64_t cap_timestamp, uint16_t t_offset,
                           float* target_x, float* target_y, float* target_z) {
  // TODO: obtain self timestamp and set delay
  uint64_t self_timestamp = 1234502;  // ms stamp
  float delay = 0.3;                  // s

  //   TODO end

  float r = 0.7;

  float t0 = (float)(cap_timestamp + t_offset) / 1000;
  float t1 = (float)(self_timestamp+ t_offset) / 1000 + delay;
  if (w == 0) {
    theta += b * (t1 - t0);
  } else {
    theta += a / w * (cos(w * t0) - cos(w * t1)) + b * (t1 - t0);
  }
  *target_x = xc + r * (sin(theta) * yc / sqrt(pow(xc, 2) + pow(yc, 2)));
  *target_y = yc + r * (-sin(theta) * xc / sqrt(pow(xc, 2) + pow(yc, 2)));
  *target_z = zc + r * cos(theta);
}

void Buff_Calcuate() 
	{
	 calcuateBuffPostition(st.xw,st.yw,st.zw,st.yaw,st.armor_id,st.vxw,st.vyw,st.vzw,st.cap_timestamp,st.t_offset, &aim_x, &aim_y, &aim_z);
 }
	
 

 