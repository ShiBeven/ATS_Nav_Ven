#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__
#include <stdint.h>
#include <stdbool.h>
#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78f
typedef unsigned char uint8_t;
enum ARMOR_ID
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
};

enum ARMOR_NUM
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
    BULLET_17 = 0,
    BULLET_42 = 1
};


//????
typedef struct 
{
    float k;            

    
    enum BULLET_TYPE bullet_type;  
    float current_v;      
    float current_pitch;  
    float current_yaw;    
    uint8_t  state;
	
    float xw;             
    float yw;            
    float zw;         
    float yaw;
    float vxw;         
    float vyw;         
    float vzw;           
    float tar_yaw;        
    float v_yaw;         
    float r1;             
    float r2;            
    float dz;            
    float bias_time;        
    float s_bias;         
    float z_bias;         
	double area;
	
	float mode;
	
	uint32_t cap_timestamp;
  	uint16_t t_offset;
    uint16_t checksum;
	
    uint8_t armor_id;     
                                 
    uint8_t armor_num;   
}SolveTrajectoryParams;

//????????????
typedef struct 
{
    float x;           //???????????x
    float y;           //???????????y
    float z;           //???????????z
    float yaw;         //???????????????yaw?
}tar_pos;
//?????????
extern float monoDirectionalAirResistanceModel(float s, float v, float angle);
//????????
extern float completeAirResistanceModel(float s, float v, float angle);
//pitch????
extern float pitchTrajectoryCompensation(float s, float y, float v);
//?????????????? ??????
extern void autoSolveTrajectory(float *pitch, float *yaw,float *aim_x, float *aim_y, float *aim_z);


#endif /*__SOLVETRAJECTORY_H__*/
