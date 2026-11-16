#ifndef PID_H
#define PID_H
#include "struct_typedef.h"

#define LIMIT(x,min,max) (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define ABS(x) (((x)>0)?(x):(-(x)))

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;

    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  
    fp32 max_iout; 

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  
    fp32 error[3]; 

} pid_type_def;




typedef struct {
    float C;
    float K;
    float ref;
    float error_eps;
    float u_max;
    float J;
    float angle;
    float ang_vel;
    float epsilon;

    float u;

    // 私有成员变量
    float error;
    float error_last;
    float dref;
    float ddref;
    float refl;
    float s;
} SMC;

extern SMC YawSMC;

extern pid_type_def pid_poke;
extern pid_type_def pid_poke_omg;


extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

extern void PID_clear(pid_type_def *pid);
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc_Ecd(pid_type_def *pid, fp32 ref, fp32 set, uint16_t ecd_range);
extern fp32 PID_Calc_Angle(pid_type_def *pid, fp32 ref, fp32 set);
extern void SMC_Init(SMC* smc, float C, float K, float ref, float error_eps, float u_max, float J, float epsilon);
extern void SMC_Tick(SMC* smc, float angle_now, float angle_vel);
#endif
