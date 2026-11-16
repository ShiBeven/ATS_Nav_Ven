#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include <stdio.h>
#include "stdbool.h"
#include "string.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "arm_math.h"
#include <math.h>
#include <stdlib.h>

/*************一阶卡尔曼**************/
typedef struct
{
  float X_last; //上一时刻的最优结果  X(k-|k-1)
  float X_mid;  //当前时刻的预测结果  X(k|k-1)
  float X_now;  //当前时刻的最优结果  X(k|k)
  float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
  float P_now;  //当前时刻最优结果的协方差  P(k|k)
  float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
  float kg;     //kalman增益
  float A;      //系统参数
  float B;
  float Q;
  float R;
  float H;
} extKalman_t;


void KalmanCreate(extKalman_t *p, float T_Q, float T_R);
float KalmanFilter(extKalman_t *p, float dat);
/*************一阶卡尔曼 END**************/

/*************二阶卡尔曼**************/
#define mat arm_matrix_instance_f32    //float
#define mat_64 arm_matrix_instance_f64 //double
#define mat_init arm_mat_init_f32
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mult arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32 //浮点矩阵转置
#define mat_inv arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

#define Angle_limit 200         //角度小于50开启预测
#define PredictAngle_limit 250 //预测值限幅

#define Kf_Angle 0
#define Kf_Speed 1

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;

typedef struct
{
  float Vision_Angle; //视觉--角度
  float Vision_Speed; //视觉--速度
  float *Kf_result;   //卡尔曼输出值
  uint16_t Kf_Delay;  //卡尔曼延时计时

  struct
  {
    float Predicted_Factor;   //预测比例因子
    float Predicted_SpeedMin; //预测值最小速度
    float Predicted_SpeedMax; //预测值最大速度
    float kf_delay_open;      //卡尔曼延时开启时间
  } Parameter;
} Kalman_Data_t;
/*******************************************************************************/


/**
  ******************************************************************************
  * @file    kalman filter.h
  * @author  Hongxi Wang
  * @version V1.1.5
  * @date    2020/2/16
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */

//cortex-m4 DSP lib
#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#ifndef __CC_ARM
#define __CC_ARM
#endif
#ifndef ARM_MATH_MATRIX_CHECK
#define ARM_MATH_MATRIX_CHECK
#endif
#ifndef ARM_MATH_ROUNDING
#define ARM_MATH_ROUNDING
#endif


#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

typedef struct kf_t
{
    float *FilteredValue;
    float *MeasuredVector;
    float *ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;      // 量测与状态的关系 how measurement relates to the state
    float *MeasurementDegree;     // 测量值对应H矩阵元素值 elements of each measurement in H
    float *MatR_DiagonalElements; // 量测方差 variance for each measurement
    float *StateMinVariance;      // 最小方差 避免方差过度收敛 suppress filter excessive convergence
    uint8_t *temp;
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    mat xhat;      // x(k|k)
    mat xhatminus; // x(k|k-1)
    mat u;         // control vector u
    mat z;         // measurement vector z
    mat P;         // covariance matrix P(k|k)
    mat Pminus;    // covariance matrix P(k|k-1)
    mat F, FT;     // state transition matrix F FT
    mat B;         // control matrix B
    mat H, HT;     // measurement matrix H
    mat Q;         // process noise covariance matrix Q
    mat R;         // measurement noise covariance matrix R
    mat K;         // kalman gain  K
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);

    float *xhat_data, *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
} KalmanFilter_t;
extern uint16_t sizeof_float, sizeof_double;
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
float *Kalman_Filter_Update(KalmanFilter_t *kf);



void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);
/*************二阶卡尔曼 END**************/

#endif
