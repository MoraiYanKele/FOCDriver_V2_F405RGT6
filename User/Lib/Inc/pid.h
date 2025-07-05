#ifndef __PID_H__
#define __PID_H__

#include "main.h"

// PID控制器模式定义

#define PID_DEFAULT_OUTPUT_MAX              1000.0f  // 默认PID输出最大值
#define PID_DEFAULT_OUTPUT_MIN              -1000.0f // 默认PID输出最小值
#define PID_DEFAULT_INTEGRAL_MAX            500.0f   // 默认PID积分项最大值
#define PID_DEFAULT_INTEGRAL_MIN            -500.0f  // 默认PID积分项最小值
#define PID_DEFAULT_DEADBAND                0.0f     // 默认PID死区 (误差绝对值小于此值时不响应)
#define PID_DEFAULT_INTEGRAL_SEP_THRESH     200.0f   // 默认积分分离阈值 (误差绝对值大于此值时不累积积分)
#define PID_DEFAULT_DERIV_FILTER_FACTOR     1.0f     // 默认微分项一阶低通滤波系数 (0-1, 越小滤波越强)
#define PID_DEFAULT_MODE                    PID_MODE_POSITION // 默认PID工作模式 (位置式)
#define PID_DEFAULT_SAMPLE_TIME             0.01f    // 默认采样时间 (秒)
#define PID_DEFAULT_INV_SAMPLE_TIME         (1.0f / PID_DEFAULT_SAMPLE_TIME) // 默认采样时间的倒数 (用于优化计算)


typedef enum 
{
    PID_MODE_POSITION = 0, // 位置式PID
    PID_MODE_DELTA    = 1  // 增量式PID
} PIDModeTypedef;


typedef struct 
{
    // PID参数
    float Kp;            // 比例系数
    float Ki;            // 积分系数
    float Kd;            // 微分系数
    
    // 控制目标与状态
    float targetValue;   // 目标值
    float error;         // 当前误差
    float previous_error; // 上一次误差
    float prev_prev_error; // 上上次误差(用于增量式PID)
    float integral;      // 积分项
    float derivative;    // 微分项
    
    // 输出限制
    float outputMax;     // 输出上限
    float outputMin;     // 输出下限
    
    // 积分限制
    float integralMax;   // 积分上限
    float integralMin;   // 积分下限
    
    // 死区设置
    float deadBand;      // 误差死区，小于该值不进行控制
    
    // 积分分离阈值
    float integralSeparationThreshold; // 误差大于该阈值时，不计算积分项
    
    // 微分滤波系数 (0-1), 1表示不滤波
    float derivativeFilterFactor;
    
    // 控制器工作模式
    PIDModeTypedef mode; // 0: 位置式, 1: 增量式
    
    // 滤波后的微分项(仅内部使用)
    float filteredDerivative;

    float sampleTime;        // 采样时间，单位为秒
    float invSampleTime;     // 采样时间的倒数，用于优化计算

    float previous_measurement; // 添加这个新成员来存储上一次的测量值
    
} PIDControllerTypedef;
  
// 函数声明

void PID_Init(PIDControllerTypedef *pid, 
    float Kp, float Ki, float Kd, 
    float sampleTime, 
    float outputlimits, float integralLimits, 
    float deadBand,
    PIDModeTypedef mode);
float PIDCompute(PIDControllerTypedef *pid, float now_Value, float target_Value);
void PID_SetOutputLimits(PIDControllerTypedef *pid, float min, float max);
void PID_SetIntegralLimits(PIDControllerTypedef *pid, float min, float max);
void PID_SetDeadBand(PIDControllerTypedef *pid, float deadBand);
void PID_SetIntegralSeparationThreshold(PIDControllerTypedef *pid, float threshold);
void PID_SetDerivativeFilterFactor(PIDControllerTypedef *pid, float factor);
void PID_SetMode(PIDControllerTypedef *pid, uint8_t mode);
void PID_Reset(PIDControllerTypedef *pid);
void PID_SetSampleTime(PIDControllerTypedef *pid, float sampleTime);


#endif