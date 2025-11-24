#ifndef __MOTOR_TASK_H__
#define __MOTOR_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif





#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "common_inc.h" // 包含必要的头文件
#include "Foc.h" // 包含 FOC 类定义




extern float targetSpeed; // 目标速度
extern float targetCurrent;
extern float targetPosition;


extern FOC motor; // 创建 FOC 类实例


extern "C" 
{

void MotorTask();

}
#endif // __cplusplus

#endif // !__MOTOR_TASK_H__
