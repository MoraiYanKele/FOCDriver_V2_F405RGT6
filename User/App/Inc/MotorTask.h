#ifndef __MOTOR_TASK_H__
#define __MOTOR_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif





#ifdef __cplusplus
}
#endif



#include "common_inc.h" // 包含必要的头文件
#include "Foc.h" // 包含 FOC 类定义

typedef enum
{
  NONE                  = 0,        // 无控制模式
  CONTROL_MODE_SPEED    = 1,        // 速度控制模式
  CONTROL_MODE_POSITION = 2,        // 位置控制模式
} ControlModeTypedef;

extern float targetSpeed; // 目标速度
extern float targetCurrent;
extern float targetPosition;

extern ControlModeTypedef motorMode; // 电机控制模式
extern TaskHandle_t MotorTaskHandle; // 电机任务句柄
extern FOC motor; // 创建 FOC 类实例

extern "C" {

void MotorTask(void *argument);

}

#endif // !__MOTOR_TASK_H__
