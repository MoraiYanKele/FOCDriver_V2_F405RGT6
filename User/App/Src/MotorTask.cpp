#include "MotorTask.h"
#include "Foc.h" 
#include "pid.h" 

FOC motor; // 创建 FOC 类实例
ControlModeTypedef motorMode = NONE; // 控制模式

TaskHandle_t MotorTaskHandle = NULL; // 电机任务句柄


float targetSpeed = 0.0f;
float targetCurrent = 0.0f;
float targetPosition = 0.0f;

void MotorTask(void *argument)
{
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  motor.Init();
  motor.CalibrateZeroElectricAngle();
  vTaskDelay(pdMS_TO_TICKS(500));

  while (1)
  {
    switch (motorMode)
    {
    case  NONE:
      
      break;
    
    case CONTROL_MODE_SPEED:
      motor.SpeedControl(targetSpeed); // 调用 FOC 类的速度控制方法
      break;

    case CONTROL_MODE_POSITION:
      motor.PositionControl(targetPosition); // 调用 FOC 类的 位置控制方法
      break; 

    default:
      break;
    }
    Delay(1); // 延时1毫秒
  }

}