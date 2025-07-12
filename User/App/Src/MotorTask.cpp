#include "MotorTask.h"
#include "SafetyTask.h"
#include "Foc.h" 

extern "C" {
#include "pid.h" 
#include "VOFA.h"
}

FOC motor; // 创建 FOC 类实例
ControlModeTypedef motorMode = CONTROL_MODE_NONE; // 控制模式

TaskHandle_t MotorTaskHandle = NULL; // 电机任务句柄

extern SafetyLimits safetyLimits; // 安全限制对象

float targetSpeed = 0.0f;
float targetCurrent = 0.0f;
float targetPosition = 0.0f;

void MotorTask(void *argument)
{
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  motor.Init();
  // TODO 测试
  // motor.CalibrateZeroElectricAngle();
  // motor.zeroElectricAngle = 1.75f;
  motorMode = CONTROL_MODE_POSITION;

  // targetSpeed = 20.0f; // 设置目标速度

  // targetPosition = -0.2f;
#ifdef YAW_MOTOR

#else 
  safetyLimits.updateLimits(Safety::MAX_SPEED, 0.5f, -1.0f); // 更新安全限制
#endif
  vTaskDelay(pdMS_TO_TICKS(500));

  TickType_t xLastWakeTime = xTaskGetTickCount(); // 初始化唤醒时间
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms周期

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    switch (motorMode)
    {
    case CONTROL_MODE_NONE: 
      motor.SetPhaseVoltage(0.0f, 0.0f, 0.0f); // 停止电机
      break;
    
    case CONTROL_MODE_SPEED:
      // 检查安全状态
      if (IsSafetyOK())
      {
        motor.SpeedControl(targetSpeed); // 调用 FOC 类的速度控制方法
      }
      else
      {
        motor.SetPhaseVoltage(0.0f, 0.0f, 0.0f); // 安全保护时停止电机
      }
      break;

    case CONTROL_MODE_POSITION:
      // 检查安全状态
      if (IsSafetyOK())
      {
        motor.PositionControl(targetPosition); // 调用 FOC 类的位置控制方法
      }
      else
      {
        motor.SetPhaseVoltage(0.0f, 0.0f, 0.0f); // 安全保护时停止电机
      }
      break; 

    default:
      break;
    }
  }

}
