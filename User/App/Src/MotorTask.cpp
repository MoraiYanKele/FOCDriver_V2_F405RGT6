#include "MotorTask.h"
#include "SafetyTask.h"
#include "Foc.h" 
#include "usermain.h"

extern "C" {
#include "pid.h" 
#include "VOFA.h"
}

FOC motor; // 创建 FOC 类实例

float currentAOffset = 0.0f; // A相电流偏置
float currentBOffset = 0.0f; // B相电流偏置
float currentCOffset = 0.0f; // C相电流偏置

void MotorTask()
{
  static uint8_t loopCount = 0;
  motor.UpdateCurrent();
  motor.UpdateMotorAngle();
  loopCount++;
  if (loopCount >= 20) // 20kHz电流环 1Khz速度位置环
  {
    loopCount = 0;
    motor.UpdateVelocity();
  }
  switch (motor.motorMode)
  {
  case MotorMode::INIT:
    
    motor.motorMode = MotorMode::NONE; // 切换到无控制模式
    
    break;

  case MotorMode::NONE:

    motor.motorMode = MotorMode::RUNNING; // 切换到运行模式
    motor.targetSpeed = 0.0f;
    motor.velocitySetpoint = 0.0f;
    motor.controlMode = ControlMode::INERTIA;
    
    break;

  case MotorMode::RUNNING:
    motor.MotorControlTask(); // 电机控制任务
    break;

  default:
    break;
  }
}

