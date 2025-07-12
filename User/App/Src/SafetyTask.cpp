#include "SafetyTask.h"
#include "MotorTask.h"
#include <cmath>      // C++标准数学库（替代math.h）

extern "C" 
{
#include "VOFA.h"
}

TaskHandle_t SafetyTaskHandle = NULL;



bool SafetyLimits::checkSpeed(float speed) 
{
  return std::fabs(speed) <= maxSpeed;  // std::fabs是C++版本的fabs
}

// 成员函数：检查位置是否有效
bool SafetyLimits::checkPosition(float position) 
{
  return (position >= minPosition) && (position <= maxPosition);
}

// 成员函数：更新限制值
void SafetyLimits::updateLimits(float newMaxSpeed, float newMaxPos, float newMinPos) 
{
  maxSpeed = newMaxSpeed;
  maxPosition = newMaxPos;
  minPosition = newMinPos;
}



    
    // 重置所有状态
void SafetyManager::reset() 
{
  currentState = SAFETY_STATE_NORMAL;
  emergencyRequested = false;
  motorShutdown = false;
  faultStartTime = 0;
  shutdownStartTime = 0;
}
    
    // 检查是否安全
bool SafetyManager::isSafe() 
{
  return (currentState == SAFETY_STATE_NORMAL) && (!motorShutdown);
}
    
// 请求紧急停止
void SafetyManager::requestEmergencyStop() 
{
    emergencyRequested = true;
}
    
// 记录故障时间
void SafetyManager::recordFaultTime() 
{
    faultStartTime = HAL_GetTick();
}

// 记录关断时间
void SafetyManager::recordShutdownTime() 
{
    shutdownStartTime = HAL_GetTick();
}

// 获取故障持续时间
uint32_t SafetyManager::getFaultDuration() 
{
    if (faultStartTime == 0) {
        return 0;
    }
    return HAL_GetTick() - faultStartTime;
}

// 获取关断持续时间
uint32_t SafetyManager::getShutdownDuration() 
{
    if (shutdownStartTime == 0) {
        return 0;
    }
    return HAL_GetTick() - shutdownStartTime;
}


// ===== 创建对象实例 =====
SafetyLimits safetyLimits;    // 安全限制对象
SafetyManager safetyManager;  // 安全管理对象

// 外部变量引用
extern FOC motor;
extern ControlModeTypedef motorMode;

// ===== 第4步：简单的枚举替代宏 =====
// 传统C方式：#define FAULT_NONE 0, #define FAULT_SPEED 1
// C++方式：枚举让代码更清晰



void SafetyTask(void *argument)
{
    const int CHECK_PERIOD = 10;  // 检查周期10ms
    vTaskDelay(pdMS_TO_TICKS(1000)); // 等待系统稳定
    
    safetyManager.reset();
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CHECK_PERIOD);

    while (true)
    {  
        // vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // ===== 第6步：用成员函数简化逻辑 =====
        // 检查紧急停止请求
        if (safetyManager.emergencyRequested) 
        {
          handleEmergencyStop();
          continue;
        }
        
        // 检查电机是否关断
        if (safetyManager.motorShutdown) 
        {
            
          handleMotorShutdown();
          continue;
        }

   
        if (motorMode == CONTROL_MODE_POSITION) 
        {
          performSafetyCheck();
        } 
        else 
        {
            // 非位置模式，保持正常状态
          safetyManager.currentState = SAFETY_STATE_NORMAL;
          safetyManager.faultStartTime = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(CHECK_PERIOD));
    }
}

/**
 * @brief 处理紧急停止请求
 */
void handleEmergencyStop()
{
  safetyManager.currentState = SAFETY_STATE_EMERGENCY_STOP;
  motor.EmergencyStop();
  safetyManager.motorShutdown = true;
  safetyManager.recordShutdownTime();
  safetyManager.emergencyRequested = false;
}

/**
 * @brief 处理电机关断状态
 */
void handleMotorShutdown()
{
    uint32_t shutdownTime = safetyManager.getShutdownDuration();
    
    // 在关断期间保持电机停止
    if (shutdownTime < Safety::SHUTDOWN_TIME) 
    {
      motor.EmergencyStop();
      return;
    }
    
    // 进入恢复状态
    safetyManager.currentState = SAFETY_STATE_RECOVERING;
    
    // 检查是否可以重启
    int totalRecoveryTime = Safety::SHUTDOWN_TIME + Safety::RESTART_DELAY;
    if (shutdownTime >= totalRecoveryTime) 
    {
      // 只有在位置模式才重启
      bool canRestart = (motorMode == CONTROL_MODE_POSITION);
      
      if (canRestart) 
      {
        safetyManager.motorShutdown = false;
        safetyManager.currentState = SAFETY_STATE_NORMAL;
        safetyManager.faultStartTime = 0;
      }
    }
}

/**
 * @brief 执行安全检查
 */
void performSafetyCheck()
{
  // 获取当前电机状态
  float currentPosition = motor.GetMechanicalAngle();
  float currentSpeed = motor.GetVelocity(currentPosition);
  
  FaultType faultType = FAULT_NONE;
  
  // 检查速度限制
  if (!safetyLimits.checkSpeed(currentSpeed)) 
  {
    faultType = FAULT_SPEED_LIMIT;
  }

#ifdef YAW_MOTOR
// TODO: 测试，记得删除
  // if (!safetyLimits.checkPosition(currentPosition))
  // {
  //   faultType = FAULT_POSITION_LIMIT;
  // }
#else 
  // 检查位置限制  
  if (!safetyLimits.checkPosition(currentPosition))
  {
    faultType = FAULT_POSITION_LIMIT;
  }
#endif
  // 处理检测到的故障
  handleFaultDetection(faultType);
}

/**
 * @brief 处理故障检测
 * @param faultType 故障类型
 */
void handleFaultDetection(FaultType faultType)
{
  SafetyStateTypedef newState = SAFETY_STATE_NORMAL;
  
  if (faultType == FAULT_SPEED_LIMIT)
  {
    newState = SAFETY_STATE_SPEED_LIMIT;
  } 
    
#ifdef YAW_MOTOR
  // 如果是yaw轴电机，位置限制故障不处理
  // TODO: 测试，记得删除
  // else if (faultType == FAULT_POSITION_LIMIT) 
  // {
  //   newState = SAFETY_STATE_POSITION_LIMIT;
  // }
#else

  else if (faultType == FAULT_POSITION_LIMIT) 
  {
    newState = SAFETY_STATE_POSITION_LIMIT;
  }
#endif    


  if (newState != SAFETY_STATE_NORMAL) 
  {
    // 检测到故障
    if (safetyManager.currentState == SAFETY_STATE_NORMAL) 
    {
      safetyManager.recordFaultTime();
      safetyManager.currentState = newState;

    } 
    else if (safetyManager.currentState == newState) 
    {
      // 故障持续，检查是否需要关闭电机
      if (safetyManager.getFaultDuration() >= Safety::FAULT_CONFIRM_TIME) 
      {
        motor.EmergencyStop();
        safetyManager.motorShutdown = true;
        safetyManager.recordShutdownTime();
      }
    } 
    else 
    {
      // 故障类型改变，重新计时
      safetyManager.recordFaultTime();
      safetyManager.currentState = newState;
    }
  } 
  else 
  {
    safetyManager.currentState = SAFETY_STATE_NORMAL;
    safetyManager.faultStartTime = 0;
  }
}

// ===== 兼容性函数：让C代码可以调用我们的C++类 =====

/**
 * @brief 安全系统初始化（C接口）
 */
void SafetyInit(void)
{
  safetyManager.reset();  // 调用C++对象的方法
}

/**
 * @brief 设置安全限制参数（C接口）
 * @param maxSpeed 最大速度 (rad/s)
 * @param maxPos 最大位置 (rad)
 * @param minPos 最小位置 (rad)
 * @param maxCurrent 最大电流 (A) - 本版本忽略
 * @param maxVoltage 最大电压 (V) - 本版本忽略
 */
void SetSafetyLimits(float maxSpeed, float maxPos, float minPos, float maxCurrent, float maxVoltage)
{
    // 只使用速度和位置参数，调用C++对象的方法
    safetyLimits.updateLimits(maxSpeed, maxPos, minPos);
}

/**
 * @brief 请求紧急停止（C接口）
 */
void RequestEmergencyStop(void)
{
  safetyManager.requestEmergencyStop();  // 调用C++对象的方法
}

/**
 * @brief 检查安全状态是否正常（C接口）
 * @return true: 安全正常, false: 存在安全问题
 */
bool IsSafetyOK(void)
{
  return safetyManager.isSafe();  // 调用C++对象的方法
}

/**
 * @brief 获取当前安全状态（C接口）
 * @return 安全状态枚举值
 */
SafetyStateTypedef GetSafetyState(void)
{
  return safetyManager.currentState;  // 直接返回C++对象的成员变量
}

/**
 * @brief 清除安全错误（C接口）
 */
void ClearSafetyError(void)
{
  if (safetyManager.motorShutdown) 
  {
    safetyManager.reset();  // 调用C++对象的方法
  }
}
