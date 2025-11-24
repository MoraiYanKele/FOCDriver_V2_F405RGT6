#ifndef __SAFETY_TASK_H__
#define __SAFETY_TASK_H__

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "common_inc.h"

extern TaskHandle_t SafetyTaskHandle; // 安全任务句柄

#ifdef __cplusplus
}
#endif

/*---------------------------- C++ Scope ---------------------------*/
#ifdef __cplusplus

#include "Foc.h"


typedef struct 
{
  float maxSpeed;          // 最大速度限制 (rad/s)
  float maxPosition;       // 最大位置限制 (rad)
  float minPosition;       // 最小位置限制 (rad)
  uint32_t shutdownTime;   // 故障关断时间 (ms)
  uint32_t restartDelay;   // 重启延时 (ms)
} SafetyParamsTypedef;


typedef enum 
{
  SAFETY_STATE_NORMAL = 0,     // 正常状态
  SAFETY_STATE_SPEED_LIMIT,    // 速度超限
  SAFETY_STATE_POSITION_LIMIT, // 位置超限
  SAFETY_STATE_EMERGENCY_STOP, // 紧急停止
  SAFETY_STATE_RECOVERING,     // 恢复中
} SafetyStateTypedef;

enum FaultType 
{
  FAULT_NONE = 0,
  FAULT_SPEED_LIMIT,
  FAULT_POSITION_LIMIT
};

// 外部变量声明（简化版本）
extern SafetyParamsTypedef safetyParams;
extern SafetyStateTypedef safetyState;
extern volatile bool emergencyStopRequested;

namespace Safety 
{
  const float MAX_SPEED = 80.0f;           // 最大速度 30 rad/s
  const float MAX_POSITION = 3.14f;     // 最大位置 +π rad  
  const float MIN_POSITION = -3.14f;    // 最小位置 -π rad
  const int SHUTDOWN_TIME = 500;           // 关断时间 500ms
  const int RESTART_DELAY = 1000;          // 重启延时 1000ms
  const int FAULT_CONFIRM_TIME = 100;      // 故障确认时间 100ms
}


class SafetyLimits 
{
public:  // public表示外部可以访问
  // 成员变量（就像C结构体的成员）
  float maxSpeed;
  float maxPosition;
  float minPosition;
  
  // 构造函数：创建对象时自动调用
  SafetyLimits() 
  {
    maxSpeed = Safety::MAX_SPEED;
    maxPosition = Safety::MAX_POSITION;
    minPosition = Safety::MIN_POSITION;
  }
  
  // 成员函数：检查速度是否有效
  bool checkSpeed(float speed);
  
  // 成员函数：检查位置是否有效
  bool checkPosition(float position);
  
  // 成员函数：更新限制值
  void updateLimits(float newMaxSpeed, float newMaxPos, float newMinPos);
};

class SafetyManager 
{
public:
    // 状态变量
    SafetyStateTypedef currentState;
    bool emergencyRequested;
    bool motorShutdown;
    uint32_t faultStartTime;
    uint32_t shutdownStartTime;
    
    // 构造函数：初始化所有变量
    SafetyManager() {
        reset();
    }
    
    // 重置所有状态
    void reset();
    
    // 检查是否安全
    bool isSafe();
    
    // 请求紧急停止
    void requestEmergencyStop();
    
    // 记录故障时间
    void recordFaultTime();
    
    // 记录关断时间
    void recordShutdownTime();
    
    // 获取故障持续时间
    uint32_t getFaultDuration();
    
    // 获取关断持续时间
    uint32_t getShutdownDuration();
};

// 函数声明（简化接口，移除电流电压参数）
void SafetyTask(void *argument);
void SafetyInit(void);
void SetSafetyLimits(float maxSpeed, float maxPos, float minPos, float maxCurrent, float maxVoltage);
void RequestEmergencyStop(void);
bool IsSafetyOK(void);
SafetyStateTypedef GetSafetyState(void);
void ClearSafetyError(void);
void handleEmergencyStop(void);
void handleMotorShutdown(void);
void performSafetyCheck(void);
void handleFaultDetection(FaultType faultType);


#endif // __cplusplus

#endif // __SAFETY_TASK_H__
