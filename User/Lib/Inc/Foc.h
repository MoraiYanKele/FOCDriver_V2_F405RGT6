#ifndef __FOC_HPP__
#define __FOC_HPP__

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/
#ifdef __cplusplus

#include "common_inc.h"

#include "LowPassFilter.h"

/**
 * @brief  将一个值限制在指定的最小和最大边界之间。
 * @tparam T 数据类型 (例如 float, int)。
 * @param  val 要限制的值。
 * @param  min_val 允许的最小值。
 * @param  max_val 允许的最大值。
 * @retval 限制后的值。如果 val 在 [min_val, max_val] 范围内，则返回 val 本身。
 *         如果 val 小于 min_val，则返回 min_val。
 *         如果 val 大于 max_val，则返回 max_val。
 */
template <typename T>
T constrain(T val, T min_val, T max_val) 
{
  return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

typedef struct 
{
  float alpha;     // 滤波器系数，范围在 0 到 1 之间，值越小，平滑效果越强
  float prevValue; // 上一个滤波后的输出值
} LowPassFilterTypedef;

class FOC
{
private:
 
  // 配置参数
  constexpr static float VOLTAGE_POWER_SUPPLY = 12.6f;  // 电源电压
  constexpr static float VOLTAGE_LIMIT = 10.0f;        // 电压限制
  constexpr static uint32_t PWM_MAX_VALUE = 4200;     // PWM最大值


#ifdef YAW_MOTOR
  constexpr static int POLE_PAIRS = 11;                // 极对数(yaw轴电机)
  // 默认PID参数（yaw轴电机）
  constexpr static float DEFAULT_POSITION_KP = 11.7f; // 默认位置PID参数
  constexpr static float DEFAULT_POSITION_KI = 0.62f;
  constexpr static float DEFAULT_POSITION_KD = 0.14f;

  constexpr static float DEFAULT_SPEED_KP = 0.78f; // 默认速度PID参数
  constexpr static float DEFAULT_SPEED_KI = 0.69f;
  constexpr static float DEFAULT_SPEED_KD = 0.0f;

  constexpr static float DEFAULT_CURRENT_KP = 0.0f; // 默认电流PID参数
  constexpr static float DEFAULT_CURRENT_KI = 0.0f;
  constexpr static float DEFAULT_CURRENT_KD = 0.0f;

#else 

  constexpr static int POLE_PAIRS = 7;                // 极对数（pitch轴电机）
  // 默认PID参数（pitch轴电机）
  constexpr static float DEFAULT_POSITION_KP = 14.05f; // 默认位置PID参数
  constexpr static float DEFAULT_POSITION_KI = 0.61f;
  constexpr static float DEFAULT_POSITION_KD = 0.11001f;

  constexpr static float DEFAULT_SPEED_KP = 0.15f; // 默认速度PID参数
  constexpr static float DEFAULT_SPEED_KI = 0.24f;
  constexpr static float DEFAULT_SPEED_KD = 0.0f;

  constexpr static float DEFAULT_CURRENT_KP = 0.0f; // 默认电流PID参数
  constexpr static float DEFAULT_CURRENT_KI = 0.0f;
  constexpr static float DEFAULT_CURRENT_KD = 0.0f;

#endif

  constexpr static float DEFAULT_PID_SAMPLE_TIME = 0.001f; // 默认PID采样时间

  // 以C相为基准的增益校正系数
  constexpr static float GAIN_A_CORRECTION = 1.0f;
  constexpr static float GAIN_B_CORRECTION = 1.3f;
  constexpr static float GAIN_C_CORRECTION = 1.0f;


  float uAlpha = 0.0f, uBeta = 0.0f; // αβ 坐标系
  float currentA, currentB, currentC; // 三相电流
  float Iq, Id;

  LPS speedFiliter{1000.0f, 4.0f};

  TIM_HandleTypeDef* pwmTimer = nullptr;  // PWM定时器句柄
  bool isInitialized = false;             // 初始化标志


 
  void SetPwm();

public:

  PIDControllerTypedef speedPidController;
  PIDControllerTypedef positionPidController;
  PIDControllerTypedef currentPidController;
 
#ifdef YAW_MOTOR

  float zeroElectricAngle = 4.62f; // 零电角度
#else
  float zeroElectricAngle = 4.5f; // 零电角度
#endif
  float voltageA = 0.0f, voltageB = 0.0f, voltageC = 0.0f; // 三相电压

  FOC();
  ~FOC();
  
  // 改进的初始化函数，支持硬件抽象
  bool Init(TIM_HandleTypeDef* timer = &htim1);
  bool IsInitialized() const { return isInitialized; }
  
  // 参数配置接口
  void SetMotorParameters(int polePairs, float voltageLimit, float powerSupply = VOLTAGE_POWER_SUPPLY);
  void SetPIDParameters(float kp_speed, float ki_speed, float kd_speed,
                        float kp_pos, float ki_pos, float kd_pos,
                        float kp_current, float ki_current, float kd_current);
  void Clarke(float electricalAngle);
  float CalibrateZeroElectricAngle();
  void CalibrationCurrentOffset();
  float GetMechanicalAngle();
  float NormalizeAngle(float angle);
  void SpeedControl(float targetSpeed);
  void PositionControl(float targetPosition);
  float GetElectricAngle(float shaftAngle);
  void SetPhaseVoltage(float uq, float ud, float electricalAngle);
  float GetVelocity(float angle);
  float LowPassFilterUpdate(LowPassFilterTypedef *filter, float input);

  // 安全检查和错误处理
  void EmergencyStop();
};


#endif // __cplusplus
#endif