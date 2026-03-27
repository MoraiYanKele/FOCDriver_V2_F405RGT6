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

class PidController {
private:
    PIDControllerTypedef pid_;
public:
    PidController() = default;

    void Init(float kp, float ki, float kd, float sampleTime,
              float outputLimits, float integralLimits, float deadBand,
              PIDModeTypedef mode) {
        PID_Init(&pid_, kp, ki, kd, sampleTime, outputLimits, integralLimits, deadBand, mode);
    }

    float Compute(float current, float target) { return PIDCompute(&pid_, current, target); }
    void SetOutputLimits(float min, float max) { PID_SetOutputLimits(&pid_, min, max); }
    void Reset() { PID_Reset(&pid_); }

    float* KpPtr() { return &pid_.Kp; }
    float* KiPtr() { return &pid_.Ki; }
    float* KdPtr() { return &pid_.Kd; }

    float Integral() const { return pid_.integral; }
    void ClearIntegral() { pid_.integral = 0.0f; }
};

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

enum class MotorMode : uint8_t
{ 
  INIT = 1,
  NONE = 2,
  RUNNING = 3
};

enum class ControlMode : uint8_t
{
  NONE = 0,
  TORQUE = 1,        // 力矩控制模式
  VELOCITY = 2,      // 速度控制模式
  POSITION = 3,      // 位置控制模式
  MIT = 4,           // MIT 协议控制模式
};


#if (MOTOR_4621)
constexpr static float TORQUE_CONST             = 0.14f;                          // 力矩常数 (Nm/A)
constexpr static float TORQUE_LIMIT             = 0.186f;                         // 力矩限制 (Nm)
constexpr static float SPEED_LIMIT_RPM          = 856.0f;                         // 速度限制 (RPM)
constexpr static int POLE_PAIRS                 = 11;                             // 极对数

constexpr static float DEFAULT_POSITION_KP = 9.0f;                                  // 默认位置PID参数
constexpr static float DEFAULT_POSITION_KI = 0.0f;
constexpr static float DEFAULT_POSITION_KD = 0.0f;

constexpr static float DEFAULT_SPEED_KP = 0.008f;                                   // 默认速度PID参数
constexpr static float DEFAULT_SPEED_KI = 3.2f;
constexpr static float DEFAULT_SPEED_KD = 0.0f;

constexpr static float DEFAULT_CURRENT_KP = 10.5f;                                  // 默认电流PID参数
constexpr static float DEFAULT_CURRENT_KI = 42100.0f;
constexpr static float DEFAULT_CURRENT_KD = 0.0f;

#elif (MOTOR_2804)

constexpr static float TORQUE_CONST             = 0.06f;                          // 力矩常数 (Nm/A)
constexpr static float TORQUE_LIMIT             = 0.14f;                          // 力矩限制 (Nm)
constexpr static float SPEED_LIMIT_RPM          = 2200.0f;                        // 速度限制 (RPM)
constexpr static int POLE_PAIRS                 = 7;                              // 极对数


constexpr static float DEFAULT_POSITION_KP = 9.7f;                                  // 默认位置PID参数
constexpr static float DEFAULT_POSITION_KI = 0.0f;
constexpr static float DEFAULT_POSITION_KD = 0.0f;

constexpr static float DEFAULT_SPEED_KP = 0.01f;                                    // 默认速度PID参数
constexpr static float DEFAULT_SPEED_KI = 1.2f;
constexpr static float DEFAULT_SPEED_KD = 0.0f;

constexpr static float DEFAULT_CURRENT_KP = 1.6f;                                   // 默认电流PID参数
constexpr static float DEFAULT_CURRENT_KI = 8518.0f;
constexpr static float DEFAULT_CURRENT_KD = 0.0f;


#endif // DEBUG


constexpr static float VOLTAGE_POWER_SUPPLY     = 12.6f;                          // 电源电压
constexpr static float N_BASE                   = 3.0f;                           // 3S电池
constexpr static float BATVEL                   = (4.0f * N_BASE);                // 电池电压
constexpr static float INVBATVEL                = (1.0f / BATVEL);                // 电池电压的倒数
constexpr static float VOLTAGE_LIMIT            = BATVEL / _SQRT3;                // 电压限制
constexpr static float CURRENT_LIMIT            = (TORQUE_LIMIT / TORQUE_CONST);  // 电流限制 (A)
constexpr static float SPEED_LIMIT              = (SPEED_LIMIT_RPM * _PI / 30.0f);// 速度限制 (rad/s)
constexpr static uint32_t PWM_MAX_VALUE         = 4200;                           // PWM最大值
constexpr static float TS                       = 1.0f;                           // 控制周期（归一化到1.0）
constexpr static float FAC_CURRENT_ADC          = (3.3f / 4096.0f) * 4;           // 电流ADC转换系数
constexpr static float FAC_VOLTAGE_ADC          = (3.3f / 4096.0f) * 11;          // 电压ADC转换系数
constexpr static float CURRENT_MEASURE_PERIOD   = 0.00005f;                        // 电流测量周期 (秒)
constexpr static float VELOCITY_MEASURE_PERIOD  = 0.001f;                         // 速度测量周期 (秒)
constexpr static float MOTOR_CURRENT_RAMP_RATE  = 0.5f;                           // 电流爬升力矩 [Nm/s]

constexpr static float CURRENT_PID_MAX_OUT = VOLTAGE_LIMIT;                     // 电流PID最大输出
constexpr static float SPEED_PID_MAX_OUT = CURRENT_LIMIT;                       // 速度PID最大输出
constexpr static float POSITION_PID_MAX_OUT = SPEED_LIMIT;                      // 位置PID最大输出

class FOC
{
private:
 




  float uAlpha = 0.0f, uBeta = 0.0f;        // αβ 坐标系
  float currentAOffset = 0.0f, currentBOffset = 0.0f, currentCOffset = 0.0f;  // 三相电流偏置

  float Vbus = 0.0f;                                                          // 总线电压

  

  TIM_HandleTypeDef* pwmTimer = nullptr;    // PWM定时器句柄

  LPS speedFiliter{1000.0f, 100.0f};         // 速度低通滤波器

  LPS IqFilter{20000.0f, 40.0f};             // q轴电流低通滤波器
  LPS IdFilter{20000.0f, 40.0f};             // d轴电流低通滤波器

  LPS IaFilter{20000.0f, 1000.0f};          // A相电流低通滤波器
  LPS IbFilter{20000.0f, 1000.0f};          // B相电流低通滤波器
  LPS IcFilter{20000.0f, 1000.0f};          // C相电流低通滤波器


  void SetPwm();

public:

  PidController speedPid;
  PidController positionPid;
  PidController currentPid;
 
#ifdef YAW_MOTOR

  float zeroElectricAngle = 1.5f; // 零电角度
#else
  float zeroElectricAngle = 0.0f;
#endif

  float Ia = 0.0f, Ib = 0.0f, Ic = 0.0f;                                      // 三相电流
  float Iq = 0.0f, Id = 0.0f;                                                 // dq 坐标系
  float voltageA = 0.0f, voltageB = 0.0f, voltageC = 0.0f; // 三相电压
  float voltageA_Duty = 0.0f, voltageB_Duty = 0.0f, voltageC_Duty = 0.0f;

  float mitKp = 0.0f;       // MIT 协议位置控制比例系数
  float mitKd = 0.0f;       // MIT 协议位置控制微分系数

  float angleSingleTurn = 0.0f;                                               // 单圈角度
  float motorAngle = 0.0f;                                                      // 当前机械角度
  float velocity = 0.0f;                                                      // 当前速度

  MotorMode motorMode = MotorMode::INIT; // 电机控制模式
  ControlMode controlMode = ControlMode::TORQUE; // 力矩控制模式

  float targetPosition = 0.0f; // 目标位置
  float targetSpeed = 0.0f;    // 目标速度
  float targetCurrent = 0.0f;  // 目标电流
  float targetTorque = 0.0f;   // 目标力矩

  float iqSetpoint = 0.0f;    // q轴电流设定值
  float idSetpoint = 0.0f;    // d轴电流设定值
  float positionSetpoint = 0.0f; // 位置设定值
  float velocitySetpoint = 0.0f; // 速度设定值


  FOC();
  ~FOC();
  
  // 改进的初始化函数，支持硬件抽象
  bool Init(TIM_HandleTypeDef* timer = &htim1);

  
  // 参数配置接口
  void SetMotorParameters(int polePairs, float voltageLimit, float powerSupply = VOLTAGE_POWER_SUPPLY);
  void SetPIDParameters(float kp_speed, float ki_speed, float kd_speed,
                        float kp_pos, float ki_pos, float kd_pos,
                        float kp_current, float ki_current, float kd_current);
  void ClarkeParkTransform(float electricalAngle);
  void CalibrateZeroElectricAngle();
  void GetMechanicalAngle();
  float NormalizeAngle(float angle);
  void SpeedControl(float targetSpeed);
  void TorqueControl(float targetTorque);
  void PositionControl(float targetPosition);
  float GetElectricAngle(float shaftAngle);
  void SetPhaseVoltage(float uq, float ud, float electricalAngle);
  float GetVelocity(float angle);
  float LowPassFilterUpdate(LowPassFilterTypedef *filter, float input);
  void SvpwmSector();
  
  void UpdateCurrent();
  void UpdateCurrentOffsets();
  void UpdateMotorAngle();
  void UpdateVelocity();

  void MotorControlTask();
  // 安全检查和错误处理
  void EmergencyStop();
};


#endif // __cplusplus
#endif