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
  // Internal electrical state
  float uAlpha_ = 0.0f, uBeta_ = 0.0f;
  float currentAOffset_ = 0.0f, currentBOffset_ = 0.0f, currentCOffset_ = 0.0f;
  float Vbus_ = 0.0f;

  // Measured currents
  float Ia_ = 0.0f, Ib_ = 0.0f, Ic_ = 0.0f;
  float Iq_ = 0.0f, Id_ = 0.0f;

  // Phase voltages (internal)
  float voltageA_ = 0.0f, voltageB_ = 0.0f, voltageC_ = 0.0f;
  float voltageA_Duty_ = 0.0f, voltageB_Duty_ = 0.0f, voltageC_Duty_ = 0.0f;

  // Sensor state
  float angleSingleTurn_ = 0.0f;
  float motorAngle_ = 0.0f;
  float velocity_ = 0.0f;

  // Control targets
  float targetPosition_ = 0.0f;
  float targetSpeed_ = 0.0f;
  float targetCurrent_ = 0.0f;
  float targetTorque_ = 0.0f;

  // MIT params
  float mitKp_ = 0.0f;
  float mitKd_ = 0.0f;

  // Internal setpoints
  float iqSetpoint_ = 0.0f;
  float idSetpoint_ = 0.0f;
  float positionSetpoint_ = 0.0f;
  float velocitySetpoint_ = 0.0f;

  // State machine
  MotorMode motorMode_ = MotorMode::INIT;
  ControlMode controlMode_ = ControlMode::TORQUE;

  // Hardware
  TIM_HandleTypeDef* pwmTimer_ = nullptr;

  // Filters
  LPS speedFilter_{1000.0f, 100.0f};
  LPS IqFilter_{20000.0f, 40.0f};
  LPS IdFilter_{20000.0f, 40.0f};
  LPS IaFilter_{20000.0f, 1000.0f};
  LPS IbFilter_{20000.0f, 1000.0f};
  LPS IcFilter_{20000.0f, 1000.0f};

  // PID controllers
  PidController speedPid_;
  PidController positionPid_;
  PidController currentPid_;

  // Loop counter for velocity decimation
  uint8_t loopCount_ = 0;

#ifdef YAW_MOTOR
  float zeroElectricAngle_ = 1.5f;
#else
  float zeroElectricAngle_ = 0.0f;
#endif

  // Private methods
  void SetPwm();
  void SvpwmSector();
  void ClarkeParkTransform(float electricalAngle);
  float NormalizeAngle(float angle);
  float GetElectricAngle(float shaftAngle);
  void SetPhaseVoltage(float uq, float ud, float electricalAngle);
  float GetVelocity(float angle);
  float LowPassFilterUpdate(LowPassFilterTypedef *filter, float input);
  void UpdateCurrent();
  void UpdateCurrentOffsets();
  void UpdateMotorAngle();
  void UpdateVelocity();
  void MotorControlTask();

public:
  FOC();
  ~FOC();

  // Lifecycle
  bool Init(TIM_HandleTypeDef* timer = &htim1);
  void Tick();
  void CalibrateZeroElectricAngle();
  void EmergencyStop();

  // Target-setting API (sets mode implicitly)
  void SetTorqueTarget(float current);
  void SetVelocityTarget(float speed);
  void SetPositionTarget(float position);
  void SetMitTarget(float pos, float vel, float torque, float kp, float kd);
  void Stop();

  // Read-only getters
  float Ia() const { return Ia_; }
  float Ib() const { return Ib_; }
  float Ic() const { return Ic_; }
  float Iq() const { return Iq_; }
  float Id() const { return Id_; }
  float MotorAngle() const { return motorAngle_; }
  float AngleSingleTurn() const { return angleSingleTurn_; }
  float Velocity() const { return velocity_; }
  float TargetPosition() const { return targetPosition_; }
  float TargetSpeed() const { return targetSpeed_; }
  float TargetCurrent() const { return targetCurrent_; }
  float TargetTorque() const { return targetTorque_; }
  ControlMode GetControlMode() const { return controlMode_; }
  MotorMode GetMotorMode() const { return motorMode_; }

  // VOFA pointer access (for online tuning)
  float* TargetPositionPtr() { return &targetPosition_; }
  float* TargetSpeedPtr() { return &targetSpeed_; }
  float* TargetCurrentPtr() { return &targetCurrent_; }
  float* TargetTorquePtr() { return &targetTorque_; }

  // PID access (for VOFA tuning)
  PidController& SpeedPid() { return speedPid_; }
  PidController& PositionPid() { return positionPid_; }
  PidController& CurrentPid() { return currentPid_; }

  // Legacy API
  void SetPIDParameters(float kp_speed, float ki_speed, float kd_speed,
                        float kp_pos, float ki_pos, float kd_pos,
                        float kp_current, float ki_current, float kd_current);
  void SpeedControl(float targetSpeed);
  void TorqueControl(float targetTorque);
  void PositionControl(float targetPosition);
  void GetMechanicalAngle();
};


#endif // __cplusplus
#endif
