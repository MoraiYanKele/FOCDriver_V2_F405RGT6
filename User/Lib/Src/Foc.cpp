#include "Foc.h"
#include "LowPassFilter.h"  // 在实现文件中包含完整定义

extern "C"
{
  #include "VOFA.h"
  #include "pid.h"
  #include "MT6701.h"
}

enum class  SectorType : uint8_t
{
  Sector_1 = 3,
  Sector_2 = 1,
  Sector_3 = 5,
  Sector_4 = 4,
  Sector_5 = 6,
  Sector_6 = 2
};


FOC::FOC()
{
}

FOC::~FOC()
{
}

bool FOC::Init(TIM_HandleTypeDef* timer)
{
  if (timer == nullptr)
    return false;

  pwmTimer_ = timer;

  MT6701_DMA_Init(); // 初始化 SPI DMA 预取，供校准和运行时使用

  // 初始化电机控制相关参数
  voltageA_ = 0.0f;
  voltageB_ = 0.0f;
  voltageC_ = 0.0f;
  Ia_ = 0.0f;
  Ib_ = 0.0f;
  Ic_ = 0.0f;

  // 启动PWM通道
  bool initSuccess = true;
  if (HAL_TIM_PWM_Start(pwmTimer_, TIM_CHANNEL_1) != HAL_OK) initSuccess = false;
  if (HAL_TIM_PWM_Start(pwmTimer_, TIM_CHANNEL_2) != HAL_OK) initSuccess = false;
  if (HAL_TIM_PWM_Start(pwmTimer_, TIM_CHANNEL_3) != HAL_OK) initSuccess = false;
  if (HAL_TIM_PWM_Start(pwmTimer_, TIM_CHANNEL_4) != HAL_OK) initSuccess = false;
  if (HAL_TIMEx_PWMN_Start(pwmTimer_, TIM_CHANNEL_1) != HAL_OK) initSuccess = false;
  if (HAL_TIMEx_PWMN_Start(pwmTimer_, TIM_CHANNEL_2) != HAL_OK) initSuccess = false;
  if (HAL_TIMEx_PWMN_Start(pwmTimer_, TIM_CHANNEL_3) != HAL_OK) initSuccess = false;

  if (!initSuccess)
  {
    return false;
  }

  // 对于高级定时器(如TIM1/TIM8)，必须使能主输出，否则PWM不会有任何输出
  __HAL_TIM_MOE_ENABLE(pwmTimer_);

  // 初始化DWT计时器
  DWT_Init(168);

  // 初始化PID控制器
  positionPid_.Init(
      DEFAULT_POSITION_KP, DEFAULT_POSITION_KI, DEFAULT_POSITION_KD,
      0.001f, POSITION_PID_MAX_OUT * 0.75f, POSITION_PID_MAX_OUT, 0.0f,
      PID_MODE_POSITION);

  speedPid_.Init(
      DEFAULT_SPEED_KP, DEFAULT_SPEED_KI, DEFAULT_SPEED_KD,
      0.001f, SPEED_PID_MAX_OUT, SPEED_PID_MAX_OUT * 0.8f, 0.01f,
      PID_MODE_POSITION);

  currentPid_.Init(
      DEFAULT_CURRENT_KP, DEFAULT_CURRENT_KI, DEFAULT_CURRENT_KD,
      0.00005f, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, 0.001f,
      PID_MODE_POSITION);

  HAL_ADCEx_InjectedStart(&hadc1); // 只启动ADC，不启动中断

  UpdateCurrentOffsets(); // 更新电流偏置

  HAL_ADCEx_InjectedStop(&hadc1);

  CalibrateZeroElectricAngle();
  // zeroElectricAngle_ = 5.43f;
  // zeroElectricAngle_ = 0.05f;

  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
  HAL_ADCEx_InjectedStart_IT(&hadc1);



  return true;
}

void FOC::GetMechanicalAngle()
{
  motorAngle_ = GetAngle(angleRead());
}

void FOC::CalibrateZeroElectricAngle()
{
  SetPhaseVoltage(0.0f, 3.0f, 0.0f); // 在d轴上施加3V电压，q轴电压为0，相当于在alpha轴上施加3V


  vTaskDelay(pdMS_TO_TICKS(1000));

  float angleSum = 0;
  int samples = 100;
  for (int i=0; i<samples; i++)
  {
    angleSum += GetAngle(angleRead());
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  float mechanicalAngle_locked = angleSum / samples;


  zeroElectricAngle_ = NormalizeAngle((float)(POLE_PAIRS) * mechanicalAngle_locked);

  SetPhaseVoltage(0.0f, 0.0f, 0.0f);
}

void FOC::ClarkeParkTransform(float electricalAngle)
{
  float Ialpha = Ia_ * 2.0f / 3.0f - (Ib_ + Ic_) / 3.0f;
  float Ibeta = (Ib_ - Ic_) * 0.57735026919f;

  Iq_ = Ibeta * arm_cos_f32(electricalAngle) - Ialpha * arm_sin_f32(electricalAngle);
  Id_ = Ialpha * arm_cos_f32(electricalAngle) + Ibeta * arm_sin_f32(electricalAngle);

  Iq_ = IqFilter_.LPF_Update(Iq_); // 低通滤波处理 Iq
  Id_ = IdFilter_.LPF_Update(Id_); // 低通滤波处理 Id

  Iq_ = -Iq_;
}

void FOC::PositionControl(float targetPosition)
{
  // 位置控制逻辑
  float currentPosition = motorAngle_; // 获取当前机械角度
  // 使用 PID 控制器计算速度命令
  float targetSpeed = positionPid_.Compute(currentPosition, targetPosition);

  float currentSpeed = velocity_;

  float uq = speedPid_.Compute(currentSpeed, targetSpeed);

  // 设置相电压
  SetPhaseVoltage(uq, 0.0f, GetElectricAngle(currentPosition));
}

void FOC::SpeedControl(float targetSpeed)
{
  float currentPosition = motorAngle_; // 获取当前机械角度
  float electricalAngle = GetElectricAngle(currentPosition);
  float currentSpeed = velocity_;

  float uq = speedPid_.Compute(currentSpeed, targetSpeed);


  SetPhaseVoltage(uq, 0.0f, GetElectricAngle(currentPosition)); // 设置相电压
}

void FOC::TorqueControl(float targetTorque)
{

}


float FOC::NormalizeAngle(float angle)
{
  float a = fmod(angle, 2 * _PI);
  return a >= 0 ? a : (a + 2 * _PI);
}

float FOC::GetElectricAngle(float shaftAngle)
{
  return NormalizeAngle((float)(POLE_PAIRS) * shaftAngle - zeroElectricAngle_);
}

void FOC::SetPhaseVoltage(float uq, float ud, float electricalAngle)
{
  uAlpha_ = -uq * arm_sin_f32(electricalAngle) + ud * arm_cos_f32(electricalAngle);
  uBeta_ = uq * arm_cos_f32(electricalAngle) + ud * arm_sin_f32(electricalAngle);

  const float vmax    = BATVEL / _SQRT3;            // 允许的最大空间矢量幅值
  const float vmax2   = vmax * vmax;                // 先算平方，省掉一次开方
  float vmag2         = uAlpha_ * uAlpha_ + uBeta_ * uBeta_;

  if (vmag2 > vmax2)
  {
      // 需要缩放时再开方（或改用 arm_inv_sqrt_f32 更快）
    float scale = vmax / sqrtf(vmag2);
    uAlpha_ *= scale;
    uBeta_  *= scale;
  }

  SvpwmSector();
  SetPwm();
}

void FOC::SvpwmSector()
{
  float ta = 0.0f;
  float tb = 0.0f;
  float tc = 0.0f;
  float k = (TS * _SQRT3) * INVBATVEL;
  float va = uBeta_;
  float vb = (_SQRT3 * uAlpha_ - uBeta_) * 0.5f;
  float vc = (-_SQRT3 * uAlpha_ - uBeta_) * 0.5f;
  int a = (va > 0) ? 1 : 0;
  int b = (vb > 0) ? 1 : 0;
  int c = (vc > 0) ? 1 : 0;
  SectorType sector = static_cast<SectorType>((c << 2) | (b << 1) | a);

  switch (sector)
  {
  case SectorType::Sector_1:
  {
    float t4 = k * vb;
    float t6 = k * va;
    float t0 = (TS - t4 - t6) * 0.5f;

    ta       = t4 + t6 + t0;
    tb       = t6 + t0;
    tc       = t0;
  }
  break;

  case SectorType::Sector_2:
  {
    float t6 = -k * vc;
    float t2 = -k * vb;
    float t0 = (TS - t2 - t6) * 0.5f;

    ta       = t6 + t0;      // a相占空比时间计算
    tb       = t2 + t6 + t0; // b相占空比时间计算
    tc       = t0;           // c相占空比时间为t0
  }
  break;

  case SectorType::Sector_3:
  {
    float t2 = k * va;
    float t3 = k * vc;
    float t0 = (TS - t2 - t3) * 0.5f;

    ta       = t0;           // a相占空比时间为t0
    tb       = t2 + t3 + t0; // b相占空比时间计算
    tc       = t3 + t0;      // c相占空比时间计算
  }
  break;

  case SectorType::Sector_4:
  {
    float t1 = -k * va;
    float t3 = -k * vb;
    float t0 = (TS - t1 - t3) * 0.5f;

    ta       = t0;           // a相占空比时间为t0
    tb       = t3 + t0;      // b相占空比时间计算
    tc       = t1 + t3 + t0; // c相占空比时间计算
  }
  break;

  case SectorType::Sector_5:
  {
    float t1 = k * vc;
    float t5 = k * vb;
    float t0 = (TS - t1 - t5) * 0.5f;

    ta       = t5 + t0;      // a相占空比时间计算
    tb       = t0;           // b相占空比时间为t0
    tc       = t1 + t5 + t0; // c相占空比时间计算
  }
  break;

  case SectorType::Sector_6:
  {
    float t4 = -k * vc;
    float t5 = -k * va;
    float t0 = (TS - t4 - t5) * 0.5f;

    ta       = t4 + t5 + t0; // a相占空比时间计算
    tb       = t0;           // b相占空比时间为t0
    tc       = t5 + t0;      // c相占空比时间计算
  }
  break;

  default:
    break;
  }

  voltageA_Duty_ = ta;
  voltageB_Duty_ = tb;
  voltageC_Duty_ = tc;

  voltageA_Duty_ = constrain(ta, 0.0f, 1.0f);
  voltageB_Duty_ = constrain(tb, 0.0f, 1.0f);
  voltageC_Duty_ = constrain(tc, 0.0f, 1.0f);
}

void FOC::SetPwm()
{
  __HAL_TIM_SET_COMPARE(pwmTimer_, TIM_CHANNEL_1, static_cast<uint16_t>(voltageA_Duty_ * PWM_MAX_VALUE));
  __HAL_TIM_SET_COMPARE(pwmTimer_, TIM_CHANNEL_2, static_cast<uint16_t>(voltageB_Duty_ * PWM_MAX_VALUE));
  __HAL_TIM_SET_COMPARE(pwmTimer_, TIM_CHANNEL_3, static_cast<uint16_t>(voltageC_Duty_ * PWM_MAX_VALUE));
}


float FOC::GetVelocity(float angle)
{
  static uint64_t timeLast = 0;
  static float angleLast = 0.0f;
  static bool firstRun = true;

  uint64_t timeNow = DWT_GetTimeline_us();

  // 第一次运行时初始化
  if (firstRun)
  {
    timeLast = timeNow;
    angleLast = angle;
    firstRun = false;
    return 0.0f;
  }

  float ts = (float)(timeNow - timeLast) / 1000000.0f; // 时间间隔，单位为秒

  // 防止除零和异常时间间隔
  if (ts <= 0.0f || ts > 0.5f)
  {
    return 0.0f; // 返回0或保持上次结果
  }

  // 处理角度跳跃（0-2π边界）
  float angleDiff = angle - angleLast;


  float velocity = angleDiff / ts;
  float velocityFiltered = speedFilter_.LPF_Update(velocity);

  // 更新历史值
  angleLast = angle;
  timeLast = timeNow;

  return velocityFiltered;
}

float FOC::LowPassFilterUpdate(LowPassFilterTypedef *filter, float input)
{
    // 一阶低通滤波公式：output = alpha * input + (1 - alpha) * prevValue
    filter->prevValue = filter->alpha * input + (1 - filter->alpha) * filter->prevValue;
    return filter->prevValue;
}

void FOC::SetPIDParameters(float kp_speed, float ki_speed, float kd_speed,
                          float kp_pos, float ki_pos, float kd_pos,
                          float kp_current, float ki_current, float kd_current)
{
  *speedPid_.KpPtr() = kp_speed;
  *speedPid_.KiPtr() = ki_speed;
  *speedPid_.KdPtr() = kd_speed;

  *positionPid_.KpPtr() = kp_pos;
  *positionPid_.KiPtr() = ki_pos;
  *positionPid_.KdPtr() = kd_pos;

  *currentPid_.KpPtr() = kp_current;
  *currentPid_.KiPtr() = ki_current;
  *currentPid_.KdPtr() = kd_current;
}

// 安全监控相关方法实现

/**
 * @brief 紧急停止电机
 */
void FOC::EmergencyStop()
{
  // 立即设置所有相电压为0
  SetPhaseVoltage(0.0f, 0.0f, 0.0f);

  // 重置PID控制器积分项，避免积分饱和
  speedPid_.ClearIntegral();
  positionPid_.ClearIntegral();
  currentPid_.ClearIntegral();
}


void FOC::UpdateCurrent()
{
  float adcValueIa = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1); // 获取 IA 电流
  float adcValueIb = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  float adcValueIc = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
  float adcValueVbus = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4); // 获取 VBUS 电压

  Ia_ = (float)(adcValueIa - currentAOffset_) * FAC_CURRENT_ADC;
  Ib_ = (float)(adcValueIb - currentBOffset_) * FAC_CURRENT_ADC;
  Ic_ = (float)(adcValueIc - currentCOffset_) * FAC_CURRENT_ADC;
  Vbus_ = (float)adcValueVbus * FAC_VOLTAGE_ADC; // 根据 ADC 分辨率和参考电压计算 VBUS
}

void FOC::UpdateCurrentOffsets()
{
    long long tempAOffset = 0;
    long long tempBOffset = 0;
    long long tempCOffset = 0;
    for (int i = 0; i < 2000; i++)
    {
      HAL_ADCEx_InjectedPollForConversion(&hadc1, 1); // 等待转换完成
      tempAOffset += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
      tempBOffset += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
      tempCOffset += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
      vTaskDelay(pdMS_TO_TICKS(1)); // 延时 1 毫秒
    }
    currentAOffset_ = (float)tempAOffset / 2000.0f;
    currentBOffset_ = (float)tempBOffset / 2000.0f;
    currentCOffset_ = (float)tempCOffset / 2000.0f;
}


void FOC::MotorControlTask()
{
  switch (controlMode_)
  {
  case ControlMode::NONE:
  {
    return;
    break;
  }
  case ControlMode::TORQUE:
  {
    currentPid_.SetOutputLimits(-CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT);

    // targetTorque_ = constrain(targetTorque_, -TORQUE_LIMIT, TORQUE_LIMIT);
    // targetCurrent_ = constrain(targetTorque_ / TORQUE_CONST, -CURRENT_LIMIT, CURRENT_LIMIT);

    float maxStepSize = CURRENT_MEASURE_PERIOD * MOTOR_CURRENT_RAMP_RATE;
    float fullstep = targetCurrent_ - iqSetpoint_;
    float step = constrain(fullstep, -maxStepSize, maxStepSize);
    iqSetpoint_ += step;
    break;

  }

  case ControlMode::VELOCITY:
    speedPid_.SetOutputLimits(-SPEED_PID_MAX_OUT, SPEED_PID_MAX_OUT);
    currentPid_.SetOutputLimits(-CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT);
    break;

  case ControlMode::POSITION:
    // positionPid_.SetOutputLimits(-POSITION_PID_MAX_OUT, POSITION_PID_MAX_OUT);
    speedPid_.SetOutputLimits(-SPEED_PID_MAX_OUT, SPEED_PID_MAX_OUT);
    currentPid_.SetOutputLimits(-CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT);
    break;

  case ControlMode::MIT:
  {
    positionPid_.SetOutputLimits(-POSITION_PID_MAX_OUT, POSITION_PID_MAX_OUT);
    speedPid_.SetOutputLimits(-SPEED_PID_MAX_OUT, SPEED_PID_MAX_OUT);
    currentPid_.SetOutputLimits(-CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT);
    targetTorque_ = constrain(targetTorque_, -TORQUE_LIMIT, TORQUE_LIMIT);
    targetCurrent_ = constrain(targetTorque_ / TORQUE_CONST, -CURRENT_LIMIT, CURRENT_LIMIT);
    float currentPosition = motorAngle_; // 获取当前机械角度
    float currentSpeed = velocity_;

    iqSetpoint_ = targetCurrent_ + mitKp_ * (targetPosition_ - currentPosition) + mitKd_ * (targetSpeed_ - currentSpeed);
    break;
  }
  }


  float electricalAngle = GetElectricAngle(motorAngle_);
  ClarkeParkTransform(electricalAngle);

  if (controlMode_ == ControlMode::VELOCITY || controlMode_ == ControlMode::POSITION)
  {
    if (loopCount_ == 0) // Tick() 已完成 20:1 降频，loopCount_==0 表示慢速环时刻
    {
      if (controlMode_ == ControlMode::VELOCITY)
      {
        velocitySetpoint_ = constrain(targetSpeed_, -SPEED_LIMIT, SPEED_LIMIT);
      }
      else if (controlMode_ == ControlMode::POSITION)
      {
        float currentPosition = motorAngle_; // 获取当前机械角度
        velocitySetpoint_ = positionPid_.Compute(currentPosition, targetPosition_);
        velocitySetpoint_ = constrain(velocitySetpoint_, -SPEED_LIMIT, SPEED_LIMIT);
      }

      float currentSpeed = velocity_;
      iqSetpoint_ = speedPid_.Compute(currentSpeed, velocitySetpoint_);
      iqSetpoint_ = constrain(iqSetpoint_, -CURRENT_LIMIT, CURRENT_LIMIT);

    }
  }

  iqSetpoint_ = constrain(iqSetpoint_, -CURRENT_LIMIT, CURRENT_LIMIT);
  Iq_ = constrain(Iq_, -CURRENT_LIMIT, CURRENT_LIMIT);
  float uq = currentPid_.Compute(Iq_, iqSetpoint_);
  SetPhaseVoltage(uq, 0.0f, GetElectricAngle(motorAngle_)); // 设置相电压

}

void FOC::UpdateMotorAngle()
{
  angleSingleTurn_ = angleRead();
  motorAngle_ = GetAngle(angleSingleTurn_);
}

void FOC::UpdateVelocity()
{
  velocity_ = GetVelocity(motorAngle_);
}

void FOC::Tick()
{
  UpdateCurrent();
  UpdateMotorAngle();
  loopCount_++;
  if (loopCount_ >= 20)
  {
    loopCount_ = 0;
    UpdateVelocity();
  }

  switch (motorMode_)
  {
  case MotorMode::INIT:
    motorMode_ = MotorMode::NONE;
    break;

  case MotorMode::NONE:
    motorMode_ = MotorMode::RUNNING;
    targetSpeed_ = 0.0f;
    velocitySetpoint_ = 0.0f;
    controlMode_ = ControlMode::TORQUE;
    targetCurrent_ = 0.0f;
    break;

  case MotorMode::RUNNING:
    MotorControlTask();
    break;

  default:
    break;
  }
}

void FOC::SetTorqueTarget(float current)
{
  targetCurrent_ = current;
  controlMode_ = ControlMode::TORQUE;
}

void FOC::SetVelocityTarget(float speed)
{
  targetSpeed_ = speed;
  controlMode_ = ControlMode::VELOCITY;
}

void FOC::SetPositionTarget(float position)
{
  targetPosition_ = position;
  controlMode_ = ControlMode::POSITION;
}

void FOC::SetMitTarget(float pos, float vel, float torque, float kp, float kd)
{
  targetPosition_ = pos;
  targetSpeed_ = vel;
  targetTorque_ = torque;
  mitKp_ = kp;
  mitKd_ = kd;
  controlMode_ = ControlMode::MIT;
}

void FOC::Stop()
{
  controlMode_ = ControlMode::NONE;
  iqSetpoint_ = 0.0f;
  idSetpoint_ = 0.0f;
  velocitySetpoint_ = 0.0f;
  positionSetpoint_ = 0.0f;
  SetPhaseVoltage(0.0f, 0.0f, 0.0f);
}
