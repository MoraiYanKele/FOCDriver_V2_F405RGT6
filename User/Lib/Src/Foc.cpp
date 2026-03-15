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
  
  pwmTimer = timer;
  
  // 初始化电机控制相关参数
  voltageA = 0.0f;
  voltageB = 0.0f;
  voltageC = 0.0f;
  Ia = 0.0f;
  Ib = 0.0f;
  Ic = 0.0f;

  // 启动PWM通道
  bool initSuccess = true;
  if (HAL_TIM_PWM_Start(pwmTimer, TIM_CHANNEL_1) != HAL_OK) initSuccess = false;
  if (HAL_TIM_PWM_Start(pwmTimer, TIM_CHANNEL_2) != HAL_OK) initSuccess = false;
  if (HAL_TIM_PWM_Start(pwmTimer, TIM_CHANNEL_3) != HAL_OK) initSuccess = false;
  if (HAL_TIM_PWM_Start(pwmTimer, TIM_CHANNEL_4) != HAL_OK) initSuccess = false;
  if (HAL_TIMEx_PWMN_Start(pwmTimer, TIM_CHANNEL_1) != HAL_OK) initSuccess = false;
  if (HAL_TIMEx_PWMN_Start(pwmTimer, TIM_CHANNEL_2) != HAL_OK) initSuccess = false;
  if (HAL_TIMEx_PWMN_Start(pwmTimer, TIM_CHANNEL_3) != HAL_OK) initSuccess = false;

  if (!initSuccess) 
  {
    return false;
  }

  // 对于高级定时器(如TIM1/TIM8)，必须使能主输出，否则PWM不会有任何输出
  __HAL_TIM_MOE_ENABLE(pwmTimer);

  // 初始化DWT计时器
  DWT_Init(168);

  // 初始化PID控制器
  PID_Init(&positionPidController, 
          DEFAULT_POSITION_KP, 
          DEFAULT_POSITION_KI, 
          DEFAULT_POSITION_KD, 
          0.001f, 
          POSITION_PID_MAX_OUT * 0.75f, POSITION_PID_MAX_OUT, 0.0f, 
          PID_MODE_POSITION);

  PID_Init(&speedPidController, 
          DEFAULT_SPEED_KP, 
          DEFAULT_SPEED_KI, 
          DEFAULT_SPEED_KD, 
          0.001f, 
          SPEED_PID_MAX_OUT, SPEED_PID_MAX_OUT * 0.8f, 0.01f, 
          PID_MODE_POSITION);

  PID_Init(&currentPidController,
          DEFAULT_CURRENT_KP, 
          DEFAULT_CURRENT_KI, 
          DEFAULT_CURRENT_KD, 
          0.00005f, 
          CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, 0.001f, 
          PID_MODE_POSITION);

  HAL_ADCEx_InjectedStart(&hadc1); // 只启动ADC，不启动中断

  UpdateCurrentOffsets(); // 更新电流偏置

  HAL_ADCEx_InjectedStop(&hadc1);

  CalibrateZeroElectricAngle();
  // zeroElectricAngle = 5.43f;
  // zeroElectricAngle = 0.05f;

  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
  HAL_ADCEx_InjectedStart_IT(&hadc1);



  return true;
}

void FOC::GetMechanicalAngle()
{
  motorAngle = GetAngle(angleRead());
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
  

  zeroElectricAngle = NormalizeAngle((float)(POLE_PAIRS) * mechanicalAngle_locked);
  
  SetPhaseVoltage(0.0f, 0.0f, 0.0f);
}

void FOC::ClarkeParkTransform(float electricalAngle)
{
  float Ialpha = Ia * 2.0f / 3.0f - (Ib + Ic) / 3.0f;
  float Ibeta = (Ib - Ic) * 0.57735026919f;

  Iq = Ibeta * arm_cos_f32(electricalAngle) - Ialpha * arm_sin_f32(electricalAngle); 
  Id = Ialpha * arm_cos_f32(electricalAngle) + Ibeta * arm_sin_f32(electricalAngle);

  Iq = IqFilter.LPF_Update(Iq); // 低通滤波处理 Iq
  Id = IdFilter.LPF_Update(Id); // 低通滤波处理 Id
  
  Iq = -Iq;
}

void FOC::PositionControl(float targetPosition)
{
  // 位置控制逻辑
  float currentPosition = motorAngle; // 获取当前机械角度
  // 使用 PID 控制器计算速度命令
  float targetSpeed = PIDCompute(&positionPidController, currentPosition, targetPosition);
  
  float currentSpeed = velocity;

  float uq = PIDCompute(&speedPidController, currentSpeed, targetSpeed);

  // 设置相电压
  SetPhaseVoltage(uq, 0.0f, GetElectricAngle(currentPosition));
}

void FOC::SpeedControl(float targetSpeed)
{
  float currentPosition = motorAngle; // 获取当前机械角度
  float electricalAngle = GetElectricAngle(currentPosition);
  float currentSpeed = velocity;

  float uq = PIDCompute(&speedPidController, currentSpeed, targetSpeed); 
  

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
  return NormalizeAngle((float)(POLE_PAIRS) * shaftAngle - zeroElectricAngle);
}

void FOC::SetPhaseVoltage(float uq, float ud, float electricalAngle)
{
  uAlpha = -uq * arm_sin_f32(electricalAngle) + ud * arm_cos_f32(electricalAngle);
  uBeta = uq * arm_cos_f32(electricalAngle) + ud * arm_sin_f32(electricalAngle);

  const float vmax    = BATVEL / _SQRT3;            // 允许的最大空间矢量幅值
  const float vmax2   = vmax * vmax;                // 先算平方，省掉一次开方
  float vmag2         = uAlpha * uAlpha + uBeta * uBeta;

  if (vmag2 > vmax2)
  {
      // 需要缩放时再开方（或改用 arm_inv_sqrt_f32 更快）
    float scale = vmax / sqrtf(vmag2);
    uAlpha *= scale;
    uBeta  *= scale;
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
  float va = uBeta;
  float vb = (_SQRT3 * uAlpha - uBeta) * 0.5f;
  float vc = (-_SQRT3 * uAlpha - uBeta) * 0.5f;
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

  voltageA_Duty = ta;
  voltageB_Duty = tb;
  voltageC_Duty = tc;

  voltageA_Duty = constrain(ta, 0.0f, 1.0f);
  voltageB_Duty = constrain(tb, 0.0f, 1.0f);
  voltageC_Duty = constrain(tc, 0.0f, 1.0f);
}

void FOC::SetPwm() 
{
  __HAL_TIM_SET_COMPARE(pwmTimer, TIM_CHANNEL_1, static_cast<uint16_t>(voltageA_Duty * PWM_MAX_VALUE));
  __HAL_TIM_SET_COMPARE(pwmTimer, TIM_CHANNEL_2, static_cast<uint16_t>(voltageB_Duty * PWM_MAX_VALUE));
  __HAL_TIM_SET_COMPARE(pwmTimer, TIM_CHANNEL_3, static_cast<uint16_t>(voltageC_Duty * PWM_MAX_VALUE));
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
  float velocityFiltered = speedFiliter.LPF_Update(velocity);

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
  // 重新配置速度PID
  speedPidController.Kp = kp_speed;
  speedPidController.Ki = ki_speed;
  speedPidController.Kd = kd_speed;
  
  // 重新配置位置PID
  positionPidController.Kp = kp_pos;
  positionPidController.Ki = ki_pos;
  positionPidController.Kd = kd_pos;
  
  // 重新配置电流PID
  currentPidController.Kp = kp_current;
  currentPidController.Ki = ki_current;
  currentPidController.Kd = kd_current;
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
  speedPidController.integral = 0.0f;
  positionPidController.integral = 0.0f;
  currentPidController.integral = 0.0f;
}


void FOC::UpdateCurrent()
{
  float adcValueIa = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1); // 获取 IA 电流
  float adcValueIb = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  float adcValueIc = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
  float adcValueVbus = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4); // 获取 VBUS 电压

  Ia = (float)(adcValueIa - currentAOffset) * FAC_CURRENT_ADC;
  Ib = (float)(adcValueIb - currentBOffset) * FAC_CURRENT_ADC;
  Ic = (float)(adcValueIc - currentCOffset) * FAC_CURRENT_ADC;
  Vbus = (float)adcValueVbus * FAC_VOLTAGE_ADC; // 根据 ADC 分辨率和参考电压计算 VBUS
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
    currentAOffset = (float)tempAOffset / 2000.0f;
    currentBOffset = (float)tempBOffset / 2000.0f;
    currentCOffset = (float)tempCOffset / 2000.0f;
}


void FOC::MotorControlTask()
{
  static uint8_t loopCount = 0;

  switch (controlMode)
  {
  case ControlMode::NONE:
  {
    return;
    break;
  }
  case ControlMode::TORQUE:
  {
    PID_SetOutputLimits(&currentPidController, -CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT);

    // targetTorque = constrain(targetTorque, -TORQUE_LIMIT, TORQUE_LIMIT);
    // targetCurrent = constrain(targetTorque / TORQUE_CONST, -CURRENT_LIMIT, CURRENT_LIMIT);

    float maxStepSize = CURRENT_MEASURE_PERIOD * MOTOR_CURRENT_RAMP_RATE;
    float fullstep = targetCurrent - iqSetpoint;
    float step = constrain(fullstep, -maxStepSize, maxStepSize);
    iqSetpoint += step;
    break;

  }

  case ControlMode::VELOCITY:
    PID_SetOutputLimits(&speedPidController, -SPEED_PID_MAX_OUT, SPEED_PID_MAX_OUT);
    PID_SetOutputLimits(&currentPidController, -CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT);
    break;

  case ControlMode::POSITION:
    // PID_SetOutputLimits(&positionPidController, -POSITION_PID_MAX_OUT, POSITION_PID_MAX_OUT);
    PID_SetOutputLimits(&speedPidController, -SPEED_PID_MAX_OUT, SPEED_PID_MAX_OUT);
    PID_SetOutputLimits(&currentPidController, -CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT);
    break;
  
  case ControlMode::MIT:
  {
    PID_SetOutputLimits(&positionPidController, -POSITION_PID_MAX_OUT, POSITION_PID_MAX_OUT);
    PID_SetOutputLimits(&speedPidController, -SPEED_PID_MAX_OUT, SPEED_PID_MAX_OUT);
    PID_SetOutputLimits(&currentPidController, -CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT);
    targetTorque = constrain(targetTorque, -TORQUE_LIMIT, TORQUE_LIMIT);
    targetCurrent = constrain(targetTorque / TORQUE_CONST, -CURRENT_LIMIT, CURRENT_LIMIT);
    float currentPosition = motorAngle; // 获取当前机械角度
    float currentSpeed = velocity;

    iqSetpoint = targetCurrent + mitKp * (targetPosition - currentPosition) + mitKd * (targetSpeed - currentSpeed);
    break;
  }

  case ControlMode::RATCHET:
  {
    PID_SetOutputLimits(&currentPidController, -CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT);

    float detentIndex = roundf(motorAngle / ratchetSpacing);
    float detentAngle = detentIndex * ratchetSpacing;
    float posError    = detentAngle - motorAngle;

    iqSetpoint = ratchetKp * posError - ratchetKd * velocity;
    iqSetpoint = constrain(iqSetpoint, -CURRENT_LIMIT, CURRENT_LIMIT);
    break;
  }

  case ControlMode::INERTIA:
    PID_SetOutputLimits(&currentPidController, -CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT);
    break;
  }


  float electricalAngle = GetElectricAngle(motorAngle);
  ClarkeParkTransform(electricalAngle);

  if (controlMode == ControlMode::VELOCITY || controlMode == ControlMode::POSITION)
  {
    loopCount++;
    if (loopCount >= 20) // 20kHz电流环 1kHz速度位置环
    {
      loopCount = 0;

      if (controlMode == ControlMode::VELOCITY)
      {
        velocitySetpoint = constrain(targetSpeed, -SPEED_LIMIT, SPEED_LIMIT);
      }
      else if (controlMode == ControlMode::POSITION)
      {
        float currentPosition = motorAngle; // 获取当前机械角度
        this->velocitySetpoint = PIDCompute(&this->positionPidController, currentPosition, targetPosition);
        this->velocitySetpoint = constrain(this->velocitySetpoint, -SPEED_LIMIT, SPEED_LIMIT);
      }

      float currentSpeed = velocity;
      iqSetpoint = PIDCompute(&speedPidController, currentSpeed, velocitySetpoint);
      iqSetpoint = constrain(iqSetpoint, -CURRENT_LIMIT, CURRENT_LIMIT);

    }
  }
  else if (controlMode == ControlMode::INERTIA)
  {
    loopCount++;
    if (loopCount >= 20) // 20kHz电流环 1kHz惯性环
    {
      loopCount = 0;

      if (fabsf(velocity) < inertiaVelocityThreshold)
      {
        // 电机已被停住（手捏或自然减速），清除惯性状态
        targetSpeed = 0.0f;
        iqSetpoint  = 0.0f;
      }
      else
      {
        // 加速时跟随（避免推动时产生阻力）
        if (fabsf(velocity) > fabsf(targetSpeed))
          targetSpeed = velocity;

        // 惯性衰减
        targetSpeed *= inertiaDecay;

        // 统一P控制输出，无分支跳变
        float speedError = targetSpeed - velocity;
        iqSetpoint = inertiaKp * speedError;
        iqSetpoint = constrain(iqSetpoint, -inertiaMaxCurrent, inertiaMaxCurrent);
      }
    }
  }

  
  iqSetpoint = constrain(iqSetpoint, -CURRENT_LIMIT, CURRENT_LIMIT);
  Iq = constrain(Iq, -CURRENT_LIMIT, CURRENT_LIMIT);
  float uq = PIDCompute(&currentPidController, Iq, iqSetpoint);
  SetPhaseVoltage(uq, 0.0f, GetElectricAngle(motorAngle)); // 设置相电压

}

void FOC::UpdateMotorAngle()
{
  angleSingleTurn = angleRead();
  motorAngle = GetAngle(angleSingleTurn);
}

void FOC::UpdateVelocity()
{
  velocity = GetVelocity(motorAngle);
}