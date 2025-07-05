#include "Foc.h"
#include "LowPassFilter.h"  // 在实现文件中包含完整定义

extern "C" 
{
  #include "VOFA.h" 
  #include "pid.h"
  #include "MT6701.h"
}


FOC::FOC()
{
}

FOC::~FOC()
{
}

bool FOC::Init(TIM_HandleTypeDef* timer)
{
  if (timer == nullptr) {
    return false;
  }
  
  pwmTimer = timer;
  
  // 初始化电机控制相关参数
  voltageA = 0.0f;
  voltageB = 0.0f;
  voltageC = 0.0f;
  currentA = 0.0f;
  currentB = 0.0f;
  currentC = 0.0f;

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

  // 初始化DWT计时器
  DWT_Init(168);

  // 初始化PID控制器
  PID_Init(&positionPidController, 
          DEFAULT_POSITION_KP, 
          DEFAULT_POSITION_KI, 
          DEFAULT_POSITION_KD, 
          DEFAULT_PID_SAMPLE_TIME, 
          40.0f, 200.0f, 0.001f, 
          PID_MODE_POSITION);

  PID_Init(&speedPidController, 
          DEFAULT_SPEED_KP, 
          DEFAULT_SPEED_KI, 
          DEFAULT_SPEED_KD, 
          DEFAULT_PID_SAMPLE_TIME, 
          10.0f, 5.0f, 0.1f, 
          PID_MODE_POSITION);

  PID_Init(&currentPidController,
          DEFAULT_CURRENT_KP, 
          DEFAULT_CURRENT_KI, 
          DEFAULT_CURRENT_KD, 
          DEFAULT_PID_SAMPLE_TIME, 
          10.0f, 500.0f, 0.001f, 
          PID_MODE_POSITION);

  isInitialized = true;
  return true;
}

float FOC::GetMechanicalAngle()
{
  return GetAngle(angleRead());
}

float FOC::CalibrateZeroElectricAngle()
{
  SetPhaseVoltage(0.0f, 8.0f, 0.0f);
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  float angleSum = 0;
  int samples = 50;
  for (int i=0; i<samples; i++) 
  {
    angleSum += GetMechanicalAngle();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  float mechanicalAngle_locked = angleSum / samples;
  
  zeroElectricAngle = GetElectricAngle(mechanicalAngle_locked);
  
  SetPhaseVoltage(0.0f, 0.0f, 0.0f);
  

  return zeroElectricAngle;
}

void FOC::CalibrationCurrentOffset()
{

}

void FOC::Clarke(float electricalAngle)
{
  float Ialpha = -1 * (currentB + currentC);
  float Ibeta = (currentB - currentC) * 0.57735026919f;

  Iq = Ibeta * arm_cos_f32(electricalAngle) - Ialpha * arm_sin_f32(electricalAngle); 
  Id = Ialpha * arm_cos_f32(electricalAngle) + Ibeta * arm_sin_f32(electricalAngle);

  // Iq = IqFilter.LPF_Update(Iq); // 低通滤波处理 Iq
  // Id = IdFilter.LPF_Update(Id); // 低通滤波处理 Id
  
  Iq = -Iq;
}

void FOC::PositionControl(float targetPosition)
{
  // 位置控制逻辑
  float currentPosition = GetMechanicalAngle(); // 获取当前机械角度
  // 使用 PID 控制器计算速度命令
  float targetSpeed = PIDCompute(&positionPidController, currentPosition, targetPosition);
  
  float velocity = GetVelocity(currentPosition);
  
  float uq = PIDCompute(&speedPidController, velocity, targetSpeed);
  
  // 设置相电压
  SetPhaseVoltage(uq, 0.0f, GetElectricAngle(currentPosition));
}

void FOC::SpeedControl(float targetSpeed)
{
  float currentPosition = GetMechanicalAngle(); // 获取当前机械角度
  float currentSpeed = GetVelocity(currentPosition); // 获取当前速度

  float uq = PIDCompute(&speedPidController, currentSpeed, targetSpeed); 
  

  SetPhaseVoltage(uq, 0.0f, GetElectricAngle(currentPosition)); // 设置相电压
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
  

  voltageA = uAlpha + VOLTAGE_LIMIT / 2;
  voltageB = (0.5f * (-uAlpha + 1.73205080757f * uBeta)) + VOLTAGE_LIMIT / 2;
  voltageC = (0.5f * (-uAlpha - 1.73205080757f * uBeta)) + VOLTAGE_LIMIT / 2;
  // Printf("Voltage: %f, %f, %f\n", voltageA, voltageB, voltageC);/\

  SetPwm();
}

void FOC::SetPwm() 
{
  // if (!isInitialized || !pwmTimer) 
  // {
  //   return; // 未初始化，直接返回
  // }

  // 电压限制
  float vA = constrain(voltageA, 0.0f, VOLTAGE_LIMIT);
  float vB = constrain(voltageB, 0.0f, VOLTAGE_LIMIT);
  float vC = constrain(voltageC, 0.0f, VOLTAGE_LIMIT);

  // 计算占空比，使用浮点运算避免精度损失
  float dutyCycleA_f = (vA / VOLTAGE_LIMIT) * PWM_MAX_VALUE;
  float dutyCycleB_f = (vB / VOLTAGE_LIMIT) * PWM_MAX_VALUE;
  float dutyCycleC_f = (vC / VOLTAGE_LIMIT) * PWM_MAX_VALUE;

  // 转换为整数并限制范围（使用四舍五入提高精度）
  uint32_t dutyCycleA = constrain(static_cast<uint32_t>(dutyCycleA_f + 0.5f), 0U, PWM_MAX_VALUE);
  uint32_t dutyCycleB = constrain(static_cast<uint32_t>(dutyCycleB_f + 0.5f), 0U, PWM_MAX_VALUE);
  uint32_t dutyCycleC = constrain(static_cast<uint32_t>(dutyCycleC_f + 0.5f), 0U, PWM_MAX_VALUE);


  // Printf("Duty: %d, %d, %d\n", dutyCycleA, dutyCycleB, dutyCycleC);

  // 设置PWM占空比
  __HAL_TIM_SET_COMPARE(pwmTimer, TIM_CHANNEL_1, dutyCycleA);
  __HAL_TIM_SET_COMPARE(pwmTimer, TIM_CHANNEL_2, dutyCycleB);
  __HAL_TIM_SET_COMPARE(pwmTimer, TIM_CHANNEL_3, dutyCycleC);
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

