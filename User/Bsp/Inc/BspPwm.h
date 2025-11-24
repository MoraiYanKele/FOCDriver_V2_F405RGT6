#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#include "common_inc.h"
#include "BspDevice.h"

#define CHIP_FREQ_MHZ 168.0f // 芯片主频 MHz

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Pwm
{
private:
  TIM_HandleTypeDef* htim;
  BspDevice_t deviceID;
  uint32_t freq; // 频率 (Hz)
  uint16_t PSC = 168 - 1; // 预分频器
  uint32_t ARR = 10000 - 1; // 自动重装载值

  Callback_t selfTimerCallback = nullptr;

  void CalcRegister(uint32_t _freq);

public:

  enum PwmChannel_t
  {
    CHANNEL_1 = TIM_CHANNEL_1,
    CHANNEL_2 = TIM_CHANNEL_2,
    CHANNEL_3 = TIM_CHANNEL_3,
    CHANNEL_4 = TIM_CHANNEL_4,
    CHANNEL_ALL = 0xFF
  };


  explicit Pwm(BspDevice_t _deviceID)
  {
    auto isValidDevice = (_deviceID >= DEVICE_PWM_START && _deviceID < DEVICE_PWM_END);
    if (isValidDevice)
    {
      deviceID = _deviceID;
    }
    else
    {
      deviceID = DEVICE_NONE;
      htim = nullptr;
      return;
    }
  }
  BspResult<bool> Init(uint32_t _freqHz); // 初始化PWM

  BspResult<bool> Start(PwmChannel_t channel = CHANNEL_ALL); // 启动PWM  
  BspResult<bool> Stop(PwmChannel_t channel = CHANNEL_ALL);  // 停止PWM            
  BspResult<bool> SetDutyCycle(PwmChannel_t channel, float dutyCycle); // 设置占空比，范围0.0~1.0
  BspResult<bool> SetDutyTicks(PwmChannel_t channel, uint32_t ticks); // 设置占空比，范围0~ARR
  BspResult<bool> SetFrequency(uint32_t freqHz); // 设置频率

  BspResult<bool> WaitUpdate();
  
  BspResult<uint32_t> GetFrequency() const; // 获取当前频率
  BspResult<uint32_t> GetARR() const; // 获取ARR值
  BspResult<uint16_t> GetPSC() const; // 获取PSC值
};
#endif // __cplusplus





#endif // __BSP_PWM_H__