#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__

#include "common_inc.h"
#include "BspDevice.h"

#define CHIP_FREQ_MHZ 168.0f // 芯片主频 MHz

#ifdef __cplusplus
extern "C" {
#endif

void Timer_Callback_Trampoline(void *_TimerHandele);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Timer
{
private:
  TIM_HandleTypeDef* htim = nullptr;
  BspDevice_t deviceID = DEVICE_NONE;
  uint32_t freq = 100; // 频率 (Hz)
  uint16_t PSC = 168 - 1; // 预分频器
  uint32_t ARR = 10000 - 1; // 自动重装载值

  Callback_t selfTimerCallback = nullptr;

  void CalcRegister(uint32_t _freq);

public:
  explicit Timer(BspDevice_t _deviceID)
  {
    auto isValidDevice = (_deviceID >= DEVICE_TIMER_START && _deviceID < DEVICE_TIMER_END);
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

  BspResult<bool> Init(uint32_t _freqHz);
  BspResult<bool> Start();
  BspResult<bool> Stop();
  BspResult<bool> SetCallback(Callback_t _timerCallback);
  void InvokeCallback(); // 添加一个公共的调用函数
};
#endif // __cplusplus



#endif // __BSP_TIMER_H__