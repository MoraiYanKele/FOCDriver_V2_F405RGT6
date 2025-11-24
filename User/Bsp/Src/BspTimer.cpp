#include "BspTimer.h"

static Timer* timerInstances[DEVICE_TIMER_END - DEVICE_TIMER_START] = {nullptr}; // 全局单例指针

/**
 * @brief Timer类构造函数
 * @param _deviceID 要初始化的TIMER设备ID (例如 DEVICE_TIMER_1)
 * @param _freqHz 定时器中断频率 (Hz)
 * @note  - 根据输入频率计算定时器的预分频值(PSC)和自动重装载值(ARR)
 *        - 初始化定时器基础设置
 *        - 将自身实例注册到全局实例表中
 */
BspResult<bool> Timer::Init(uint32_t _freqHz)
{
  if (_freqHz < 1) _freqHz = 1;
  else if (_freqHz > 10000000) _freqHz = 10000000;

  BSP_CHECK(deviceID >= DEVICE_TIMER_START && deviceID < DEVICE_TIMER_END, BspError::InvalidDevice, bool);

  auto handleResult = Bsp_GetDeviceHandle(deviceID);
  BSP_CHECK(handleResult.ok() && handleResult.value != nullptr, handleResult.error, bool);
  htim = static_cast<TIM_HandleTypeDef*>(handleResult.value);

  selfTimerCallback = nullptr;
  
  CalcRegister(_freqHz);
  __HAL_TIM_SET_PRESCALER(htim, PSC);
  __HAL_TIM_SET_AUTORELOAD(htim, ARR);
  
  timerInstances[deviceID - DEVICE_TIMER_START] = this;



  return BspResult<bool>::success(true);
}

/**
 * @brief  根据期望的频率计算PSC和ARR寄存器值
 * @param  _freq 期望的频率 (Hz)
 * @note   这是一个内部辅助函数，用于寻找最优的PSC和ARR组合
 *         以在满足精度要求的同时，尽可能获得最大的ARR值（即分辨率）
 *         它还考虑了不同总线（APB1/APB2）上的定时器时钟频率差异
 */
void Timer::CalcRegister(uint32_t _freq)
{
    bool isAPB2 = (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM9 || htim->Instance == TIM10 || htim->Instance == TIM11);
  
  uint32_t apbFreq = isAPB2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();

  RCC_ClkInitTypeDef clk;
  uint32_t flashLatency;
  HAL_RCC_GetClockConfig(&clk, &flashLatency);

  bool apbDivGt1 = isAPB2 ? (clk.APB2CLKDivider != RCC_HCLK_DIV1) : (clk.APB1CLKDivider != RCC_HCLK_DIV1);

  uint32_t timClk = apbFreq * (apbDivGt1 ? 2 : 1); // 定时器时钟频率

  uint32_t bestPSC = 0, bestARR = 0;
  for (uint32_t psc = 0; psc <= 0xFFFF; ++psc) 
  {
    uint32_t arrPlus1 = timClk / (_freq * (psc + 1));
    if (arrPlus1 <= 1) 
      break;

    uint32_t arr = arrPlus1 - 1;
    if (arr <= 0xFFFF) 
    {
      bestARR = arr;
      bestPSC = psc;
      break;
    }
  }
  PSC = (uint16_t)bestPSC;
  ARR = (uint32_t)bestARR;
  freq = timClk / ((PSC + 1) * (ARR + 1)); // 实际频率
}

/**
 * @brief  启动定时器
 * @note   以中断模式启动定时器
 */
BspResult<bool> Timer::Start()
{
  BSP_CHECK(deviceID >= DEVICE_TIMER_START && deviceID < DEVICE_TIMER_END, BspError::InvalidDevice, bool);
  BSP_CHECK(htim != nullptr, BspError::NullHandle, bool);

  auto startResult = Bsp_StartDevice(deviceID);
  BSP_CHECK(startResult.ok(), startResult.error, bool);

  HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(htim);
  if (status != HAL_OK)
  {
    auto rollback = Bsp_StopDevice(deviceID);
    if (!rollback.ok())
    {
      return BspResult<bool>::failure(rollback.error, false, rollback.context);
    }
    return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }

  return BspResult<bool>::success(true);
}

/**
 * @brief  停止定时器
 * @note   停止定时器中断
 */
BspResult<bool> Timer::Stop()
{
  BSP_CHECK(deviceID >= DEVICE_TIMER_START && deviceID < DEVICE_TIMER_END, BspError::InvalidDevice, bool);
  BSP_CHECK(htim != nullptr, BspError::NullHandle, bool);

  HAL_StatusTypeDef status = HAL_TIM_Base_Stop_IT(htim);
  if (status != HAL_OK)
  {
    return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }

  auto stopResult = Bsp_StopDevice(deviceID);
  BSP_CHECK(stopResult.ok(), stopResult.error, bool);

  return BspResult<bool>::success(true);
}

/**
 * @brief  设置用户自定义的定时器中断回调函数
 * @param  _timerCallback 用户提供的回调函数指针
 */
BspResult<bool> Timer::SetCallback(Callback_t _timerCallback)
{
  BSP_CHECK(_timerCallback != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(deviceID != DEVICE_NONE, BspError::InvalidDevice, bool);

  selfTimerCallback = _timerCallback;
  
  return BspResult<bool>::success(true);
}

/**
 * @brief  调用用户设置的回调函数
 * @note   这是一个内部函数，由蹦床函数调用，以保持封装性
 */
void Timer::InvokeCallback()
{
  if (selfTimerCallback != nullptr)
  {
    selfTimerCallback();
  }
}


/**
 * @brief  定时器中断回调的“蹦床”函数
 * @param  _TimerHandele 触发中断的TIMER句柄 (void*类型)
 * @note   这是一个全局C函数，作为HAL库和C++类成员函数之间的桥梁
 *         - 在HAL的中断处理函数(例如`HAL_TIM_PeriodElapsedCallback`)中被调用
 *         - 通过句柄找到对应的Timer实例
 *         - 调用实例的公共回调接口
 */
void Timer_Callback_Trampoline(void *_TimerHandele)
{

  auto findResult = Bsp_FindDeviceByHandle(_TimerHandele);

  if (!findResult.ok())
  {
    return; // 未找到对应设备，直接返回
  }

  BspDevice_t devID = findResult.value;

  if (devID >= DEVICE_TIMER_START && devID < DEVICE_TIMER_END)
  {
    Timer* instance = timerInstances[devID - DEVICE_TIMER_START];
    if (instance != nullptr)
    {
      instance->InvokeCallback();
    }
  }
}

