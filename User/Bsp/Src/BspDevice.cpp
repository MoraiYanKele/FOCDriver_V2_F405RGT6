#include "BspDevice.h"

/**
 * @brief 设备引用计数数组
 * @note  用于跟踪每个设备是否已被某个上层模块占用
 *        - `false`: 设备空闲
 *        - `true`: 设备正在被使用
 */
static bool deviceRefCount[DEVICE_COUNT] = {0}; // 引用数组

/**
 * @brief 设备句柄数组
 * @note  这是一个核心的映射表，将 BspDevice_t 枚举与具体的STM32 HAL库句柄指针关联起来
 *        例如，deviceHandles[DEVICE_TIMER_2] 存储的是 &htim2
 */
static void* deviceHandles[DEVICE_COUNT] = 
{
  [DEVICE_TIMER_2] = &htim2,
  [DEVICE_CAN_1] = &hcan1,
  [DEVICE_USART_1] = &huart1, 
  [DEVICE_PWM_3] = &htim3,
}; // 设备句柄数组


/**
 * @brief  设置设备句柄，将指定的设备ID与HAL句柄关联
 * @param  _devID 要设置的设备ID
 * @param  _handle 要关联的HAL句柄指针
 * @return BspResult<bool> 操作结果，成功返回true，失败返回错误码
 * @note   此函数用于动态配置设备句柄，通常在初始化时使用
 */
BspResult<bool> Bsp_SetDeviceHandle(BspDevice_t _devID, void* _handle)
{
  BSP_CHECK(_devID >= DEVICE_NONE && _devID < DEVICE_COUNT, BspError::InvalidDevice, bool);
  BSP_CHECK(_handle != nullptr, BspError::NullHandle, bool);

  deviceHandles[_devID] = _handle;
  return BspResult<bool>::success();
}
/**
 * @brief  根据设备ID获取对应的HAL句柄
 * @param  deviceID 设备ID
 * @return BspResult<void*> 操作结果，成功返回句柄指针，失败返回错误码和nullptr
 * @note   用于获取设备句柄以进行HAL操作
 */
BspResult<void*> Bsp_GetDeviceHandle(BspDevice_t deviceID)
{
  BSP_CHECK(deviceID >= DEVICE_NONE && deviceID < DEVICE_COUNT, BspError::InvalidDevice, void*);

  BSP_CHECK(deviceHandles[deviceID] != nullptr, BspError::NullHandle, void*);

  return BspResult<void*>::success(deviceHandles[deviceID]);

}

/**
 * @brief  根据HAL句柄反向查找对应的设备ID
 * @param  _deviceHandle HAL句柄指针
 * @return BspResult<BspDevice_t> 操作结果，成功返回设备ID，失败返回错误码和DEVICE_NONE
 * @note   用于中断处理中将句柄映射回设备ID
 */
BspResult<BspDevice_t> Bsp_FindDeviceByHandle(void* _deviceHandle)
{
  for (BspDevice_t i = static_cast<BspDevice_t>(DEVICE_NONE + 1); i < DEVICE_COUNT; i = static_cast<BspDevice_t>(i + 1))
  {
    if (deviceHandles[i] == _deviceHandle)
    {
      return BspResult<BspDevice_t>::success(i);
    }
  }
  return BspResult<BspDevice_t>::failure(BspError::DeviceNotFound, DEVICE_NONE, {__FILE__, __LINE__, __func__});
}

/**
 * @brief  启动设备，标记设备为占用状态
 * @param  _deviceID 要启动的设备ID
 * @return BspResult<bool> 操作结果，成功返回true，失败返回错误码
 * @note   防止多个模块同时使用同一设备
 */
BspResult<bool> Bsp_StartDevice(BspDevice_t _deviceID)
{
  BSP_CHECK(_deviceID > DEVICE_NONE && _deviceID < DEVICE_COUNT, BspError::InvalidDevice, bool);

  if (deviceRefCount[_deviceID] == false)
  {
    deviceRefCount[_deviceID] = true;
    return BspResult<bool>::success(true);
  }

  return BspResult<bool>::failure(BspError::DeviceBusy, false, {__FILE__, __LINE__, __func__});
}

/**
 * @brief  停止设备，标记设备为闲置状态
 * @param  _deviceID 要停止的设备ID
 * @return BspResult<bool> 操作结果，成功返回true，失败返回错误码
 * @note   允许其他模块使用该设备
 */
BspResult<bool> Bsp_StopDevice(BspDevice_t _deviceID)
{
  BSP_CHECK(_deviceID > DEVICE_NONE && _deviceID < DEVICE_COUNT, BspError::InvalidDevice, bool);

  if (deviceRefCount[_deviceID] == true)
  {
    deviceRefCount[_deviceID] = false;
    return BspResult<bool>::success(true);
  }

  return BspResult<bool>::failure(BspError::DeviceBusy, false, {__FILE__, __LINE__, __func__});
}

// /**
//  * @brief 定时器回调函数指针数组
//  * @note  用于存储由上层(例如Timer类)设置的回调函数
//  */
// static Callback_t TimerCallbacks[DEVICE_TIMER_END - DEVICE_TIMER_START] = {nullptr};

// /**
//  * @brief UART回调函数指针数组
//  * @note  用于存储由上层(例如Uart类)设置的回调函数
//  */
// static Callback_t UartCallbacks[DEVICE_USART_END - DEVICE_USART_START] = {nullptr};



/**
//  * @brief  设置指定定时器的回调函数
//  * @param  _dev 定时器设备ID
//  * @param  _callback 回调函数指针
//  */
// void Bsp_SetTimerCallback(BspDevice_t _dev, Callback_t _callback)
// {
//   if (_dev >= DEVICE_TIMER_START && _dev < DEVICE_TIMER_END)
//   {
//     TimerCallbacks[_dev - DEVICE_TIMER_START] = _callback;
//   }
// }

// /**
//  * @brief  通过定时器句柄获取其对应的回调函数
//  * @param  _timerHandle 定时器HAL句柄指针
//  * @return Callback_t 找到的回调函数指针，未找到则返回NULL
//  */
// Callback_t Bsp_GetTimerCallback(void* _timerHandle)
// {
//   BspDevice_t devID = Bsp_FindDeviceByHandle(_timerHandle);
//   if (devID >= DEVICE_TIMER_START && devID < DEVICE_TIMER_END)
//   {
//     return TimerCallbacks[devID - DEVICE_TIMER_START];
//   }
//   return NULL;
// }

// /**
//  * @brief  设置指定UART的回调函数
//  * @param  _dev UART设备ID
//  * @param  _callback 回调函数指针
//  */
// void Bsp_SetUartCallback(BspDevice_t _dev, Callback_t _callback)
// {
//   if (_dev >= DEVICE_USART_START && _dev < DEVICE_USART_END)
//   {
//     UartCallbacks[_dev - DEVICE_USART_START] = _callback;
//   }
// }

// /**
//  * @brief  获取指定UART的回调函数
//  * @param  _dev UART设备ID
//  * @return Callback_t 找到的回调函数指针，未找到则返回NULL
//  */
// Callback_t Bsp_GetUartCallback(BspDevice_t _dev)
// {
//   if (_dev >= DEVICE_USART_START && _dev < DEVICE_USART_END)
//   {
//     return UartCallbacks[_dev - DEVICE_USART_START];
//   }
//   return NULL;
// }

