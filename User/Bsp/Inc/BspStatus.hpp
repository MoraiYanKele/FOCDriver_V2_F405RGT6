#ifndef __BSP_STATUS_H__
#define __BSP_STATUS_H__

#include "stm32f4xx_hal.h"
#include "common_inc.h" // 包含 Printf 函数声明



#ifdef __cplusplus

enum class BspError : uint8_t
{
  OK = 0,                 // 操作成功
  InvalidDevice,          // 无效的设备ID
  NullHandle,             // 空句柄
  DeviceBusy,             // 设备忙
  DeviceStateError,       // 设备状态错误
  DeviceNotFound,         // 设备未找到
  InvalidParam,           // 无效参数
  BufferError,            // 缓冲区错误
  HalError,               // HAL库错误
  Timeout,                // 超时
  Unsupported,            // 不支持
  InitError               // 初始化错误
};

inline const char *BspErrorToString(BspError error)
{
  switch (error)
  {
  case BspError::OK:                return "OK";
  case BspError::InvalidDevice:     return "InvalidDevice";
  case BspError::NullHandle:        return "NullHandle";
  case BspError::DeviceBusy:        return "DeviceBusy";
  case BspError::DeviceStateError:  return "DeviceStateError";
  case BspError::DeviceNotFound:    return "DeviceNotFound";
  case BspError::InvalidParam:      return "InvalidParam";
  case BspError::BufferError:       return "BufferError";
  case BspError::HalError:          return "HalError";
  case BspError::Timeout:           return "Timeout";
  case BspError::Unsupported:       return "Unsupported";
  case BspError::InitError:         return "InitError";
  default:                          return "Unknown";
  }
}

struct BspErrorContext
{
  const char* file = nullptr;
  int line = 0;
  const char* function = nullptr;
};

struct ResultBase
{
  BspError error = BspError::OK;
  BspErrorContext context{};
  constexpr bool ok() const { return error == BspError::OK; }
};


template <typename T>
struct BspResult : ResultBase
{
  T value{};

  static constexpr BspResult success(const T& val = T{})
  {
    return BspResult{ ResultBase{ BspError::OK, {} }, val };
  }

  static constexpr BspResult failure(BspError err, const T& val = T{}, BspErrorContext ctx = {})
  {
    return BspResult{ ResultBase{ err, ctx }, val };
  }
};


#define BSP_RETURN_FAILURE(err, type) \
  return BspResult<type>::failure((err), {}, {__FILE__, __LINE__, __func__})


#define BSP_CHECK(expr, err, type) \
  do { if (!(expr)) { BSP_RETURN_FAILURE((err), type); } } while (0)


inline BspError BspErrorFromHalStatus(HAL_StatusTypeDef status)
{
  switch (status)
  {
  case HAL_OK:
    return BspError::OK;
  case HAL_TIMEOUT:
    return BspError::Timeout;
  case HAL_BUSY:
    return BspError::DeviceBusy;
  default:
    return BspError::HalError;
  }
}


inline void BspLogError(const ResultBase& result, const char* action)
{
  const auto& ctx = result.context;
  Printf("[BSP] %s failed: %s (%s:%d, %s)\n",
        action,
        BspErrorToString(result.error),
        ctx.file ? ctx.file : "unknown",
        ctx.line,
        ctx.function ? ctx.function : "unknown");
}

#endif // __cplusplus

#endif // __BSP_STATUS_H__
