#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "common_inc.h"
#include "BspDevice.h"

#ifdef __cplusplus
  extern "C" {
#endif

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// #include "RingBuffer.h"


void Uart_TxCpltCallback_Trampoline(void *_uartHandle);
void Uart_RxEventCallback_Trampoline(void *_uartHandle, uint16_t size);
void Uart_ErrorCallback_Trampoline(void *_uartHandle);

#ifdef __cplusplus
  } // extern "C"
#endif // __cplusplus

typedef void (*UartRxCallback_t)(uint16_t _size);
typedef void (*UartTxCallback_t)(void);

class Uart
{
private:
  BspDevice_t deviceID;
  UART_HandleTypeDef* huart;

  UartTxCallback_t userTxCpltCallback = nullptr;
  UartRxCallback_t userRxCpltCallback = nullptr;

  static const uint32_t DMA_RX_BUFFER_SIZE = 64;
  static const uint32_t TX_BUFFER_SIZE = 256; // 发送缓冲区大小

  uint8_t dmaRxBuffer[DMA_RX_BUFFER_SIZE]; // DMA接收缓冲区
  volatile uint16_t lastDmaRxPos = 0; // 上次DMA接收位置
  bool rxCircularMode = true; // 是否启用环形DMA接收模式

  uint8_t txBuffers[2][TX_BUFFER_SIZE];
  volatile uint16_t txBufferCounts[2] = {0};
  volatile uint8_t fillIndex = 0; // 当前正在填充的缓冲区索引 (0 或 1)
  volatile bool txDmaBusyFlag = false; // DMA发送忙标志

  void StartDmaTx(uint8_t index); // 内部辅助函数：启动DMA发送
  BspResult<bool> ConfigureUart(uint32_t baud);
  BspResult<uint32_t> PushToRxBuffer(uint8_t *data, uint16_t len, size_t offset);

public:

  explicit Uart(BspDevice_t _deviceID)
  {
    auto isValidDevice = (_deviceID >= DEVICE_USART_START && _deviceID < DEVICE_USART_END);
    if (isValidDevice)
    {
      deviceID = _deviceID;
    }
    else
    {
      deviceID = DEVICE_NONE;
      huart = nullptr;
      return;
    }
  }

  BspResult<bool> Init(uint32_t baud);

  BspResult<bool> SetTxCallback(UartTxCallback_t _userCallback);
  BspResult<bool> SetRxCallback(UartRxCallback_t _userCallback);

  BspResult<bool> EnableRxDMA(bool circular = true);

  BspResult<uint32_t> SendData(const uint8_t* data, size_t size);
  BspResult<uint32_t> ReceiveData(uint8_t* data, uint16_t currentDmaPos); // 接收数据
  void Printf(const char *format, ...);

  void HandleError();

  void InvokeTxCallback();
  void InvokeRxCallback(uint16_t size);

  void TxCpltCallback(); // 发送完成回调
  void RxEventCallback(uint16_t size); // 新增：接收事件回调

  BspResult<bool> ClearRxBuffer();
  BspResult<bool> ClearTxBuffer();

  const char* GetInfo() const;

};



#endif // __BSP_UART_H__