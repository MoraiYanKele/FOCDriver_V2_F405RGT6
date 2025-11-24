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

#include "RingBuffer.h"


void Uart_TxCpltCallback_Trampoline(void *_uartHandle);
void Uart_RxEventCallback_Trampoline(void *_uartHandle, uint16_t size);

#ifdef __cplusplus
  } // extern "C"
#endif // __cplusplus

class Uart
{
private:
  BspDevice_t deviceID;
  UART_HandleTypeDef* huart;

  RingBuffer_t ringBuffer_Tx; // 环形缓冲区对象
  RingBuffer_t ringBuffer_Rx; // 环形缓冲区对象

  Callback_t userTxCpltCallback = nullptr;
  Callback_t userRxCpltCallback = nullptr;

  static const uint32_t RING_BUFFER_SIZE = 128; // 环形缓冲区大小，必须是2的幂

  static const uint32_t DMA_RX_BUFFER_SIZE = 64;

  uint8_t dmaRxBuffer[DMA_RX_BUFFER_SIZE]; // DMA接收缓冲区

  char rxBuffer[RING_BUFFER_SIZE]; // 接收缓冲区
  char txBuffer[RING_BUFFER_SIZE]; // 发送缓冲区

  volatile bool txDmaBusyFlag = false; // DMA发送忙标志
  uint32_t lastTxDmaSize;

  void StartDmaTxIfIdle();
  BspResult<bool> ConfigureUart(uint32_t baud);


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

  BspResult<bool> SetTxCallback(Callback_t _userCallback);
  BspResult<bool> SetRxCallback(Callback_t _userCallback);

  BspResult<bool> EnableRxDMA(); // 启用DMA接收

  BspResult<uint32_t> SendData(const uint8_t* data, size_t size);
  BspResult<uint32_t> ReceiveData(uint8_t* data, size_t size); // 接收数据
  void Printf(const char *format, ...);

  void InvokeTxCallback();
  void InvokeRxCallback(uint16_t size);

  void TxCpltCallback(); // 发送完成回调
  void RxEventCallback(uint16_t size); // 新增：接收事件回调
  BspResult<uint32_t> GetRxDataLength() const; // 新增：获取接收缓冲区中待读数据的长度

  BspResult<bool> ClearRxBuffer();
  BspResult<bool> ClearTxBuffer();

};



#endif // __BSP_UART_H__