#ifndef __UART_DMA_H__
#define __UART_DMA_H__

#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>


#define  USE_UART_DMA // 用于其他文件判断是否包含了本文件


#define BUFFER_SIZE   128  // 每次发送的数据长度
typedef void (*UART_TxCallbackTypedef)(UART_HandleTypeDef *);

typedef struct 
{
  UART_HandleTypeDef *huart;          // 串口句柄
  uint8_t BufferA[BUFFER_SIZE];       // 缓冲区A
  uint8_t BufferB[BUFFER_SIZE];       // 缓冲区B
  uint16_t idleBufferLength;          // 待写入的缓冲区长度
  uint8_t *activeBuffer;              // 当前正在使用的缓冲区
  uint8_t *idleBuffer;                // 待写入的缓冲区
  volatile uint8_t txBusy;            // 发送状态
} DoubleBufferUartDMATypeDef;



void DoubleBufferUartDMA_Init(DoubleBufferUartDMATypeDef *uart);
void DoubleBufferUartDMA_Send(DoubleBufferUartDMATypeDef *uart, uint8_t *data, uint16_t size);
void TxCallBack_DoubleBufferUartDMA(DoubleBufferUartDMATypeDef *uart);

#endif