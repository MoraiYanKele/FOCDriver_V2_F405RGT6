/*
 * UART DMA双缓冲通信模块
 * 实现无阻塞DMA传输，通过乒乓缓冲机制提升通信效率
 */

 #include "Uart_DMA.h"

 static UART_TxCallbackTypedef UserTxCallback = NULL; // 用户自定义发送完成回调（当前未启用）
 
 /**
   * @brief  双缓冲DMA UART初始化
   * @param  uart 双缓冲结构体指针
   * @note   初始化双缓冲工作状态：
   *         - 设置BufferA为当前活动缓冲区
   *         - 设置BufferB为备用缓冲区
   *         - 初始化发送状态标志为就绪
   */
 void DoubleBufferUartDMA_Init(DoubleBufferUartDMATypeDef *uart)
 {
   uart->activeBuffer = uart->BufferA;    // 激活缓冲区指向BufferA
   uart->idleBuffer = uart->BufferB;      // 空闲缓冲区指向BufferB
   uart->txBusy = 0;                      // 发送状态标志复位
   uart->idleBufferLength = 0;            // 空闲缓冲区数据长度清零
 }
 
 /**
   * @brief  双缓冲DMA数据发送
   * @param  uart  双缓冲结构体指针
   * @param  data  待发送数据指针
   * @param  size  数据长度（字节）
   * @note   发送策略：
   *         - 当前无传输时：立即发送并占用激活缓冲区
   *         - 当前有传输时：缓存数据到空闲缓冲区
   *         注意：缓冲区尺寸需大于等于发送数据长度
   */
 void DoubleBufferUartDMA_Send(DoubleBufferUartDMATypeDef *uart, uint8_t *data, uint16_t size) 
 {
   if (uart->txBusy) // DMA正在传输
   {
     // 将数据缓存到空闲缓冲区（乒乓操作）
     memcpy(uart->idleBuffer, data, size);
     uart->idleBufferLength = size; // 记录待发送数据长度
   } 
   else  // DMA空闲
   {
     // 立即使用激活缓冲区开始传输
     memcpy(uart->activeBuffer, data, size);
     uart->txBusy = 1; // 标记发送状态为忙碌
     HAL_UART_Transmit_DMA(uart->huart, uart->activeBuffer, size);
   }
 }
 
 /**
   * @brief  DMA传输完成回调
   * @param  uart 双缓冲结构体指针
   * @note   执行缓冲区切换逻辑：
   *         1. 交换激活/空闲缓冲区指针
   *         2. 检查有空闲数据则继续传输
   *         应在DMA传输完成中断中调用
   */
 void TxCallBack_DoubleBufferUartDMA(DoubleBufferUartDMATypeDef *uart)
 {
   /* 缓冲区指针交换（乒乓切换） */
   uint8_t *temp = uart->activeBuffer;
   uart->activeBuffer = uart->idleBuffer;
   uart->idleBuffer = temp;
   
   uart->txBusy = 0; // 重置发送状态
 
   /* 检查待发缓存数据 */
   if (uart->idleBufferLength > 0)
   {
     uart->txBusy = 1; // 标记发送状态为忙碌
     // 立即发送空闲缓冲区数据
     HAL_UART_Transmit_DMA(uart->huart, uart->activeBuffer, uart->idleBufferLength);
     
     uart->idleBufferLength = 0; // 清空缓存数据长度
   }
 }