#include "Callback.h"
#include "CmdTask.h"
#include "BspUart.h"
#include "BspCan.h"
#include "MotorTask.h"

extern "C" 
{
#include "VOFA.h"
}

extern uint8_t receiveFlag;



extern TaskHandle_t CmdTaskHandle; // 命令任务句柄

extern "C"
{

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc == &hadc1) // 检查是否是 ADC1
  {
    MotorTask();
  }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

  if (huart == VOFA_UART) 
  {
    VOFA_RxCallBack();
  }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) 
{
  // Uart_TxCpltCallback_Trampoline(huart);
// #ifdef VOFA_DEBUG
  if (huart == VOFA_UART) 
  {
    TxCallBack_DoubleBufferUartDMA(&uartToVOFA);
  }
// #endif
}


/**
 * @brief CAN FIFO0 接收消息挂起回调
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  Can_RxFifo0Callback_Trampoline(hcan);
}

/**
 * @brief CAN FIFO1 接收消息挂起回调
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  Can_RxFifo1Callback_Trampoline(hcan);
}

/**
 * @brief CAN 发送邮箱0完成回调
 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  Can_TxMailboxCallback_Trampoline(hcan, CAN_TX_MAILBOX0);
}

/**
 * @brief CAN 发送邮箱1完成回调
 */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  Can_TxMailboxCallback_Trampoline(hcan, CAN_TX_MAILBOX1);
}

/**
 * @brief CAN 发送邮箱2完成回调
 */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  Can_TxMailboxCallback_Trampoline(hcan, CAN_TX_MAILBOX2);
}
}

