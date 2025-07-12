#include "CmdTask.h"
#include "usart.h"
#include "MotorTask.h"
#include "SafetyTask.h"
extern "C" {
#include "VOFA.h"
}

TaskHandle_t CmdTaskHandle = NULL;

uint8_t cmdBuffer[sizeof(CmdFrameTypedef) * 2]; // 命令帧缓冲区

CmdFrameTypedef cmdFrame; // 命令帧结构体
float receivedFloat = 0.0f;



/**
 * @brief 命令处理任务
 * @param argument 任务参数
 */
void CmdTask(void *argument)
{
  __HAL_UART_ENABLE_IT(CMD_UART, UART_IT_IDLE); // 启用空闲中断
  HAL_UARTEx_ReceiveToIdle_DMA(CMD_UART, cmdBuffer, sizeof(cmdBuffer)); // 启动DMA空闲中断接收
  __HAL_DMA_DISABLE_IT(CMD_DMA_HANDEL, DMA_IT_HT); // 禁用DMA半传输中断

  while (1)
  {    
    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待通知
    if (notificationValue > 0)
    {
      ReceiveCmdFrame(cmdBuffer, sizeof(cmdBuffer)); // 接收命令帧
      Delay(2);
      if (CheckCmdFrame(&cmdFrame))
      {
        
        // 根据命令类型处理不同的命令
        switch (cmdFrame.cmd)
        {
          case Cmd::SET_POSITION:
            ProcessFloatCommand(&cmdFrame);
            targetPosition = receivedFloat;
            break;
            
          case Cmd::SET_SPEED:
            ProcessFloatCommand(&cmdFrame);
            targetSpeed = receivedFloat;
            break;
            
          case Cmd::EMERGENCY_STOP:
            RequestEmergencyStop();
 
            break;
            
          case Cmd::CLEAR_SAFETY:
            ClearSafetyError();
     
            break;
            
          case Cmd::GET_TARGET_POSITION:
            {
              float position = targetPosition; // 获取目标位置
              int32_t positionInt = static_cast<int32_t>(position * 10000.0f); // 转换为整数格式
              CmdFrameTypedef responseFrame;
              responseFrame.header = Cmd::RESPONSE_HEADER;
              responseFrame.cmd = Cmd::GET_TARGET_POSITION;
              responseFrame.dataLength = sizeof(float);
              memcpy(responseFrame.data, &positionInt, sizeof(int32_t)); // 将整数数据放入命令帧
              responseFrame.footer = Cmd::RESPONSE_FOOTER;
              TransmitCmdFrame(&responseFrame); // 发送目标位置
       
            }
            break;
            
          case Cmd::GET_CURRENT_POSITION:
            {
              float currentPosition = motor.GetMechanicalAngle();
              int32_t positionInt = static_cast<int32_t>(currentPosition * 10000.0f); // 转换为整数格式
              CmdFrameTypedef responseFrame;
              responseFrame.header = Cmd::RESPONSE_HEADER;
              responseFrame.cmd = Cmd::GET_CURRENT_POSITION;
              responseFrame.dataLength = sizeof(float);
              memcpy(responseFrame.data, &positionInt, sizeof(int32_t));
              responseFrame.footer = Cmd::RESPONSE_FOOTER;
              
              TransmitCmdFrame(&responseFrame); // 发送当前位置信息
            }
            // 这里可以扩展为设置安全参数的命令
      
            break;
            
          default:
            // 处理其他命令或报告未知命令

            break;
        }
      } 
    }

    
    HAL_UARTEx_ReceiveToIdle_DMA(CMD_UART, cmdBuffer, sizeof(cmdBuffer)); // 启动DMA空闲中断接收
    __HAL_DMA_DISABLE_IT(CMD_DMA_HANDEL, DMA_IT_HT); // 禁用DMA半传输中断
    Delay(10);
  }
  
}

void TransmitCmdFrame(const CmdFrameTypedef *frame)
{
  if (frame == NULL)
    return; // 无效的命令帧

  HAL_UART_Transmit_DMA(CMD_UART, (uint8_t *)frame, sizeof(CmdFrameTypedef)); // 通过DMA发送命令帧
}

void TsetTransmitCmdFrame(uint8_t data)
{
  uint8_t frame[1];
  frame[0] = data; // 将数据放入帧中

  HAL_UART_Transmit_DMA(CMD_UART, frame, 1); // 通过DMA发送命令帧
}

bool ReceiveCmdFrame(uint8_t *rxData, uint16_t length)
{
  if (length < sizeof(CmdFrameTypedef))
    return false;
  
  // 将接收到的数据复制到命令帧结构体
  memcpy(&cmdFrame, rxData, sizeof(CmdFrameTypedef));
  return true;
}


bool ProcessFloatCommand(const CmdFrameTypedef *frame)
{
  if (frame == NULL || frame->dataLength < sizeof(float))
    return false; // 无效的命令帧或数据长度不足
  int32_t receiveInt = 0;
  memcpy(&receiveInt, frame->data, sizeof(int32_t)); // 从命令帧中获取整数数据
  receivedFloat = static_cast<float>(receiveInt) / 10000.0f; // 将整数转换为浮点数
  return true; // 成功处理命令
}

bool CheckCmdFrame(const CmdFrameTypedef *frame)
{
  if (cmdFrame.header == Cmd::FRAME_HEADER && cmdFrame.footer == Cmd::FRAME_FOOTER)
    return true; // 命令帧有效
  
  else 
    return false; // 命令帧无效
}