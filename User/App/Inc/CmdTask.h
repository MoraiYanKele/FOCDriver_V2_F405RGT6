#ifndef __CMD_TASK_H__
#define __CMD_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif





#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

#include "common_inc.h" // 包含必要的头文件

#define CMD_UART            &huart1 
#define CMD_DMA_HANDEL      &hdma_usart1_rx

// CRC8校验相关定义
#define CRC8_POLYNOMIAL    0x07        // CRC8多项式 (x^8 + x^2 + x^1 + x^0)
#define CRC8_INIT_VALUE    0x00        // CRC8初始值
// #define FRAME_HEADER       0xAA        // 帧头标识
// #define FRAME_FOOTER       0x55        // 帧尾标识


namespace Cmd 
{
  const uint8_t SET_POSITION = 0x01;
  const uint8_t SET_SPEED = 0x02;
  const uint8_t EMERGENCY_STOP = 0x10;
  const uint8_t CLEAR_SAFETY = 0x11;
  const uint8_t GET_TARGET_POSITION = 0x20; // 获取目标位置命令
  const uint8_t GET_CURRENT_POSITION = 0x21; // 获取当前位置信息命令
  const uint8_t FRAME_HEADER = 0xAA; // 帧尾标识
  const uint8_t FRAME_FOOTER = 0x55; // 帧尾标识
  const uint8_t RESPONSE_HEADER = 0xBB; // 帧头标识
  const uint8_t RESPONSE_FOOTER = 0x66; // 响应帧尾标识
}

// CRC8校验函数声明


// 优化的命令帧结构（支持CRC8校验）
typedef struct 
{
    uint8_t header;         // 帧头 0xAA
    uint8_t cmd;            // 命令类型
    uint8_t dataLength;     // 数据长度 (0-8字节)
    uint8_t data[4];        // 数据区 (可存储1个float或4个uint8_t)
    uint8_t footer;         // 帧尾 0x55
} CmdFrameTypedef;

// 命令类型定义
#define CMD_SET_POSITION     0x01    // 设置位置命令
#define CMD_SET_SPEED        0x02    // 设置速度命令
#define CMD_EMERGENCY_STOP   0x10    // 紧急停止命令
#define CMD_CLEAR_SAFETY     0x11    // 清除安全错误命令






// 数据联合体（支持多种数据类型）
typedef union
{
  float floatData; // 浮点数据
  uint8_t uint8Data[4]; // 4字节无符号整数数据
}CmdDataUnionTypedef;

extern TaskHandle_t CmdTaskHandle;



bool ProcessFloatCommand(const CmdFrameTypedef *frame);
bool CheckCmdFrame(const CmdFrameTypedef *frame);
void TransmitCmdFrame(const CmdFrameTypedef *frame);
void TsetTransmitCmdFrame(uint8_t data);

extern "C" {

bool ReceiveCmdFrame(uint8_t *rxData, uint16_t length);
void CmdTask(void *argument);

}
#endif // __cplusplus


#endif // __CMD_TASK_H__
