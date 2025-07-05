#ifndef __CMD_TASK_H__
#define __CMD_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif





#ifdef __cplusplus
}
#endif




#include "common_inc.h" // 包含必要的头文件

// CRC8校验相关定义
#define CRC8_POLYNOMIAL    0x07        // CRC8多项式 (x^8 + x^2 + x^1 + x^0)
#define CRC8_INIT_VALUE    0x00        // CRC8初始值
#define FRAME_HEADER       0xAA        // 帧头标识
#define FRAME_FOOTER       0x55        // 帧尾标识

// CRC8校验函数声明


// 优化的命令帧结构（支持CRC8校验）
typedef struct 
{
    uint8_t header;         // 帧头 0xAA
    uint8_t cmd;            // 命令类型
    uint8_t dataLength;     // 数据长度 (0-8字节)
    uint8_t data[4];        // 数据区 (可存储2个float或8个uint8_t)
    uint8_t crc8;           // CRC8校验值
    uint8_t footer;         // 帧尾 0x55
} CmdFrameTypedef;

// 命令类型定义
#define CMD_SET_POSITION    0x01    // 设置位置命令
#define CMD_SET_SPEED       0x02    // 设置速度命令
#define CMD_SET_CURRENT     0x03    // 设置电流命令
#define CMD_GET_STATUS      0x04    // 获取状态命令
#define CMD_SET_PARAMS      0x05    // 设置参数命令

// 数据联合体（支持多种数据类型）
typedef union
{
  float floatData; // 浮点数据
  uint8_t uint8Data[4]; // 4字节无符号整数数据
}CmdDataUnionTypedef;

extern TaskHandle_t CmdTaskHandle;

uint8_t CalculateCrc8(const uint8_t* data, uint16_t length);
bool ValidateFrameCrc8(const CmdFrameTypedef* frame);
void GenerateFrameCrc8(CmdFrameTypedef* frame);
bool ReceiveCmdFrame(uint8_t *rxData, uint16_t length);
bool ProcessFloatCommand(const CmdFrameTypedef *frame);
bool CheckCmdFrame(const CmdFrameTypedef *frame);

extern "C" {

void CmdTask(void *argument);

}



#endif // __CMD_TASK_H__
