#include "Log.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstdlib>
#include "BspUart.h"

QueueHandle_t Log::logQueue = nullptr;
Log& Log::GetInstance()
{
    static Log instance;
    return instance;
}
void Log::Init(Uart& uartInstance)
{
  if (logQueue != nullptr) return; // 防止重复初始化

  Log& instance = GetInstance();
  instance.debugUart = &uartInstance; // 取地址保存

  instance.debugUart->Init(115200); // 初始化UART波特率
  instance.debugUart->EnableRxDMA(false); // 启用DMA接收

  instance.debugUart->SetRxCallback([](uint16_t dmaCurrentPos)
  {
    uint8_t rxData[64] = {0};
    auto receiveResult = Log::GetInstance().debugUart->ReceiveData(rxData, dmaCurrentPos);
    
    // 确保字符串以 \0 结尾，防止 strtof 越界
    rxData[receiveResult.value] = '\0';

    Log::ProcessRxData(rxData, receiveResult.value);
  }); // 设置接收回调

  logQueue = xQueueCreate(20, sizeof(char*)); // 创建日志队列
  xTaskCreate(LogTask, "LogTask", 1024, nullptr, osPriorityBelowNormal, nullptr);

}

void Log::Print(const char* fmt, ...)
{
  if (logQueue == nullptr) return; // 未初始化日志系统

  // 格式化日志消息
  char* logMsg = (char*)pvPortMalloc(LOG_MSG_MAX_SIZE); // 分配内存
  if (logMsg == nullptr) return; // 内存分配失败

  va_list args;
  va_start(args, fmt);
  vsnprintf(logMsg, LOG_MSG_MAX_SIZE, fmt, args);
  va_end(args);

  // 将日志消息发送到队列
  if (xQueueSend(logQueue, &logMsg, 0) != pdPASS) 
  {
    vPortFree(logMsg); // 队列满则丢弃并释放
  }
}

void Log::LogTask(void *arg)
{
  char* logMsg = nullptr;

  for (;;) 
  {
    // 等待日志消息
    if (xQueueReceive(logQueue, &logMsg, portMAX_DELAY) == pdPASS) 
    {
      if (logMsg != nullptr)
      {
        // 通过UART发送日志消息
        Log::GetInstance().debugUart->SendData((uint8_t*)logMsg, strlen(logMsg));
      }
      vPortFree(logMsg); // 释放内存
    }
  }
}

void Log::RegisterData_Vofa(const char* name, float* data)
{
  Log& instance = GetInstance();

  for (int i = 0; i < LOG_CMD_LIST_SIZE; i++)
  {
    if (instance.cmdList[i].controlData_float == data)
      return; // 已注册，直接返回
  }

  for (uint8_t i = 0; i < LOG_CMD_LIST_SIZE; ++i)
  {
    if (instance.cmdList[i].controlData_float == nullptr && instance.cmdList[i].dataName == nullptr)
    {
      instance.cmdList[i].dataName = name;
      instance.cmdList[i].controlData_float = data;
      break;
    }
  }
}



void Log::ProcessRxData(uint8_t* data, uint16_t len)
{
  auto& instance = GetInstance();
  instance.HandleCommand((char*)data);
}

void Log::HandleCommand(char* cmdStr)
{
  char temp[32] = {0};
  memcpy(temp, cmdStr, (strlen(cmdStr) < sizeof(temp)-1) ? strlen(cmdStr) : sizeof(temp)-1);
  for (int i = 0; i < LOG_CMD_LIST_SIZE; i++)
  {
    // if (cmdList[i].dataName == nullptr || cmdList[i].controlData_float == nullptr)
    //   continue;

    int nameLen = strlen(cmdList[i].dataName);
    if (strncmp(cmdStr, cmdList[i].dataName, nameLen) == 0 && cmdStr[nameLen] == ':')
    {

      char* valStr = cmdStr + nameLen + 1;

      if (cmdList[i].controlData_float != nullptr)
      {
        float val = strtof(valStr, nullptr);
        *(cmdList[i].controlData_float) = val;
      }
      break;
    }
  }
  
}