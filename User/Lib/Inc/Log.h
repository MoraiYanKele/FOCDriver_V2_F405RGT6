#pragma once

#ifndef __LOG_H__
#define __LOG_H__


#ifdef __cplusplus
extern "C" {
#endif

// ANSI Color Codes
#define ANSI_RESET   "\033[0m"
#define ANSI_BLACK   "\033[30m"
#define ANSI_RED     "\033[31m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_BLUE    "\033[34m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_CYAN    "\033[36m"
#define ANSI_WHITE   "\033[37m"

// Bold Colors
#define ANSI_B_RED     "\033[1;31m"
#define ANSI_B_GREEN   "\033[1;32m"
#define ANSI_B_YELLOW  "\033[1;33m"
#define ANSI_B_BLUE    "\033[1;34m"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// Log Macros
// Usage: LOG_INFO("System Ready, ID: %d", 1);
#define LOG_INFO(fmt, ...)  Log::GetInstance().Print(ANSI_B_GREEN "[INFO] " ANSI_RESET fmt "\r\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  Log::GetInstance().Print(ANSI_B_YELLOW "[WARN] " ANSI_RESET fmt "\r\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) Log::GetInstance().Print(ANSI_B_RED   "[ERROR] " ANSI_RESET fmt "\r\n", ##__VA_ARGS__)


// Raw Printf with color
#define LOG_COLOR(color, fmt, ...) Printf(color fmt ANSI_RESET, ##__VA_ARGS__)


#include "common_inc.h"
#include "BspUart.h"

static const uint8_t LOG_CMD_LIST_SIZE = 16;

struct VofaCmdTypedef
{
  float *controlData_float; 
  const char *dataName;
};

class Log
{
private:

  Log() = default;
  ~Log() = default;

  static void LogTask(void *arg);
  static constexpr size_t LOG_MSG_MAX_SIZE = 128;

  VofaCmdTypedef cmdList[LOG_CMD_LIST_SIZE] = {0};
  static QueueHandle_t logQueue;
  void HandleCommand(char* cmdStr);
  Uart* debugUart = nullptr; // 内部还是存指针比较方便，因为引用必须在构造时初始化
public:

  Log(const Log&) = delete;
  Log& operator=(const Log&) = delete;

  static Log& GetInstance();

  static void RegisterData_Vofa(const char* name, float* data);

  static void ProcessRxData(uint8_t* data, uint16_t len);

  static void Init(Uart& uartInstance);
  static void Print(const char* fmt, ...);
  
};

#endif // __cplusplus

#endif // __LOG_H__