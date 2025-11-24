#ifndef __COMMOM_INC_H__
#define __COMMOM_INC_H__

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "dwt.h" // DWT 延时函数
#include "arm_math.h" // ARM CMSIS-DSP数学库
#include "VOFA.h"


// #define USE_USB_VIRTUAL // 使用USB虚拟串口
// #define USE_UART_DMA        1
#define VOFA_DEBUG            1
#define MOTOR_4621            1 
#define MOTOR_2804            0 

#define CHIP_FREQ_MHZ 168.0f // 芯片主频 MHz

#define _PI 3.14159265358979323846f // 定义 PI 常量
#define _SQRT3 1.73205080756887729352f // 定义 根号3 常量

#define Delay(ms)         vTaskDelay(pdMS_TO_TICKS(ms)) // FreeRTOS延时函数


void userMain();

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/

#ifdef __cplusplus


#endif
#endif // __USER_MAIN_H__
