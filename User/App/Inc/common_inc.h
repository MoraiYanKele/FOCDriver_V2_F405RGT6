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

#include "dwt.h" // DWT 延时函数
#include "arm_math.h" // ARM CMSIS-DSP数学库


// #define USE_USB_VIRTUAL // 使用USB虚拟串口
#define  USE_UART_DMA 

#define _PI 3.14159265358979323846f // 定义 PI 常量

#define Delay(ms)   vTaskDelay(pdMS_TO_TICKS(ms)) // FreeRTOS延时函数

void userMain();

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/

#ifdef __cplusplus


#endif
#endif // __USER_MAIN_H__
