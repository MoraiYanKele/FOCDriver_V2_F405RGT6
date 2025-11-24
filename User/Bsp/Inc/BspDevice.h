#ifndef __BSP_DEVICE_H__
#define __BSP_DEVICE_H__

#include "common_inc.h"
#include "BspStatus.hpp"

typedef void (*Callback_t)(void);

typedef enum {
    DEVICE_NONE = -1,

    // 定义设备类别范围
    DEVICE_TIMER_START = 0,
    DEVICE_TIMER_1 = DEVICE_TIMER_START,
    DEVICE_TIMER_2,
    DEVICE_TIMER_3,
    DEVICE_TIMER_END,


    DEVICE_CAN_START = DEVICE_TIMER_END,
    DEVICE_CAN_1 = DEVICE_CAN_START,
    DEVICE_CAN_END,

    DEVICE_USART_START = DEVICE_CAN_END,
    DEVICE_USART_1 = DEVICE_USART_START,
    DEVICE_USART_END,

    DEVICE_PWM_START = DEVICE_USART_END,
    DEVICE_PWM_1 = DEVICE_PWM_START,
    DEVICE_PWM_3,
    DEVICE_PWM_END,

    DEVICE_SPI_START = DEVICE_PWM_END,
    DEVICE_SPI_1 = DEVICE_SPI_START,
    DEVICE_SPI_END,

    DEVICE_I2C_START = DEVICE_SPI_END,
    DEVICE_I2C_1 = DEVICE_I2C_START,
    DEVICE_I2C_END,

    DEVICE_ADC_START = DEVICE_I2C_END,
    DEVICE_ADC_1 = DEVICE_ADC_START,
    DEVICE_ADC_END,

    DEVICE_COUNT
} BspDevice_t;


#ifdef __cplusplus

BspResult<bool> Bsp_SetDeviceHandle(BspDevice_t _devID, void* _handle);
BspResult<void*> Bsp_GetDeviceHandle(BspDevice_t deviceID);
BspResult<BspDevice_t> Bsp_FindDeviceByHandle(void* _deviceHandle);



BspResult<bool> Bsp_StartDevice(BspDevice_t _deviceID);
BspResult<bool> Bsp_StopDevice(BspDevice_t _deviceID);

#endif

#endif // __BSP_DEVICE_H__