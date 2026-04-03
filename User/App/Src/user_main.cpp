#include "UserMain.h" // 包含用户主函数头文件

#include "Foc.h"
#include "LowPassFilter.h"
#include "MotorTask.h"
#include "CmdTask.h"
#include "SafetyTask.h"
#include "ws2812.h" // 包含WS2812驱动头文件
#include "BspDevice.h"
#include "BspTimer.h"
#include "BspUart.h"
#include "BspCan.h"
#include "CmdTask.h"
#include "Log.h"

extern "C" {
#include "pid.h"
#include "MT6701.h"
}

Uart debugUart(DEVICE_USART_1); // 创建UART对象用于调试输出
uint8_t canRxBuf[8] = {0};      // CAN接收缓冲区
bool canReceiveFlag = false;
MITCmd_t mitCmd_Tx;

void GetMitCmd(uint8_t *data, MITCmd_t *mitCmd);

void userMain()
{

    Log::Init(debugUart);

    motor.Init(); // 初始化电机

    motor.SetVelocityTarget(0.0f);
    Log::RegisterData_Vofa("targetSpeed", motor.TargetSpeedPtr());

    xTaskCreate(CmdTask, "CmdTask", 512 * 2, nullptr, osPriorityNormal, &cmdTaskHandle);

    Log::Print("ready\n");

    while (1) 
    {
        Log::Print("%f, %f, %f, %f\n", motor.MotorAngle(), motor.Velocity(), motor.TargetSpeed(), motor.Iq());
        Delay(10);
    }
}
