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
extern "C" 
{
  #include "VOFA.h" 
  #include "pid.h"
  #include "MT6701.h"
}


uint8_t canRxBuf[8] = {0}; // CAN接收缓冲区
bool canReceiveFlag = false;
MITCmd_t mitCmd_Tx;


void PrintTask(void *argument);




void GetMitCmd(uint8_t* data, MITCmd_t* mitCmd);




void userMain() 
{

  VOFA_Init();
  
  motor.Init(); // 初始化电机

  // xTaskCreate(PrintTask, "PrintTask", 512, nullptr, osPriorityBelowNormal, nullptr);
  xTaskCreate(CmdTask, "CmdTask", 512 * 2, nullptr, osPriorityNormal, &cmdTaskHandle);

  
  
  Printf("ready\n");
  while (1)
  { 
    Delay(10);
  }
}



void PrintTask(void *argument)
{
  VOFA_RegisterData_float("targetPosition", &motor.targetPosition);
  VOFA_RegisterData_float("targetSpeed", &motor.targetSpeed);
  VOFA_RegisterData_float("targetCurrent", &motor.targetCurrent);
  VOFA_RegisterData_float("kp", &motor.positionPidController.Kp);
  VOFA_RegisterData_float("ki", &motor.positionPidController.Ki);
  VOFA_RegisterData_float("kd", &motor.positionPidController.Kd);
  Printf("PrintTask\n");
  while (1)
  {
    float electricAngle = motor.GetElectricAngle(motor.motorAngle);
    Printf("%f, %f, %f, %f, %f, %f\n", motor.motorAngle, motor.velocity, motor.zeroElectricAngle, motor.targetCurrent, electricAngle, motor.Iq);
    Delay(10);
  }
}


