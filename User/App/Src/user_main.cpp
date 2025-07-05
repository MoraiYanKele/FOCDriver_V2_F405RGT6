
#include "UserMain.h" // 包含用户主函数头文件

#include "Foc.h" 
#include "LowPassFilter.h"
#include "MotorTask.h"
#include "CmdTask.h"

extern "C" 
{
  #include "VOFA.h" 
  #include "pid.h"
  #include "MT6701.h"
}





float uq = 0.0f; // q 轴电压

float rawAngle_MT6701 = 0;
float angle_MT6701 = 0;




float Clarke(float electricalAngle);
void PrintTask(void *argument);

// LPS IqFilter(1000.0f, 8.0f);
// LPS IdFilter(1000.0f, 8.0f);

// LPS IaFilter(1000.0f, 1000.0f);
// LPS IbFilter(1000.0f, 1000.0f);
// LPS IcFilter(1000.0f, 1000.0f);


// float  currentAOffset = 0; // 电流偏置
// float  currentBOffset = 0;
// float  currentCOffset = 0;


// float Iq = 0;
// float Id = 0;

// uint32_t adcValueIa = 0;
// uint32_t adcValueIb = 0;
// uint32_t adcValueIc = 0;
// uint32_t adcValueVbus = 0;

void userMain() 
{




  // HAL_ADCEx_InjectedStart_IT(&hadc1); // 启动注入通道ADC转换



  xTaskCreate(PrintTask, "PrintTask", 2048, NULL, osPriorityNormal, NULL); // 创建打印任务
  xTaskCreate(MotorTask, "MotorTask", 256, NULL, osPriorityHigh, &MotorTaskHandle); // 创建电机任务
  motorMode = CONTROL_MODE_POSITION; // 设置电机控制模式为 NONE
  // targetSpeed = 40.0f; // 设置目标速度
  // xTaskCreate(CmdTask, "CmdTask", 2048, NULL, 1, &CmdTaskHandle); // 创建命令任务
  // long long tempAOffset = 0;
  // long long tempBOffset = 0;
  // long long tempCOffset = 0;
  // for (int i  = 0; i < 2000; i++)
  // {
  //   tempAOffset += adcValueIa; // A相不再使用，无需校准
  //   tempBOffset += adcValueIb; // 累加 IB 电流
  //   tempCOffset += adcValueIc; // 累加 IC 电流
  //   vTaskDelay(pdMS_TO_TICKS(1)); // 延时 1 毫秒
  // }
  // currentAOffset = (float)tempAOffset / 2000.0f; 
  // currentBOffset = (float)tempBOffset / 2000.0f;
  // currentCOffset = (float)tempCOffset / 2000.0f; 


  vTaskDelay(pdMS_TO_TICKS(500));

 



  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
void PrintTask(void *argument)
{

  VOFA_Init();
  vTaskDelay(pdMS_TO_TICKS(1000)); // 等待 VOFA 初始化完成
  VOFA_RegisterData_float("targetPosition", &targetPosition);
  VOFA_RegisterData_float("targetSpeed", &targetSpeed);
  VOFA_RegisterData_float("kp", &motor.positionPidController.Kp);
  VOFA_RegisterData_float("ki", &motor.positionPidController.Ki);
  VOFA_RegisterData_float("kd", &motor.positionPidController.Kd);

  while (1)
  {
    float angle = motor.GetMechanicalAngle(); // 获取机械角度
    float velocity = motor.GetVelocity(angle); // 获取电机速度
    Printf("%f, %f\n", angle, targetPosition);
    // Printf("%f, %f, %f\n", angle, targetSpeed, velocity);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// float Clarke(float electricalAngle)
// {
//   float Ialpha = -1 * (Ib + Ic);
//   float Ibeta = (Ib - Ic) * 0.57735026919f;

//   Iq = Ibeta * arm_cos_f32(electricalAngle) - Ialpha * arm_sin_f32(electricalAngle); 
//   Id = Ialpha * arm_cos_f32(electricalAngle) + Ibeta * arm_sin_f32(electricalAngle);

//   Iq = IqFilter.LPF_Update(Iq); // 低通滤波处理 Iq
//   Id = IdFilter.LPF_Update(Id); // 低通滤波处理 Id
  
//   Iq = -Iq;

//   return Iq; 
// }


// float GetVBUS()
// {
//   return (float)adcValue[3] * (3.3f / 4096.0f) * 11; // 根据 ADC 分辨率和参考电压计算 VBUS
// }

// extern  "C"
// {
//   void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
//   {
//     if (hadc == &hadc1) // 检查是否是 ADC1
//     {
//       adcValueIa = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); // 获取 IA 电流
//       adcValueIb = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
//       adcValueIc = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
//       adcValueVbus = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4); // 获取 VBUS 电压

//       Ia = (float)(adcValueIa - currentAOffset) * (3.3f / 4096.0f) * 4 * gain_a_correction;
//       Ib = (float)(adcValueIb - currentBOffset) * (3.3f / 4096.0f) * 4 * gain_b_correction;
//       Ic = (float)(adcValueIc - currentCOffset) * (3.3f / 4096.0f) * 4 * gain_c_correction;
//       vBUS = (float)adcValueVbus * (3.3f / 4096.0f) * 11; // 根据 ADC 分辨率和参考电压计算 VBUS
//       // Ia = IaFilter.LPF_Update(Ia); // 低通滤波处理 IA 电流
//       // Ib = IbFilter.LPF_Update(Ib); // 低通滤波处理 IB
//       // Ic = IcFilter.LPF_Update(Ic); // 低通滤波处理 IC 电流
//     }
//   }
// }
