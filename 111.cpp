
#include "UserMain.h" // 包含用户主函数头文件

#include "Foc.h" 
#include "LowPassFilter.h"

extern "C" 
{
  #include "VOFA.h" 
  #include "pid.h"
  #include "MT6701.h"
}

#define _PI 3.14159265358979323846f // 定义 PI 常量



float uq = 0.0f; // q 轴电压

float rawAngle_MT6701 = 0;
float angle_MT6701 = 0;




uint16_t adcValue[4] = {0};

FOC foc; // 创建 FOC 类实例


float Clarke(float electricalAngle);
void PrintTask(void *argument);

LPS IqFilter(1000.0f, 8.0f);
LPS IdFilter(1000.0f, 8.0f);

LPS IaFilter(1000.0f, 1000.0f);
LPS IbFilter(1000.0f, 1000.0f);
LPS IcFilter(1000.0f, 1000.0f);


float Ia, Ib, Ic; // 三相电流
float vBUS = 0.0f; // VBUS 电压

const float gain_a_correction = 1.0f; 
const float gain_b_correction = 1.3f; // TODO: 此处需要填入B相的实际校准值
const float gain_c_correction = 1.0f; // TODO: 此处需要填入C相的实际校准值

float  currentAOffset = 0; // 电流偏置
float  currentBOffset = 0;
float  currentCOffset = 0;

float electricalAngle;
float velocity = 0.0f; // 速度变量


float Iq = 0;
float Id = 0;

uint32_t adcValueIa = 0;
uint32_t adcValueIb = 0;
uint32_t adcValueIc = 0;
uint32_t adcValueVbus = 0;

void userMain() 
{

  VOFA_Init();
  Printf("ready\n");
  
  // ARM Math PID 控制器实例

  PIDControllerTypedef speedPidController;
  PIDControllerTypedef positionPidController;
  PIDControllerTypedef currentPidController;


  float targetSpeed = 0.0f;
  float targetCurrent = 0.0f;
  float targetPosition = 0.0f; // 目标位置

  VOFA_RegisterData_float("targetCurrent", &targetCurrent);
  VOFA_RegisterData_float("targetPosition", &targetPosition);
  VOFA_RegisterData_float("targetSpeed", &targetSpeed);
  VOFA_RegisterData_float("kp", &currentPidController.Kp);
  VOFA_RegisterData_float("ki", &currentPidController.Ki);
  VOFA_RegisterData_float("kd", &currentPidController.Kd);
  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValue, sizeof(adcValue) / sizeof(uint16_t)); // 启动ADC DMA转换
  HAL_ADCEx_InjectedStart_IT(&hadc1); // 启动注入通道ADC转换
  PID_Init(&speedPidController, 0.68f, 0.9223f, 0.0f, 0.001f, 10.0f, 5.0f, 0.1f, PID_MODE_POSITION);
  PID_Init(&positionPidController, 2.0f, 0.0f, 0.00001f, 0.001f, 100.0f, 500.0f, 0.001f, PID_MODE_POSITION);
  PID_Init(&currentPidController, 3.0f, 0.0f, 0.0f, 0.001f, 10.0f, 500.0f, 0.001f, PID_MODE_POSITION);
  // PID_Init(&currentPidController, 0.5f, 0.0f, 0.0f, 0.001f, 10.0f, 500.0f, 0.001f, PID_MODE_POSITION);

  foc.Init(); // 初始化 FOC

  foc.SetPhaseVoltage(0, 0, 0); 
  vTaskDelay(pdMS_TO_TICKS(500)); // 等待硬件稳定

  xTaskCreate(PrintTask, "PrintTask", 256, NULL, 1, NULL); // 创建打印任务

  long long tempAOffset = 0;
  long long tempBOffset = 0;
  long long tempCOffset = 0;
  for (int i  = 0; i < 2000; i++)
  {
    tempAOffset += adcValueIa; // A相不再使用，无需校准
    tempBOffset += adcValueIb; // 累加 IB 电流
    tempCOffset += adcValueIc; // 累加 IC 电流
    vTaskDelay(pdMS_TO_TICKS(1)); // 延时 1 毫秒
  }
  currentAOffset = (float)tempAOffset / 2000.0f; 
  currentBOffset = (float)tempBOffset / 2000.0f;
  currentCOffset = (float)tempCOffset / 2000.0f; 
  Printf("Calibration done: B=%.2f, C=%.2f\n", currentBOffset, currentCOffset);

  // 【电角度校准步骤1】：施加固定电压，将转子锁定到已知位置
  // 使用 ud=3V, uq=0, electricalAngle=0 来产生沿Alpha轴的固定磁场
  foc.SetPhaseVoltage(0.0f, 8.0f, 0.0f); // uq=0, ud=3V, angle=0
  vTaskDelay(pdMS_TO_TICKS(2000));  // 等待转子稳定到电角度0位置
  
  // 【电角度校准步骤2】：读取此时的机械角度
  float angle_sum = 0;
  int samples = 50;
  for (int i=0; i<samples; i++) {
      angle_sum += GetAngle(angleRead());
      vTaskDelay(pdMS_TO_TICKS(10)); // 短暂延时
  }
  float mechanicalAngle_locked = angle_sum / samples;
  
  
  // 【电角度校准步骤3】：计算零点偏移
  // 在当前位置，电角度应该为0，所以：0 = polePairs * mechanicalAngle - zeroElectricAngle
  // 因此：zeroElectricAngle = polePairs * mechanicalAngle
  foc.zeroElectricAngle = foc.GetElectricAngle(mechanicalAngle_locked); // 设置零电角度
  // foc.zeroElectricAngle = mechanicalAngle_locked;

  Printf("%f\n", foc.zeroElectricAngle);
  
  foc.SetPhaseVoltage(0.0f, 0.0f, 0.0f); // 关闭所有电压

  // foc.SetPhaseVoltage(0.0f, 3.0f, 0); // 设置相电压
  // vTaskDelay(pdMS_TO_TICKS(3000));  
  //  float mechanicalAngle_locked = GetAngle(angleRead()); 

  // // 计算零点偏移。FOC类的 polePairs 变量需要被正确设置。
  // // 这个偏移量将用于后续将机械角度转换为正确的电角度。
  // foc.zeroElectricAngle = foc.NormalizeAngle(foc.polePairs * mechanicalAngle_locked);
  // foc.SetPhaseVoltage(0.0f, 0.0f, 0); // 设置相电压为零

  vTaskDelay(pdMS_TO_TICKS(500));
  // foc.zeroElectricAngle = GetAngle(angleRead()); // 设置零电角度
 



  while (1)
  {
    rawAngle_MT6701 = angleRead(); // 读取角度
    angle_MT6701 = GetAngle(rawAngle_MT6701);

    velocity = foc.GetVelocity(angle_MT6701); // 计算速度

    electricalAngle = foc.GetElectricAngle(angle_MT6701);

    // 纯速度环
    // targetSpeed = PIDCompute(&positionPidController, angle_MT6701, targetPosition); // 更新位置 PID 控制器
    uq = PIDCompute(&speedPidController, velocity, targetSpeed); // 更新 PID 控制器




    // 纯电流环
    Clarke(electricalAngle);
    // uq = PIDCompute(&currentPidController, Iq, targetCurrent); // 计算电流 PID 控制器输出
    
    
    foc.SetPhaseVoltage(uq, 0.0f, electricalAngle); // 设置相电压

    // Printf("%f, %f\n", angle_MT6701, targetPosition);


    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
void PrintTask(void *argument)
{
  while (1)
  {

    float iaWithFilter = IaFilter.LPF_Update(Ia); // 低通滤波处理 IA 电流
    
    float ibWithFilter = IbFilter.LPF_Update(Ib); // 低通滤
    float icWithFilter = IcFilter.LPF_Update(Ic); // 低通滤波处理 IC 电流
    // float iaWithFilter = - ibWithFilter - icWithFilter; // 根据基尔霍夫定律计算 IA 电流
    float iSumWithFilter = iaWithFilter + ibWithFilter + icWithFilter; // 三相电流之和

    float iSum = Ia + Ib + Ic; // 三相电流之和
    // Printf("%f\n", uq);
    // Printf("%f, %f, %f, %f\n", Iq, uq, velocity, ta); // 打印 Iq 和三相电流
    Printf("%f, %f, %f, %f, %f, %f\n", Iq, uq, Ia, Ib, Ic, velocity); // 打印 Iq 和三相电流
    // Printf("%f, %f\n", foc.GetElectricAngle(angle_MT6701), foc.zeroElectricAngle);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

float Clarke(float electricalAngle)
{
  float Ialpha = -1 * (Ib + Ic);
  float Ibeta = (Ib - Ic) * 0.57735026919f;

  Iq = Ibeta * arm_cos_f32(electricalAngle) - Ialpha * arm_sin_f32(electricalAngle); 
  Id = Ialpha * arm_cos_f32(electricalAngle) + Ibeta * arm_sin_f32(electricalAngle);

  Iq = IqFilter.LPF_Update(Iq); // 低通滤波处理 Iq
  Id = IdFilter.LPF_Update(Id); // 低通滤波处理 Id
  
  Iq = -Iq;

  return Iq; 
}


// float GetVBUS()
// {
//   return (float)adcValue[3] * (3.3f / 4096.0f) * 11; // 根据 ADC 分辨率和参考电压计算 VBUS
// }

extern  "C"
{
  void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
  {
    if (hadc == &hadc1) // 检查是否是 ADC1
    {
      adcValueIa = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); // 获取 IA 电流
      adcValueIb = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
      adcValueIc = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
      adcValueVbus = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4); // 获取 VBUS 电压

      Ia = (float)(adcValueIa - currentAOffset) * (3.3f / 4096.0f) * 4 * gain_a_correction;
      Ib = (float)(adcValueIb - currentBOffset) * (3.3f / 4096.0f) * 4 * gain_b_correction;
      Ic = (float)(adcValueIc - currentCOffset) * (3.3f / 4096.0f) * 4 * gain_c_correction;
      vBUS = (float)adcValueVbus * (3.3f / 4096.0f) * 11; // 根据 ADC 分辨率和参考电压计算 VBUS
      // Ia = IaFilter.LPF_Update(Ia); // 低通滤波处理 IA 电流
      // Ib = IbFilter.LPF_Update(Ib); // 低通滤波处理 IB
      // Ic = IcFilter.LPF_Update(Ic); // 低通滤波处理 IC 电流
    }
  }
}
