#include "Callback.h"
#include "VOFA.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == VOFA_UART) // 检查是否是 VOFA UART
  {
    VOFA_RxCallBack(); // 调用 VOFA 接收回调函数
  }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) 
{
  if (huart == VOFA_UART) 
  {
    TxCallBack_DoubleBufferUartDMA(&uartToVOFA);
  }
}

// void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
// {
//   if (hadc == &hadc1) // 检查是否是 ADC1
//   {
//     adcValueIa = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); // 获取 IA 电流
//     adcValueIb = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
//     adcValueIc = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
//     adcValueVbus = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4); // 获取 VBUS 电压

//     Ia = (float)(adcValueIa - currentAOffset) * (3.3f / 4096.0f) * 4 * gain_a_correction;
//     Ib = (float)(adcValueIb - currentBOffset) * (3.3f / 4096.0f) * 4 * gain_b_correction;
//     Ic = (float)(adcValueIc - currentCOffset) * (3.3f / 4096.0f) * 4 * gain_c_correction;
//     vBUS = (float)adcValueVbus * (3.3f / 4096.0f) * 11; // 根据 ADC 分辨率和参考电压计算 VBUS
//     // Ia = IaFilter.LPF_Update(Ia); // 低通滤波处理 IA 电流
//     // Ib = IbFilter.LPF_Update(Ib); // 低通滤波处理 IB
//     // Ic = IcFilter.LPF_Update(Ic); // 低通滤波处理 IC 电流
//   }
// }
