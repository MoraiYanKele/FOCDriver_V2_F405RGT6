#include "LowPassFilter.h"
LPS::~LPS()
{
}

LPS::LPS(float fs, float fc)
{
  float Ts = 1.0f / fs; // 采样周期
  float Rc = 1.0f / (2.0f * _PI * fc); // 截止频率对应的时间常数
  float alpha = Ts / (Rc + Ts); // 计算滤波系数

  // 一阶低通滤波器 H(z) = alpha / (1 - (1-alpha)*z^-1)
  // CMSIS-DSP DF2T 传递函数形式: H(z) = (b0 + b1z^-1 + b2z^-2) / (1 - a1z^-1 - a2z^-2)
  coeffs[0] = alpha;        // b0
  coeffs[1] = 0.0f;         // b1  
  coeffs[2] = 0.0f;         // b2
  coeffs[3] = (1 - alpha);  // a1 (对应传递函数中的 a1)
  coeffs[4] = 0.0f;         // a2 (对应传递函数中的 a2)

  for(int i=0;i<4;i++) state[i]=0;

  arm_biquad_cascade_df2T_init_f32(&filter, 1, coeffs, state);
}

float LPS::LPF_Update(float input)
{
  float output = 0.0f;
  arm_biquad_cascade_df2T_f32(&filter, &input, &output, 1);
  return output;
}