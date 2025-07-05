#ifndef __LOW_PASS_FILTER_HPP__
#define __LOW_PASS_FILTER_HPP__

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/
#ifdef __cplusplus

#define _PI 3.14159265358979323846f // 定义 PI 常量

class LPS
{
private:
  arm_biquad_cascade_df2T_instance_f32 filter;
  float coeffs[5];   // biquad系数
  float state[4];    // 状态变量
public:
  LPS(float fs, float fc);
  ~LPS();
  

  float LPF_Update(float input); // 更新滤波器
};

#endif

#endif