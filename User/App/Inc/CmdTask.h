#ifndef __CMD_TASK_H__
#define __CMD_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif





#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

#include "common_inc.h" // 包含必要的头文件
#include "Foc.h"

constexpr static float MIT_POS_MAX = PI;  // 位置最大值限制(rad)
constexpr static float MIT_POS_MIN = -PI; // 位置最小值限制(rad)
constexpr static float MIT_VEL_MAX = SPEED_LIMIT / 2;  // 速度最大值限制(rad/s)
constexpr static float MIT_VEL_MIN = -SPEED_LIMIT / 2;  // 速度最小值限制(rad/s)
constexpr static float MIT_TORQUE_MAX = TORQUE_LIMIT; // 力矩最大值限制(Nm)
constexpr static float MIT_TORQUE_MIN = -TORQUE_LIMIT; // 力矩最小值限制(Nm)
constexpr static float MIT_KP_MAX = 10.0f; // kp 最大值限制
constexpr static float MIT_KP_MIN = 0.0f;    // kp 最小
constexpr static float MIT_KD_MAX = 0.1f; // kd 最大值限制
constexpr static float MIT_KD_MIN = 0.0f;    // kd 最小

constexpr static uint8_t MIT_POS_BITS = 16;   // 位置位数
constexpr static uint8_t MIT_VEL_BITS = 12;   // 速度位数
constexpr static uint8_t MIT_TORQUE_BITS = 12; // 力矩位数
constexpr static uint8_t MIT_KP_BITS = 12;     // kp 位数
constexpr static uint8_t MIT_KD_BITS = 12;     // kd 位数

enum MotorCanId : uint32_t
{
  M1_ID = 0x100, 
  M2_ID = 0x200, 
  M3_ID = 0x300, 
  M4_ID = 0x400,
};

enum MotorCanExtId : uint32_t
{
  TORQUE_CTRL     = 0x1,
  VELOCITY_CTRL   = 0x2,
  POSITION_CTRL   = 0x3,
  MIT_CTRL        = 0x4,
  STATUS_FEEDBACK = 0x5,
};



struct MITCmd_t
{
  float position;
  float velocity;
  float torque;
  float kp;
  float kd;
};

extern TaskHandle_t cmdTaskHandle;
extern "C" 
{

  void CmdTask(void *argument);

}
#endif // __cplusplus


#endif // __CMD_TASK_H__
