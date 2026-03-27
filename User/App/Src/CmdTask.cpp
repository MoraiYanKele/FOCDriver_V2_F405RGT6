#include "CmdTask.h"
#include "usart.h"
#include "MotorTask.h"
#include "SafetyTask.h"
#include "BspCan.h"
#include "Foc.h"
extern "C" 
{
#include "VOFA.h"
}

constexpr uint32_t BASE_MASK = 0x7F8; // 低3位为控制码
constexpr uint32_t CTRL_MASK = 0x007;
constexpr uint32_t REPORT_INTERVAL_MS = 2; // 状态上报周期 2ms，可调整

Can can1(DEVICE_CAN_1);

TaskHandle_t cmdTaskHandle = nullptr;

uint8_t canRxBuffer[8] = {0}; // CAN接收缓冲区
uint32_t canId_Rx = 0; // 接收ID
static MITCmd_t mitCmd_Rx;

static constexpr uint32_t MOTOR_ID = MotorCanId::M2_ID;

uint32_t baseId = 0;
uint32_t ctrlId = 0;


void GetMitCmd(uint8_t* data, MITCmd_t* mitCmd);
void SendStatusCmd();
void AnalyzeCmd(uint32_t ctrlId);
static uint32_t FloatToUint(float x, float x_min, float x_max, uint8_t bits);
static float UintToFloat(uint32_t x_int, float x_min, float x_max, uint8_t bits);


void CanRxCallback(uint32_t canId, uint8_t* data, uint8_t len)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  baseId = canId & BASE_MASK;
  ctrlId = canId & CTRL_MASK;

  if (baseId == MOTOR_ID)
  {

    canId_Rx = canId;
    memcpy(canRxBuffer, data, len);

    vTaskNotifyGiveFromISR(cmdTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/**
 * @brief 命令处理任务
 * @param argument 任务参数
 */
void CmdTask(void *argument)
{
  vTaskDelay(10);
  can1.Init(Can::BAUD_1M, Can::MODE_NORMAL);
  can1.SetRxFifo0Callback(CanRxCallback);
  can1.ConfigFilterStdId(MOTOR_ID, 0x7F8, Can::FIFO_0, 0);
  can1.Start();
  while (1)
  {
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(REPORT_INTERVAL_MS));

    if (ulNotificationValue == 0)
    {
      // 定时发送状态反馈
      SendStatusCmd();
      continue;
    }

    else if (ulNotificationValue > 0)
    {
      AnalyzeCmd(ctrlId);
    }
  }
}
void AnalyzeCmd(uint32_t ctrlId)
{
  switch (ctrlId)
  {
  case MotorCanExtId::TORQUE_CTRL:
  {
    float torqueRx;
    memcpy(&torqueRx, &canRxBuffer[0], sizeof(float));
    motor.SetTorqueTarget(torqueRx);
    break;
  }
  case MotorCanExtId::VELOCITY_CTRL:
  {
    float velocityRx;
    memcpy(&velocityRx, &canRxBuffer[0], sizeof(float));
    motor.SetVelocityTarget(velocityRx);
    break;
  }
  case MotorCanExtId::POSITION_CTRL:
  {
    float positionRx;
    memcpy(&positionRx, &canRxBuffer[0], sizeof(float));
    motor.SetPositionTarget(positionRx);
    break;
  }
  case MotorCanExtId::MIT_CTRL:
    GetMitCmd(canRxBuffer, &mitCmd_Rx);
    motor.SetMitTarget(mitCmd_Rx.position, mitCmd_Rx.velocity,
                       mitCmd_Rx.torque, mitCmd_Rx.kp, mitCmd_Rx.kd);
    break;
  default:
    break;
  }
}

void SendStatusCmd()
{
  float position = motor.AngleSingleTurn();
  float velocity = motor.Velocity();
  float torque   = motor.Iq() * TORQUE_CONST;

  uint16_t velInt    = FloatToUint(velocity, -200, 200, 16);
  uint16_t torqueInt = FloatToUint(torque, -TORQUE_LIMIT * 2, TORQUE_LIMIT * 2, 16);

  uint8_t statusCmd[8];

  memcpy(&statusCmd[0], &position, sizeof(float));
  statusCmd[4] = velInt >> 8;
  statusCmd[5] = velInt & 0xFF;
  statusCmd[6] = torqueInt >> 8;
  statusCmd[7] = torqueInt & 0xFF;

  can1.SendStdData(MOTOR_ID + MotorCanExtId::STATUS_FEEDBACK, statusCmd, 8);
}


void GetMitCmd(uint8_t* data, MITCmd_t* mitCmd)
{
  uint16_t posInt    = (data[0] << 8) | data[1];
  uint16_t velInt    = (data[2] << 4) | (data[3] >> 4);
  uint16_t kpInt     = ((data[3] & 0xF) << 8) | data[4];
  uint16_t kdInt     = (data[5] << 4) | (data[6] >> 4);
  uint16_t torqueInt = ((data[6] & 0xF) << 8) | data[7];

  mitCmd->position = UintToFloat(posInt, MIT_POS_MIN, MIT_POS_MAX, 16);
  mitCmd->velocity = UintToFloat(velInt, MIT_VEL_MIN, MIT_VEL_MAX, 12);
  mitCmd->torque   = UintToFloat(torqueInt, MIT_TORQUE_MIN, MIT_TORQUE_MAX, 12);
  mitCmd->kp       = UintToFloat(kpInt, MIT_KP_MIN, MIT_KP_MAX, 12);
  mitCmd->kd       = UintToFloat(kdInt, MIT_KD_MIN, MIT_KD_MAX, 12);
}


static uint32_t FloatToUint(float x, float x_min, float x_max, uint8_t bits)
{
  if (x_max <= x_min) 
    return 0;
  const uint32_t maxInt = (1u << bits) - 1u;

  float span = x_max - x_min;
  float normalized = (x - x_min) / span; // in [0,1]

  if (normalized < 0.0f) 
    normalized = 0.0f;
  if (normalized > 1.0f) 
    normalized = 1.0f;
  // 四舍五入到 nearest
  uint32_t val = (uint32_t)(normalized * (float)maxInt + 0.5f);

  if (val > maxInt) 
    val = maxInt;

  return val;
}
   
// 安全的 uint->float（解码）
static float UintToFloat(uint32_t x_int, float x_min, float x_max, uint8_t bits)
{
  const uint32_t maxInt = (1u << bits) - 1u;
  if (maxInt == 0) 
    return x_min;
  float span = x_max - x_min;
  return ((float)x_int) * span / (float)maxInt + x_min;
}






