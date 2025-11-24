#include "BspCan.h"

static Can* canInstances[DEVICE_CAN_END - DEVICE_CAN_START] = {nullptr}; // 全局单例指针

/**
 * @brief 计算CAN波特率参数
 * @param baud 目标波特率
 * @param prescaler 输出:预分频值
 * @param bs1 输出:时间段1
 * @param bs2 输出:时间段2
 * @return true=计算成功, false=不支持的波特率
 * 
 * 计算公式: Baud = APB1_CLK / (Prescaler * (1 + BS1 + BS2))
 * STM32F405: APB1 = 42MHz
 * 采样点位置 = (1 + BS1) / (1 + BS1 + BS2) ≈ 87.5%
 */

static bool CalculateCanBaudParams(uint32_t baud, uint32_t& prescaler, uint32_t& bs1, uint32_t& bs2)
{
  struct TimingCandidate
  {
    uint32_t baudRate;
    uint32_t prescaler;
    uint32_t bs1Value;
    uint32_t bs2Value;
  };

 // APB1 = 42MHz 配置表
  // 波特率 = 42MHz / (Prescaler × (1 + BS1 + BS2))
  // 采样点 ≈ (1 + BS1 - SJW) / (1 + BS1 + BS2) -> 16/21 ≈ 76.2%
  static const TimingCandidate candidates_42MHz[] =
  {
    {1000000,  2, CAN_BS1_14TQ, CAN_BS2_6TQ},  // 1Mbps:   42M/(2×21)  = 1M,   采样点约 76.2%
    {500000,   4, CAN_BS1_16TQ, CAN_BS2_3TQ},  // 500kbps: 42M/(4×21)  = 500k, 采样点约 76.2%
    {250000,   8, CAN_BS1_16TQ, CAN_BS2_3TQ},  // 250kbps: 42M/(8×21)  = 250k, 采样点约 76.2%
    {125000,  16, CAN_BS1_16TQ, CAN_BS2_3TQ}   // 125kbps: 42M/(16×21) = 125k, 采样点约 76.2%
  };

  // APB1 = 45MHz 配置表（已实测验证 1Mbps 正常通信）
  // 波特率 = 45MHz / (Prescaler × (1 + BS1 + BS2))
  // 采样点 ≈ (1 + BS1 - SJW) / (1 + BS1 + BS2) -> 11/15 ≈ 73.3%
  static const TimingCandidate candidates_45MHz[] =
  {
    {1000000,  3, CAN_BS1_10TQ, CAN_BS2_4TQ},  // 1Mbps:   45M/(3×15)  = 1M,   采样点约 73.3%
    {500000,   6, CAN_BS1_11TQ, CAN_BS2_2TQ},  // 500kbps: 45M/(6×15)  = 500k, 采样点约 73.3%
    {250000,  12, CAN_BS1_11TQ, CAN_BS2_2TQ},  // 250kbps: 45M/(12×15) = 250k, 采样点约 73.3%
    {125000,  24, CAN_BS1_11TQ, CAN_BS2_2TQ}   // 125kbps: 45M/(24×15) = 125k, 采样点约 73.3%
  };

  // 获取APB1时钟频率
  const uint32_t apb1Clock = static_cast<uint32_t>(CHIP_FREQ_MHZ * 1000000.0f) / 4; // MHz转Hz

  // 选择对应的配置表
  const TimingCandidate* candidates = nullptr;
  size_t candidateCount = 0;
  
  if (apb1Clock == 42000000)
  {
    candidates = candidates_42MHz;
    candidateCount = sizeof(candidates_42MHz) / sizeof(candidates_42MHz[0]);
  }
  else if (apb1Clock == 45000000)
  {
    candidates = candidates_45MHz;
    candidateCount = sizeof(candidates_45MHz) / sizeof(candidates_45MHz[0]);
  }
  else
  {
    return false; // 不支持的APB1时钟
  }

  // 查找匹配的波特率
  for (size_t i = 0; i < candidateCount; i++)
  {
    if (candidates[i].baudRate == baud)
    {
      prescaler = candidates[i].prescaler;
      bs1 = candidates[i].bs1Value;
      bs2 = candidates[i].bs2Value;
      return true;
    }
  }

  return false; // 不支持的波特率
}

BspResult<bool> Can::Init(uint32_t baud, CanMode mode)
{
  // 1. 验证设备ID
  BSP_CHECK(deviceID >= DEVICE_CAN_START && deviceID < DEVICE_CAN_END, BspError::InvalidDevice, bool);
  
  // 2. 验证波特率参数
  uint32_t prescaler, bs1, bs2;
  BSP_CHECK(CalculateCanBaudParams(baud, prescaler, bs1, bs2), BspError::InvalidParam, bool);
  
  // 3. 获取设备句柄7
  auto handleResult = Bsp_GetDeviceHandle(deviceID);
  BSP_CHECK(handleResult.ok() && handleResult.value != nullptr, handleResult.error, bool);
  hcan = static_cast<CAN_HandleTypeDef*>(handleResult.value);
  
  BSP_CHECK(hcan->Instance != nullptr, BspError::InvalidDevice, bool);
  
  // 4. 初始化成员变量
  userRxFifo0Callback = nullptr;
  userRxFifo1Callback = nullptr;
  userTxCallback = nullptr;
  
  // 5. 启动设备(标记为占用)
  auto startResult = Bsp_StartDevice(deviceID);
  if (!startResult.ok())
  {
    return startResult;
  }
  

  hcan->Init.Prescaler = prescaler;
  hcan->Init.Mode = mode;  // 使用传入的工作模式
  hcan->Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan->Init.TimeSeg1 = bs1;
  hcan->Init.TimeSeg2 = bs2;
  hcan->Init.TimeTriggeredMode = DISABLE;
  hcan->Init.AutoBusOff = ENABLE;  // 自动从Bus Off恢复
  hcan->Init.AutoWakeUp = DISABLE;
  hcan->Init.AutoRetransmission = ENABLE; // 自动重传
  hcan->Init.ReceiveFifoLocked = DISABLE; // FIFO满时覆盖旧消息
  hcan->Init.TransmitFifoPriority = DISABLE; // 按ID优先级发送
  
  // 7. 初始化CAN外设
  HAL_StatusTypeDef status = HAL_CAN_Init(hcan);
  if (status != HAL_OK)
  {
    Bsp_StopDevice(deviceID);
    BSP_CHECK(false, BspError::HalError, bool);
  }
  
  // 8. 配置默认滤波器(接受所有消息)
  auto filterResult = ConfigFilterAcceptAll();
  if (!filterResult.ok())
  {
    HAL_CAN_DeInit(hcan);
    Bsp_StopDevice(deviceID);
    return filterResult;
  }
  
  // 9. 保存配置并注册实例
  baudRate = baud;
  canInstances[deviceID - DEVICE_CAN_START] = this;
  
  return BspResult<bool>::success(true);
}

// ==================== 核心功能 ====================

BspResult<bool> Can::Start()
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(deviceID >= DEVICE_CAN_START && deviceID < DEVICE_CAN_END, BspError::InvalidDevice, bool);
  
  // 1. 启动CAN外设
  HAL_StatusTypeDef status = HAL_CAN_Start(hcan);
  BSP_CHECK(status == HAL_OK, BspError::HalError, bool);
  
  // 2. 使能中断
  status = HAL_CAN_ActivateNotification(hcan, 
    CAN_IT_RX_FIFO0_MSG_PENDING |  // FIFO0消息挂起
    CAN_IT_RX_FIFO1_MSG_PENDING |  // FIFO1消息挂起
    CAN_IT_TX_MAILBOX_EMPTY);      // 发送邮箱空
  
  if (status != HAL_OK)
  {
    HAL_CAN_Stop(hcan); // 回滚启动操作
    BSP_CHECK(false, BspError::HalError, bool);
  }
  
  return BspResult<bool>::success(true);
}

BspResult<bool> Can::Stop()
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(deviceID >= DEVICE_CAN_START && deviceID < DEVICE_CAN_END, BspError::InvalidDevice, bool);
  
  // 1. 禁用中断
  HAL_StatusTypeDef status = HAL_CAN_DeactivateNotification(hcan, 
    CAN_IT_RX_FIFO0_MSG_PENDING | 
    CAN_IT_RX_FIFO1_MSG_PENDING | 
    CAN_IT_TX_MAILBOX_EMPTY);
  
  // 2. 停止CAN外设
  status = HAL_CAN_Stop(hcan);
  BSP_CHECK(status == HAL_OK, BspError::HalError, bool);
  
  return BspResult<bool>::success(true);
}

// ==================== 发送功能 ====================

BspResult<bool> Can::SendStdData(uint32_t id, const uint8_t* data, uint8_t len)
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(data != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(len <= 8, BspError::InvalidParam, bool);
  BSP_CHECK(id <= 0x7FF, BspError::InvalidParam, bool); // 标准ID最大11位
  
  // 配置发送头
  CAN_TxHeaderTypeDef txHeader;
  txHeader.StdId = id;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = len;
  txHeader.TransmitGlobalTime = DISABLE;
  
  // 尝试发送
  uint32_t txMailbox;
  HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &txHeader, const_cast<uint8_t*>(data), &txMailbox);
  
  if (status == HAL_OK)
  {
    return BspResult<bool>::success(true);
  }
  
  BSP_CHECK(false, BspError::DeviceBusy, bool);
}

BspResult<bool> Can::SendExtData(uint32_t id, const uint8_t* data, uint8_t len)
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(data != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(len <= 8, BspError::InvalidParam, bool);
  BSP_CHECK(id <= 0x1FFFFFFF, BspError::InvalidParam, bool); // 扩展ID最大29位
  
  // 配置发送头
  CAN_TxHeaderTypeDef txHeader;
  txHeader.ExtId = id;
  txHeader.IDE = CAN_ID_EXT;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = len;
  txHeader.TransmitGlobalTime = DISABLE;
  
  // 尝试发送
  uint32_t txMailbox;
  HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &txHeader, const_cast<uint8_t*>(data), &txMailbox);
  
  if (status == HAL_OK)
  {
    return BspResult<bool>::success(true);
  }
  
  BSP_CHECK(false, BspError::DeviceBusy, bool);
}

BspResult<bool> Can::SendMessage(const CanMessage& msg)
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(msg.len <= 8, BspError::InvalidParam, bool);
  
  // 配置发送头
  CAN_TxHeaderTypeDef txHeader;
  
  if (msg.isExtended)
  {
    txHeader.ExtId = msg.id;
    txHeader.IDE = CAN_ID_EXT;
    BSP_CHECK(msg.id <= 0x1FFFFFFF, BspError::InvalidParam, bool);
  }
  else
  {
    txHeader.StdId = msg.id;
    txHeader.IDE = CAN_ID_STD;
    BSP_CHECK(msg.id <= 0x7FF, BspError::InvalidParam, bool);
  }
  
  txHeader.RTR = msg.isRemote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
  txHeader.DLC = msg.len;
  txHeader.TransmitGlobalTime = DISABLE;
  
  // 尝试发送
  uint32_t txMailbox;
  HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &txHeader, const_cast<uint8_t*>(msg.data), &txMailbox);
  
  if (status == HAL_OK)
  {
    return BspResult<bool>::success(true);
  }
  
  BSP_CHECK(false, BspError::DeviceBusy, bool);
}

BspResult<bool> Can::SendRemoteFrame(uint32_t id, bool isExtended)
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  
  if (isExtended)
  {
    BSP_CHECK(id <= 0x1FFFFFFF, BspError::InvalidParam, bool);
  }
  else
  {
    BSP_CHECK(id <= 0x7FF, BspError::InvalidParam, bool);
  }
  
  // 配置发送头
  CAN_TxHeaderTypeDef txHeader;
  
  if (isExtended)
  {
    txHeader.ExtId = id;
    txHeader.IDE = CAN_ID_EXT;
  }
  else
  {
    txHeader.StdId = id;
    txHeader.IDE = CAN_ID_STD;
  }
  
  txHeader.RTR = CAN_RTR_REMOTE;
  txHeader.DLC = 0;
  txHeader.TransmitGlobalTime = DISABLE;
  
  // 发送远程帧
  uint32_t txMailbox;
  uint8_t dummyData[8] = {0};
  HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &txHeader, dummyData, &txMailbox);
  
  BSP_CHECK(status == HAL_OK, BspError::HalError, bool);
  
  return BspResult<bool>::success(true);
}

// ==================== 滤波器配置 ====================

BspResult<bool> Can::ConfigFilter(const CanFilterConfig& config)
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(config.filterBank <= 27, BspError::InvalidParam, bool);
  
  CAN_FilterTypeDef filter;
  
  filter.FilterBank = config.filterBank;
  filter.FilterMode = config.filterMode;
  filter.FilterScale = config.filterScale;
  filter.FilterFIFOAssignment = config.filterFIFO;
  filter.FilterActivation = config.filterActivation ? CAN_FILTER_ENABLE : CAN_FILTER_DISABLE;
  filter.SlaveStartFilterBank = 14; // 对于单CAN,这个值无关紧要
  
  // 根据是否为扩展帧和滤波器位宽设置ID和掩码
  if (config.filterScale == CAN_FILTERSCALE_32BIT)
  {
    if (config.isExtended)
    {
      // 29位扩展ID
      filter.FilterIdHigh = (config.filterId >> 13) & 0xFFFF;
      filter.FilterIdLow = ((config.filterId << 3) | CAN_ID_EXT) & 0xFFFF;
      filter.FilterMaskIdHigh = (config.filterMask >> 13) & 0xFFFF;
      filter.FilterMaskIdLow = ((config.filterMask << 3) | CAN_ID_EXT) & 0xFFFF;
    }
    else
    {
      // 11位标准ID
      filter.FilterIdHigh = (config.filterId << 5) & 0xFFFF;
      filter.FilterIdLow = 0;
      filter.FilterMaskIdHigh = (config.filterMask << 5) & 0xFFFF;
      filter.FilterMaskIdLow = 0;
    }
  }
  else // CAN_FILTERSCALE_16BIT
  {
    // 16位模式可以配置2个标准ID
    filter.FilterIdHigh = (config.filterId << 5) & 0xFFFF;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = (config.filterMask << 5) & 0xFFFF;
    filter.FilterMaskIdLow = 0;
  }
  
  HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, &filter);
  BSP_CHECK(status == HAL_OK, BspError::HalError, bool);
  
  return BspResult<bool>::success(true);
}

BspResult<bool> Can::ConfigFilterAcceptAll(CanFIFO fifo)
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  
  CAN_FilterTypeDef filter;
  
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000; // 掩码为0表示接受所有ID
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = static_cast<uint32_t>(fifo);
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.SlaveStartFilterBank = 14;
  
  HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, &filter);
  BSP_CHECK(status == HAL_OK, BspError::HalError, bool);
  
  return BspResult<bool>::success(true);
}

BspResult<bool> Can::ConfigFilterStdId(uint32_t id, uint32_t mask, CanFIFO fifo, uint32_t filterBank)
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(id <= 0x7FF, BspError::InvalidParam, bool);
  BSP_CHECK(mask <= 0x7FF, BspError::InvalidParam, bool);
  BSP_CHECK(filterBank <= 27, BspError::InvalidParam, bool);
  
  CAN_FilterTypeDef filter;
  
  filter.FilterBank = filterBank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (id << 5) & 0xFFFF;
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = (mask << 5) & 0xFFFF;
  filter.FilterMaskIdLow = 0;
  filter.FilterFIFOAssignment = static_cast<uint32_t>(fifo);
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.SlaveStartFilterBank = 14;
  
  HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, &filter);
  BSP_CHECK(status == HAL_OK, BspError::HalError, bool);
  
  return BspResult<bool>::success(true);
}

BspResult<bool> Can::ConfigFilterExtId(uint32_t id, uint32_t mask, CanFIFO fifo, uint32_t filterBank)
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(id <= 0x1FFFFFFF, BspError::InvalidParam, bool);
  BSP_CHECK(mask <= 0x1FFFFFFF, BspError::InvalidParam, bool);
  BSP_CHECK(filterBank <= 27, BspError::InvalidParam, bool);
  
  CAN_FilterTypeDef filter;
  
  filter.FilterBank = filterBank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (id >> 13) & 0xFFFF;
  filter.FilterIdLow = ((id << 3) | CAN_ID_EXT) & 0xFFFF;
  filter.FilterMaskIdHigh = (mask >> 13) & 0xFFFF;
  filter.FilterMaskIdLow = ((mask << 3) | CAN_ID_EXT) & 0xFFFF;
  filter.FilterFIFOAssignment = static_cast<uint32_t>(fifo);
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.SlaveStartFilterBank = 14;
  
  HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, &filter);
  BSP_CHECK(status == HAL_OK, BspError::HalError, bool);
  
  return BspResult<bool>::success(true);
}

// ==================== 回调设置 ====================

BspResult<bool> Can::SetRxFifo0Callback(CanRxCallback_t callback)
{
  BSP_CHECK(callback != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(deviceID != DEVICE_NONE, BspError::InvalidDevice, bool);
  
  userRxFifo0Callback = callback;
  
  return BspResult<bool>::success(true);
}

BspResult<bool> Can::SetRxFifo1Callback(CanRxCallback_t callback)
{
  BSP_CHECK(callback != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(deviceID != DEVICE_NONE, BspError::InvalidDevice, bool);
  
  userRxFifo1Callback = callback;
  
  return BspResult<bool>::success(true);
}

BspResult<bool> Can::SetTxCallback(Callback_t callback)
{
  BSP_CHECK(callback != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(deviceID != DEVICE_NONE, BspError::InvalidDevice, bool);
  
  userTxCallback = callback;
  
  return BspResult<bool>::success(true);
}

// ==================== 状态查询 ====================

BspResult<uint32_t> Can::GetBaudRate() const
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, uint32_t);
  
  return BspResult<uint32_t>::success(baudRate);
}

BspResult<uint32_t> Can::GetErrorCount() const
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, uint32_t);
  BSP_CHECK(hcan->Instance != nullptr, BspError::InvalidDevice, uint32_t);
  
  // 读取 ESR 寄存器
  uint32_t esr = hcan->Instance->ESR;
  
  // 提取错误计数器
  uint16_t tec = (esr >> 16) & 0xFF;  // TEC[23:16]
  uint16_t rec = (esr >> 24) & 0xFF;  // REC[31:24]
  
  // 打包返回 (高16位=REC, 低16位=TEC)
  uint32_t result = ((uint32_t)rec << 16) | tec;
  
  return BspResult<uint32_t>::success(result);
}

BspResult<bool> Can::IsErrorPassive() const
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(hcan->Instance != nullptr, BspError::InvalidDevice, bool);
  
  // 读取 ESR 寄存器的 EPVF 位 (Error Passive Flag)
  uint32_t esr = hcan->Instance->ESR;
  bool isPassive = (esr & CAN_ESR_EPVF) != 0;
  
  return BspResult<bool>::success(isPassive);
}

BspResult<bool> Can::IsBusOff() const
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, bool);
  BSP_CHECK(hcan->Instance != nullptr, BspError::InvalidDevice, bool);
  
  // 读取 ESR 寄存器的 BOFF 位 (Bus-Off Flag)
  uint32_t esr = hcan->Instance->ESR;
  bool isBusOff = (esr & CAN_ESR_BOFF) != 0;
  
  return BspResult<bool>::success(isBusOff);
}

BspResult<uint32_t> Can::GetFreeTxMailboxes() const
{
  BSP_CHECK(hcan != nullptr, BspError::NullHandle, uint32_t);
  BSP_CHECK(hcan->Instance != nullptr, BspError::InvalidDevice, uint32_t);
  
  uint32_t freeMailboxes = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
  
  return BspResult<uint32_t>::success(freeMailboxes);
}

// ==================== 内部回调接口 ====================

void Can::InvokeRxFifo0Callback(const CanMessage& msg)
{
  if (userRxFifo0Callback != nullptr)
  {
    userRxFifo0Callback(msg.id, const_cast<uint8_t*>(msg.data), msg.len);
  }
}

void Can::InvokeRxFifo1Callback(const CanMessage& msg)
{
  if (userRxFifo1Callback != nullptr)
  {
    userRxFifo1Callback(msg.id, const_cast<uint8_t*>(msg.data), msg.len);
  }
}

void Can::InvokeTxCallback()
{
  // 调用用户回调
  if (userTxCallback != nullptr)
  {
    userTxCallback();
  }
}

// ==================== 蹦床函数 ====================

void Can_RxFifo0Callback_Trampoline(void *_canHandle)
{
  auto deviceResult = Bsp_FindDeviceByHandle(_canHandle);
  if (!deviceResult.ok())
  {
    return;
  }
  
  BspDevice_t deviceID = deviceResult.value;
  if (deviceID >= DEVICE_CAN_START && deviceID < DEVICE_CAN_END)
  {
    Can* instance = canInstances[deviceID - DEVICE_CAN_START];
    if (instance != nullptr)
    {
      // 读取接收到的消息
      CAN_RxHeaderTypeDef rxHeader;
      uint8_t rxData[8];
      
      CAN_HandleTypeDef* hcan = static_cast<CAN_HandleTypeDef*>(_canHandle);
      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
      {
        // 构建 CanMessage
        CanMessage msg;
        msg.id = (rxHeader.IDE == CAN_ID_EXT) ? rxHeader.ExtId : rxHeader.StdId;
        msg.len = rxHeader.DLC;
        msg.isExtended = (rxHeader.IDE == CAN_ID_EXT);
        msg.isRemote = (rxHeader.RTR == CAN_RTR_REMOTE);
        
        for (uint8_t i = 0; i < msg.len; i++)
        {
          msg.data[i] = rxData[i];
        }
        
        // 调用回调
        instance->InvokeRxFifo0Callback(msg);
      }
    }
  }
}

void Can_RxFifo1Callback_Trampoline(void *_canHandle)
{
  auto deviceResult = Bsp_FindDeviceByHandle(_canHandle);
  if (!deviceResult.ok())
  {
    return;
  }
  
  BspDevice_t deviceID = deviceResult.value;
  if (deviceID >= DEVICE_CAN_START && deviceID < DEVICE_CAN_END)
  {
    Can* instance = canInstances[deviceID - DEVICE_CAN_START];
    if (instance != nullptr)
    {
      // 读取接收到的消息
      CAN_RxHeaderTypeDef rxHeader;
      uint8_t rxData[8];
      
      CAN_HandleTypeDef* hcan = static_cast<CAN_HandleTypeDef*>(_canHandle);
      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK)
      {
        // 构建 CanMessage
        CanMessage msg;
        msg.id = (rxHeader.IDE == CAN_ID_EXT) ? rxHeader.ExtId : rxHeader.StdId;
        msg.len = rxHeader.DLC;
        msg.isExtended = (rxHeader.IDE == CAN_ID_EXT);
        msg.isRemote = (rxHeader.RTR == CAN_RTR_REMOTE);
        
        for (uint8_t i = 0; i < msg.len; i++)
        {
          msg.data[i] = rxData[i];
        }
        
        // 调用回调
        instance->InvokeRxFifo1Callback(msg);
      }
    }
  }
}

void Can_TxMailboxCallback_Trampoline(void *_canHandle, uint32_t mailbox)
{
  auto deviceResult = Bsp_FindDeviceByHandle(_canHandle);
  if (!deviceResult.ok())
  {
    return;
  }
  
  BspDevice_t deviceID = deviceResult.value;
  if (deviceID >= DEVICE_CAN_START && deviceID < DEVICE_CAN_END)
  {
    Can* instance = canInstances[deviceID - DEVICE_CAN_START];
    if (instance != nullptr)
    {
      instance->InvokeTxCallback();
    }
  }
}