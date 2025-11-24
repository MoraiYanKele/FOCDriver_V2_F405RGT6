#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "common_inc.h"
#include "BspDevice.h"

#ifdef __cplusplus
extern "C" {
#endif

// CAN 接收消息回调函数类型
typedef void (*CanRxCallback_t)(uint32_t canId, uint8_t* data, uint8_t len);

void Can_RxFifo0Callback_Trampoline(void *_canHandle);
void Can_RxFifo1Callback_Trampoline(void *_canHandle);
void Can_TxMailboxCallback_Trampoline(void *_canHandle, uint32_t mailbox);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief CAN 消息结构体
 */
struct CanMessage
{
  uint32_t id;           // CAN ID（标准帧11位或扩展帧29位）
  uint8_t data[8];       // 数据（最多8字节）
  uint8_t len;           // 数据长度（0-8）
  bool isExtended;       // 是否为扩展帧
  bool isRemote;         // 是否为远程帧
};

/**
 * @brief CAN 滤波器配置结构体
 */
struct CanFilterConfig
{
  uint32_t filterId;           // 滤波器ID
  uint32_t filterMask;         // 滤波器掩码
  uint32_t filterFIFO;         // FIFO分配（0或1）
  uint32_t filterBank;         // 滤波器组号（0-27）
  uint32_t filterMode;         // 滤波器模式（ID掩码模式或ID列表模式）
  uint32_t filterScale;        // 滤波器位宽（16位或32位）
  bool isExtended;             // 是否为扩展帧滤波
  bool filterActivation;       // 是否激活滤波器
};

class Can
{
private:
  CAN_HandleTypeDef* hcan = nullptr;
  BspDevice_t deviceID;
  uint32_t baudRate; // 波特率 (bps)

  CanRxCallback_t userRxFifo0Callback = nullptr;
  CanRxCallback_t userRxFifo1Callback = nullptr;
  Callback_t userTxCallback = nullptr;

public:
  /**
   * @brief CAN 波特率预设枚举
   */
  enum CanBaudRate : uint32_t
  {
    BAUD_125K  = 125000,
    BAUD_250K  = 250000,
    BAUD_500K  = 500000,
    BAUD_1M    = 1000000
  };

  /**
   * @brief CAN FIFO 选择枚举
   */
  enum CanFIFO : uint32_t
  {
    FIFO_0 = CAN_RX_FIFO0,
    FIFO_1 = CAN_RX_FIFO1
  };

  /**
   * @brief CAN 工作模式枚举
   */
  enum CanMode : uint32_t
  {
    MODE_NORMAL    = CAN_MODE_NORMAL,      // 正常模式
    MODE_LOOPBACK  = CAN_MODE_LOOPBACK,    // 环回模式(自测试)
    MODE_SILENT    = CAN_MODE_SILENT,      // 静默模式(只接收)
    MODE_SILENT_LOOPBACK = CAN_MODE_SILENT_LOOPBACK  // 静默环回
  };

  /**
   * @brief 构造函数
   * @param _deviceID CAN设备ID (DEVICE_CAN_1, DEVICE_CAN_2等)
   */
  explicit Can(BspDevice_t _deviceID)
  {
    auto isValidDevice = (_deviceID >= DEVICE_CAN_START && _deviceID < DEVICE_CAN_END);
    if (isValidDevice)
    {
      deviceID = _deviceID;
    }
    else
    {
      deviceID = DEVICE_NONE;
      hcan = nullptr;
      return;
    }
  }

  // ==================== 核心功能 ====================
  
  /**
   * @brief 初始化CAN
   * @param baud 波特率（125K/250K/500K/1M）
   * @param mode 工作模式（正常/环回/静默/静默环回）
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> Init(uint32_t baud = BAUD_1M, CanMode mode = MODE_NORMAL);

  /**
   * @brief 启动CAN
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> Start();

  /**
   * @brief 停止CAN
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> Stop();

  // ==================== 发送功能 ====================
  
  /**
   * @brief 发送CAN消息（标准帧）
   * @param id CAN标准ID（11位）
   * @param data 数据指针
   * @param len 数据长度（0-8）
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> SendStdData(uint32_t id, const uint8_t* data, uint8_t len);

  /**
   * @brief 发送CAN消息（扩展帧）
   * @param id CAN扩展ID（29位）
   * @param data 数据指针
   * @param len 数据长度（0-8）
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> SendExtData(uint32_t id, const uint8_t* data, uint8_t len);

  /**
   * @brief 发送CAN消息（通用）
   * @param msg CAN消息结构体
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> SendMessage(const CanMessage& msg);

  /**
   * @brief 发送远程帧
   * @param id CAN ID
   * @param isExtended 是否为扩展帧
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> SendRemoteFrame(uint32_t id, bool isExtended = false);

  // ==================== 滤波器配置 ====================
  
  /**
   * @brief 配置CAN滤波器
   * @param config 滤波器配置结构体
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> ConfigFilter(const CanFilterConfig& config);

  /**
   * @brief 配置简单滤波器（接受所有消息）
   * @param fifo FIFO选择（FIFO_0或FIFO_1）
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> ConfigFilterAcceptAll(CanFIFO fifo = FIFO_0);

  /**
   * @brief 配置标准ID滤波器
   * @param id 标准ID
   * @param mask 掩码（0表示不关心该位）
   * @param fifo FIFO选择
   * @param filterBank 滤波器组号
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> ConfigFilterStdId(uint32_t id, uint32_t mask, CanFIFO fifo = FIFO_0, uint32_t filterBank = 0);

  /**
   * @brief 配置扩展ID滤波器
   * @param id 扩展ID
   * @param mask 掩码
   * @param fifo FIFO选择
   * @param filterBank 滤波器组号
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> ConfigFilterExtId(uint32_t id, uint32_t mask, CanFIFO fifo = FIFO_0, uint32_t filterBank = 0);

  // ==================== 回调设置 ====================
  
  /**
   * @brief 设置FIFO0接收回调
   * @param callback 回调函数
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> SetRxFifo0Callback(CanRxCallback_t callback);

  /**
   * @brief 设置FIFO1接收回调
   * @param callback 回调函数
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> SetRxFifo1Callback(CanRxCallback_t callback);

  /**
   * @brief 设置发送完成回调
   * @param callback 回调函数
   * @return BspResult<bool> 操作结果
   */
  BspResult<bool> SetTxCallback(Callback_t callback);

  // ==================== 状态查询 ====================
  
  /**
   * @brief 获取当前波特率
   * @return BspResult<uint32_t> 操作结果，成功返回波特率
   */
  BspResult<uint32_t> GetBaudRate() const;

  /**
   * @brief 获取错误计数
   * @return BspResult<uint32_t> 操作结果，成功返回错误计数（高16位:RX错误，低16位:TX错误）
   */
  BspResult<uint32_t> GetErrorCount() const;

  /**
   * @brief 检查是否处于错误被动状态
   * @return BspResult<bool> 操作结果，成功返回true表示错误被动
   */
  BspResult<bool> IsErrorPassive() const;

  /**
   * @brief 检查是否处于总线关闭状态
   * @return BspResult<bool> 操作结果，成功返回true表示总线关闭
   */
  BspResult<bool> IsBusOff() const;

  /**
   * @brief 获取可用的发送邮箱数量
   * @return BspResult<uint32_t> 操作结果，成功返回可用邮箱数（0-3）
   */
  BspResult<uint32_t> GetFreeTxMailboxes() const;

  // ==================== 内部回调接口 ====================
  
  void InvokeRxFifo0Callback(const CanMessage& msg);
  void InvokeRxFifo1Callback(const CanMessage& msg);
  void InvokeTxCallback();
};

#endif // __cplusplus

#endif // __BSP_CAN_H__