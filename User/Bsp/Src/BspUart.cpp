#include "BspUart.h"

extern "C" 
{
  // #include "RingBuffer.h"
  // #include "stm32f4xx_hal_uart.h"
  // #include "usart.h"
}


static Uart* uartInstances[DEVICE_USART_END - DEVICE_USART_START] = {nullptr}; // 全局单例指针

static const char* UartInstanceName(const USART_TypeDef* instance)
{
    if (instance == USART1) return "USART1";
    if (instance == USART2) return "USART2";
    if (instance == USART3) return "USART3";
#ifdef UART4
    if (instance == UART4)  return "UART4";
#endif
#ifdef UART5
    if (instance == UART5)  return "UART5";
#endif
#ifdef USART6
    if (instance == USART6) return "USART6";
#endif
#ifdef UART7
    if (instance == UART7)  return "UART7";
#endif
#ifdef UART8
    if (instance == UART8)  return "UART8";
#endif
    return "UNKNOWN";
}

/**
 * @brief Uart类构造函数
 * @param _deviceID 要初始化的UART设备ID (例如 DEVICE_USART_1)
 * @note  - 初始化Tx和Rx环形缓冲区
 *        - 将自身实例注册到全局实例表中
 *        - 显式初始化回调函数指针为nullptr
 */
BspResult<bool> Uart::Init(uint32_t baud)
{
  BSP_CHECK(deviceID >= DEVICE_USART_START && deviceID < DEVICE_USART_END, BspError::InvalidDevice, bool);
  BSP_CHECK(baud > 0, BspError::InvalidParam, bool);

  auto handleResult = Bsp_GetDeviceHandle(deviceID);
  BSP_CHECK(handleResult.ok() && handleResult.value != nullptr, handleResult.error, bool);
  huart = static_cast<UART_HandleTypeDef*>(handleResult.value);

  BSP_CHECK(huart->Instance != nullptr, BspError::InvalidDevice, bool);

  // 初始化环形缓冲区
  // RingBuffer_Init(&ringBuffer_Tx, reinterpret_cast<uint8_t*>(txBuffer), RoundUpToPowerOfTwo(RING_BUFFER_SIZE));
  // RingBuffer_Init(&ringBuffer_Rx, reinterpret_cast<uint8_t*>(rxBuffer), RoundUpToPowerOfTwo(RING_BUFFER_SIZE));

  // 初始化乒乓发送状态
  txBufferCounts[0] = 0;
  txBufferCounts[1] = 0;
  fillIndex = 0;
  txDmaBusyFlag = false;

  userTxCpltCallback = nullptr; 
  userRxCpltCallback = nullptr; 
  
  auto startResult = Bsp_StartDevice(deviceID);
  if (!startResult.ok())
  {
    return startResult;
  }

  auto configResult = ConfigureUart(baud);
  if (!configResult.ok())
  {
    Bsp_StopDevice(deviceID);
    uartInstances[deviceID - DEVICE_USART_START] = nullptr;
    return configResult;
  }

  uartInstances[deviceID - DEVICE_USART_START] = this; // 注册实例指针

  return BspResult<bool>::success(true);
}

BspResult<bool> Uart::ConfigureUart(uint32_t baud)
{
  BSP_CHECK(huart != nullptr, BspError::NullHandle, bool);

  // 进入临界区，确保配置过程不被中断
  __disable_irq();


  HAL_UART_AbortTransmit(huart);
  
  // 停止接收 DMA（如果已启动）
  HAL_UART_AbortReceive(huart);

  // 2. 禁用 UART（避免在重新配置时产生毛刺）
  __HAL_UART_DISABLE(huart);

  // 3. 更新波特率和其他参数（不需要完全 DeInit/Init）
  huart->Init.BaudRate = baud;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;

  // 4. 仅重新初始化 UART（不触发 MSP 层反初始化）
  // 使用 HAL 内部函数直接配置寄存器
  USART_TypeDef *instance = huart->Instance;
  
  // 禁用 UART
  CLEAR_BIT(instance->CR1, USART_CR1_UE);
  
  // 计算并设置波特率寄存器
  uint32_t pclk;
  if (instance == USART1 || instance == USART6)
  {
    pclk = HAL_RCC_GetPCLK2Freq();  // APB2 时钟
  }
  else
  {
    pclk = HAL_RCC_GetPCLK1Freq();  // APB1 时钟
  }
  
  uint32_t usartdiv = (pclk * 25U) / (4U * baud);
  uint32_t divmant = usartdiv / 100U;
  uint32_t divfraq = ((usartdiv - (divmant * 100U)) * 16U + 50U) / 100U;
  instance->BRR = (divmant << 4U) | (divfraq & 0x0FU);
  
  // 配置控制寄存器
  MODIFY_REG(instance->CR1,
             USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8,
             USART_CR1_TE | USART_CR1_RE);  // 8位数据，无校验，使能发送接收
             
  MODIFY_REG(instance->CR2, USART_CR2_STOP, UART_STOPBITS_1);
  MODIFY_REG(instance->CR3, USART_CR3_RTSE | USART_CR3_CTSE, UART_HWCONTROL_NONE);
  
  // 5. 重新使能 UART
  SET_BIT(instance->CR1, USART_CR1_UE);
  
  // 6. 重新使能 IDLE 中断
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  // 7. 禁用 DMA 半传输中断（如果有 RX DMA）
  if (huart->hdmarx != nullptr)
  {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }

  __enable_irq();  // 退出临界区

  return BspResult<bool>::success(true);
}

/**
 * @brief  向发送缓冲区写入数据，使用乒乓缓存机制
 * @param  data 要发送的数据指针
 * @param  size 要发送的数据大小
 * @return BspResult<uint32_t> 操作结果，成功返回实际写入缓冲区的数据大小
 */
BspResult<uint32_t> Uart::SendData(const uint8_t* data, size_t size) 
{
  BSP_CHECK(huart != nullptr, BspError::NullHandle, uint32_t);
  BSP_CHECK(data != nullptr, BspError::InvalidParam, uint32_t);
  BSP_CHECK(size > 0, BspError::InvalidParam, uint32_t);

  size_t originalSize = size;

  while (size > 0)
  {
    __disable_irq();
    uint8_t idx = fillIndex;
    uint16_t count = txBufferCounts[idx];

    // 如果当前缓冲区已满
    if (count >= TX_BUFFER_SIZE)
    {
      if (!txDmaBusyFlag)
      {
        // DMA空闲，立即发送当前满的缓冲区，并切换到另一个
        StartDmaTx(idx);
        __enable_irq();
        continue; // 重新进入循环，此时 fillIndex 已切换，count 应为 0
      }
      else
      {
        // DMA忙，且当前缓冲区已满。必须等待DMA释放另一个缓冲区。
        __enable_irq();
        vTaskDelay(1); // 避免死循环占用CPU
        continue; // 忙等待
      }
    }

    // 计算本次能写入的空间
    uint16_t space = TX_BUFFER_SIZE - count;
    uint16_t chunk = (size < space) ? size : space; // 本次写入的数据块大小

    memcpy(&txBuffers[idx][count], data, chunk);
    txBufferCounts[idx] += chunk;
    data += chunk;
    size -= chunk;

    // 如果填满了缓冲区，且DMA空闲，立即发送
    if (txBufferCounts[idx] >= TX_BUFFER_SIZE)
    {
      if (!txDmaBusyFlag)
      {
        StartDmaTx(idx);
      }
    }

    __enable_irq();
  }

  // 数据全部写入后，如果还有剩余数据未发送且DMA空闲，触发发送
  __disable_irq();
  if (!txDmaBusyFlag && txBufferCounts[fillIndex] > 0)
  {
    StartDmaTx(fillIndex);
  }
  __enable_irq();

  return BspResult<uint32_t>::success(originalSize);
}


/**
 * @brief  启动DMA发送指定索引的缓冲区
 * @param  index 缓冲区索引 (0 或 1)
 * @note   内部函数，必须在临界区内调用
 */
void Uart::StartDmaTx(uint8_t index)
{
  if (txBufferCounts[index] == 0)
  {
    return;
  } 

  txDmaBusyFlag = true;
  
  // 启动DMA传输
  HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, txBuffers[index], txBufferCounts[index]);

  if (status != HAL_OK)
  {
    txDmaBusyFlag = false;
  }
  else
  {
    // 成功启动后，切换填充索引到另一个缓冲区，并重置其计数
    // 这样上层应用可以继续填充新的缓冲区，而DMA在后台发送旧的
    fillIndex = 1 - index;
    txBufferCounts[fillIndex] = 0;
  }
}

/**
 * @brief  DMA发送完成中断回调的核心处理函数
 * @note   由蹦床函数在中断上下文中调用
 *         - 检查当前填充缓冲区是否有数据
 *         - 如果有，立即启动发送
 *         - 如果没有，标记DMA为空闲
 */
void Uart::TxCpltCallback()
{
  // DMA刚刚完成了上一个缓冲区的发送
  // 检查当前正在填充的缓冲区是否有数据需要发送
  // 注意：fillIndex 指向的是当前正在填充（或等待填充）的缓冲区
  // 如果它有数据，说明在DMA发送期间，应用层已经填了一些数据进去
  
  uint8_t idx = fillIndex;
  if (txBufferCounts[idx] > 0)
  {
    // 立即启动发送当前缓冲区
    // StartDmaTx 会切换 fillIndex，所以下一次填充会去用刚才发送完的那个缓冲区
    StartDmaTx(idx);
  }
  else
  {
    // 没有数据要发送，标记为空闲
    txDmaBusyFlag = false;
  }
}

/**
 * @brief  设置用户自定义的发送完成回调函数
 * @param  _userCallback 用户提供的回调函数指针
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Uart::SetTxCallback(UartTxCallback_t _userCallback)
{
  BSP_CHECK(_userCallback != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(deviceID != DEVICE_NONE, BspError::InvalidDevice, bool);

  userTxCpltCallback = _userCallback;
  
  return BspResult<bool>::success(true);
}

/**
 * @brief  格式化打印函数，类似标准printf
 * @param  format 格式化字符串
 * @param  ... 可变参数
 * @return BspResult<bool> 操作结果
 * @note   内部使用一个临时栈缓冲区来格式化字符串，然后调用SendData
 */
void Uart::Printf(const char *format, ...) 
{ 
  char txBufferTemp[128];  
  va_list args;
  va_start(args, format);
  vsnprintf(txBufferTemp, sizeof(txBufferTemp), format, args); // 格式化字符串
  va_end(args);

  SendData((uint8_t *)txBufferTemp, strlen(txBufferTemp));
}

/**
 * @brief  启动UART的DMA接收
 * @return BspResult<bool> 操作结果
 * @note   使用IDLE线空闲检测模式，配合DMA临时缓冲区
 */
BspResult<bool> Uart::EnableRxDMA(bool circular)
{
  BSP_CHECK(huart != nullptr, BspError::NullHandle, bool);

  rxCircularMode = circular;

  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, dmaRxBuffer, DMA_RX_BUFFER_SIZE);
  if (status != HAL_OK)
  {
    return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }

  // 禁用 DMA 半传输中断
  if (huart->hdmarx != nullptr)
  {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }

  return BspResult<bool>::success(true);
}

BspResult<uint32_t> Uart::PushToRxBuffer(uint8_t *data, uint16_t len, size_t offset)
{
  BSP_CHECK(huart != nullptr, BspError::NullHandle, uint32_t);
  BSP_CHECK(data != nullptr, BspError::InvalidParam, uint32_t);

  if (offset >= DMA_RX_BUFFER_SIZE) return BspResult<uint32_t>::failure(BspError::InvalidParam, 0, {__FILE__, __LINE__, __func__});
  if (offset + len > DMA_RX_BUFFER_SIZE) len = DMA_RX_BUFFER_SIZE - offset;

  memcpy(data, &dmaRxBuffer[offset], len);

  return BspResult<uint32_t>::success(len);
}


BspResult<uint32_t> Uart::ReceiveData(uint8_t* data, uint16_t currentDmaPos) 
{
  uint32_t length = 0;

    if (!rxCircularMode)
    {
        // 普通模式：直接从 0 读取 currentDmaPos 长度
        return PushToRxBuffer(data, currentDmaPos, 0);
    }

    // Circular 模式处理
    if (currentDmaPos >= lastDmaRxPos)
    {
        // 情况 1: 指针向后移动，未卷绕
        // [ ... lastRxPos ... NewData ... currentDmaPos ... ]
        length = currentDmaPos - lastDmaRxPos;
        PushToRxBuffer(data, length, lastDmaRxPos);
    }
    else
    {
        // 情况 2: 指针卷绕
        // [ NewData2 ... currentDmaPos ... OldData ... lastRxPos ... NewData1 ... End ]
        
        // 第一段：lastRxPos -> End
        uint16_t len1 = DMA_RX_BUFFER_SIZE - lastDmaRxPos;
        PushToRxBuffer(data, len1, lastDmaRxPos);
        
        // 第二段：0 -> currentDmaPos
        uint16_t len2 = currentDmaPos;
        PushToRxBuffer(data + len1, len2, 0);
        
        length = len1 + len2;
    }

    // 更新位置
    lastDmaRxPos = currentDmaPos;
    
    // 处理边界情况：如果 buffer 刚好填满，HAL 库可能会返回 0 或 Size
    if (lastDmaRxPos == DMA_RX_BUFFER_SIZE) lastDmaRxPos = 0;

    return BspResult<uint32_t>::success(length);
}

/**
 * @brief  DMA接收事件回调的核心处理函数 (IDLE或缓冲区满)
 * @param  size DMA本次接收到的数据长度
 * @note   由蹦床函数在中断上下文中调用
 *         - 将DMA临时缓冲区的数据写入主接收环形缓冲区
 *         - 重新启动下一次DMA空闲中断接收
 */
void Uart::RxEventCallback(uint16_t size)
{

}

/**
 * @brief  处理UART错误并尝试恢复
 * @note   由蹦床函数在中断上下文中调用
 *         - 清除常见错误标志 (ORE, FE, NE)
 *         - 如果接收中断被关闭，尝试重启DMA接收
 */
void Uart::HandleError()
{
  if (huart == nullptr) return;

  // 1. 获取错误代码
  uint32_t errCode = huart->ErrorCode;

  // 2. 检查并清除常见错误标志
  // 这些错误通常会导致接收中断被 HAL 库关闭
  if (errCode & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE | HAL_UART_ERROR_NE))
  {
    // 暴力清除标志位
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    
    // 3. 尝试重启接收
    // 只有当 HAL 库已经把状态置为 READY (说明它已经停止了接收) 时才重启
    // 避免在 DMA 还在跑的时候重复开启
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        // 重新开启 DMA 接收 (Circular 模式)
        // 注意：这里假设我们总是使用 dmaRxBuffer 和 DMA_RX_BUFFER_SIZE
        // 如果你的应用场景有变化，可能需要记录上次使用的 buffer 和 size
        memset(dmaRxBuffer, 0, DMA_RX_BUFFER_SIZE);
        EnableRxDMA(rxCircularMode); 
    }
  }
}

/**
 * @brief  设置用户自定义的接收完成回调函数
 * @param  _userCallback 用户提供的回调函数指针
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Uart::SetRxCallback(UartRxCallback_t _userCallback)
{
  BSP_CHECK(_userCallback != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(deviceID != DEVICE_NONE, BspError::InvalidDevice, bool);

  userRxCpltCallback = _userCallback;
  
  return BspResult<bool>::success(true);
} 


  const char* Uart::GetInfo() const
  {
    if (huart == nullptr) return "Error: Null Handle";

    static char infoBuffer[256];
    const UART_HandleTypeDef* handle = huart;

    snprintf(infoBuffer, sizeof(infoBuffer), 
             "===== %s Info =====\n"
             "deviceID: %d\n"
             "baud: %u\n"
             "Callbacks: Tx=%s, Rx=%s\n"
             "=======================\n",
             UartInstanceName(handle->Instance),
             deviceID, handle->Init.BaudRate,
             userTxCpltCallback ? "SET" : "NULL", 
             userRxCpltCallback ? "SET" : "NULL");

    return infoBuffer;
  }



/**
 * @brief  调用内部核心发送回调和用户自定义发送回调
 * @note   这是一个公共接口，由蹦床函数调用，以保持封装性
 */
void Uart::InvokeTxCallback()
{
  TxCpltCallback();
  if (userTxCpltCallback != nullptr)
  {
    userTxCpltCallback();
  }
}

/**
 * @brief  调用内部核心接收回调和用户自定义接收回调
 * @param  size DMA接收到的数据长度
 * @note   这是一个公共接口，由蹦床函数调用，以保持封装性
 */
void Uart::InvokeRxCallback(uint16_t size)
{
  RxEventCallback(size);
  if (userRxCpltCallback != nullptr)
  {
    userRxCpltCallback(size);
  }

  if (!rxCircularMode)
  {
    // 1. 停止当前的接收 (这会重置 DMA 计数器)
    HAL_UART_AbortReceive(huart);
    
    // 2. 重新启动接收 (使用相同的模式设置)
    // 注意：这里会有极短的盲区，但对于 Log/命令行交互是完全可以接受的
    EnableRxDMA(false);
  }
}



/**
 * @brief  发送完成回调的“蹦床”函数
 * @param  _uartHandle 触发中断的UART句柄 (void*类型)
 * @note   这是一个全局C函数，作为HAL库和C++类成员函数之间的桥梁
 *         - 通过句柄找到对应的Uart实例
 *         - 调用实例的公共回调接口
 */
void Uart_TxCpltCallback_Trampoline(void *_uartHandle)
{
  auto deviceResult = Bsp_FindDeviceByHandle(_uartHandle);
  if (!deviceResult.ok())
  {
    return;
  }

  BspDevice_t deviceID = deviceResult.value;
  // 增加严格的范围检查，确保 deviceID 是一个有效的 UART 设备
  if (deviceID >= DEVICE_USART_START && deviceID < DEVICE_USART_END)
  {
    Uart* instance = uartInstances[deviceID - DEVICE_USART_START];
    if (instance != nullptr)
    {
      instance->InvokeTxCallback();
    }
  }
}

/**
 * @brief  清空接收环形缓冲区
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Uart::ClearRxBuffer()
{
  return BspResult<bool>::success(true);
}

/**
 * @brief  清空发送环形缓冲区
 * @return BspResult<bool> 操作结果
 * @note   这是一个危险操作，会停止正在进行的DMA传输
 */
BspResult<bool> Uart::ClearTxBuffer()
{
  BSP_CHECK(huart != nullptr, BspError::NullHandle, bool);
  
  // 清空发送缓冲区比较危险，需要先停止DMA
  __disable_irq();
  
  HAL_StatusTypeDef status = HAL_UART_DMAStop(huart);
  
  // 重置乒乓缓存状态
  txBufferCounts[0] = 0;
  txBufferCounts[1] = 0;
  fillIndex = 0;
  txDmaBusyFlag = false;
  
  __enable_irq();

  if (status != HAL_OK)
  {
    return BspResult<bool>::failure(BspErrorFromHalStatus(status), false, {__FILE__, __LINE__, __func__});
  }
  
  return BspResult<bool>::success(true);
}

/**
 * @brief  接收事件回调的“蹦床”函数
 * @param  _uartHandle 触发中断的UART句柄 (void*类型)
 * @param  size DMA接收到的数据长度
 * @note   这是一个全局C函数，作为HAL库和C++类成员函数之间的桥梁
 *         - 通过句柄找到对应的Uart实例
 *         - 调用实例的公共回调接口
 */
void Uart_RxEventCallback_Trampoline(void *_uartHandle, uint16_t size)
{
  auto deviceResult = Bsp_FindDeviceByHandle(_uartHandle);
  if (!deviceResult.ok())
  {
    return;
  }

  BspDevice_t deviceID = deviceResult.value;
  // 增加严格的范围检查，确保 deviceID 是一个有效的 UART 设备
  if (deviceID >= DEVICE_USART_START && deviceID < DEVICE_USART_END)
  {
    Uart* instance = uartInstances[deviceID - DEVICE_USART_START];
    if (instance != nullptr)
    {
      instance->InvokeRxCallback(size);
    }
  }
}

/**
 * @brief  错误回调的“蹦床”函数
 * @param  _uartHandle 触发中断的UART句柄 (void*类型)
 */
void Uart_ErrorCallback_Trampoline(void *_uartHandle)
{
  auto deviceResult = Bsp_FindDeviceByHandle(_uartHandle);
  if (!deviceResult.ok())
  {
    return;
  }

  BspDevice_t deviceID = deviceResult.value;
  if (deviceID >= DEVICE_USART_START && deviceID < DEVICE_USART_END)
  {
    Uart* instance = uartInstances[deviceID - DEVICE_USART_START];
    if (instance != nullptr)
    {
      instance->HandleError();
    }
  }
}
