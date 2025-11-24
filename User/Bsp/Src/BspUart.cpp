#include "BspUart.h"

extern "C" 
{
  #include "RingBuffer.h"
}


static Uart* uartInstances[DEVICE_USART_END - DEVICE_USART_START] = {nullptr}; // 全局单例指针



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
  RingBuffer_Init(&ringBuffer_Tx, reinterpret_cast<uint8_t*>(txBuffer), RoundUpToPowerOfTwo(RING_BUFFER_SIZE));
  RingBuffer_Init(&ringBuffer_Rx, reinterpret_cast<uint8_t*>(rxBuffer), RoundUpToPowerOfTwo(RING_BUFFER_SIZE));

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

  // 1. 停止所有正在进行的传输
  if (txDmaBusyFlag)
  {
    HAL_UART_AbortTransmit(huart);
    txDmaBusyFlag = false;
  }
  
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
 * @brief  向发送环形缓冲区写入数据，并尝试启动DMA传输
 * @param  data 要发送的数据指针
 * @param  size 要发送的数据大小
 * @return BspResult<uint32_t> 操作结果，成功返回实际写入缓冲区的数据大小
 */
BspResult<uint32_t> Uart::SendData(const uint8_t* data, size_t size) 
{
  BSP_CHECK(huart != nullptr, BspError::NullHandle, uint32_t);
  BSP_CHECK(data != nullptr, BspError::InvalidParam, uint32_t);
  BSP_CHECK(size > 0, BspError::InvalidParam, uint32_t);

  // 将数据写入环形缓冲区
  uint32_t written = RingBuffer_Write(&ringBuffer_Tx, data, size);
  
  if (written > 0) 
  {
    // 尝试启动DMA传输 (如果它当前是空闲的)
    StartDmaTxIfIdle();
  }

  return BspResult<uint32_t>::success(written);
}


/**
 * @brief  如果DMA空闲且发送缓冲区有数据，则启动DMA传输
 * @note   这是一个内部函数，由SendData和TxCpltCallback调用
 *         - 整个函数体位于临界区，防止与中断产生竞态条件
 *         - 计算本次DMA可传输的最大连续数据块长度
 *         - 启动DMA，并处理HAL库启动失败的情况
 */
void Uart::StartDmaTxIfIdle()
{
  __disable_irq(); // --- 临界区开始 ---

  // 检查DMA是否已在忙，或者环形缓冲区是否为空
  if (txDmaBusyFlag || RingBuffer_GetLength(&ringBuffer_Tx) == 0)
  {
    __enable_irq(); // 提前退出时，不要忘记恢复中断
    return;
  }


  txDmaBusyFlag = true;

  uint32_t blockLen;
  if (ringBuffer_Tx.writeIndex > ringBuffer_Tx.readIndex)
  {
    // 数据是单个连续块
    blockLen = ringBuffer_Tx.writeIndex - ringBuffer_Tx.readIndex;
  }
  else
  {
    blockLen = ringBuffer_Tx.size - ringBuffer_Tx.readIndex;
  }

  lastTxDmaSize = blockLen;

  // 启动DMA传输
  HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, ringBuffer_Tx.buffer + ringBuffer_Tx.readIndex, blockLen);

  if (status != HAL_OK)
  {
    txDmaBusyFlag = false;
  }

  __enable_irq(); // --- 临界区结束 ---
}

/**
 * @brief  DMA发送完成中断回调的核心处理函数
 * @note   由蹦床函数在中断上下文中调用
 *         - 更新发送环形缓冲区的读指针
 *         - 清除DMA繁忙标志
 *         - 尝试启动下一次DMA传输（如果缓冲区还有数据）
 */
void Uart::TxCpltCallback()
{
  ringBuffer_Tx.readIndex = (ringBuffer_Tx.readIndex + lastTxDmaSize) & (ringBuffer_Tx.size - 1);

  txDmaBusyFlag = false;

  StartDmaTxIfIdle();
}

/**
 * @brief  设置用户自定义的发送完成回调函数
 * @param  _userCallback 用户提供的回调函数指针
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Uart::SetTxCallback(Callback_t _userCallback)
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
BspResult<bool> Uart::EnableRxDMA()
{
  BSP_CHECK(huart != nullptr, BspError::NullHandle, bool);

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

/**
 * @brief  从接收环形缓冲区读取数据
 * @param  data 用于存放读取数据的用户缓冲区指针
 * @param  size 期望读取的数据大小
 * @return BspResult<uint32_t> 操作结果，成功返回实际从缓冲区读取到的数据大小
 */
BspResult<uint32_t> Uart::ReceiveData(uint8_t* data, size_t size) 
{
  BSP_CHECK(huart != nullptr, BspError::NullHandle, uint32_t);
  BSP_CHECK(data != nullptr, BspError::InvalidParam, uint32_t);
  BSP_CHECK(size > 0, BspError::InvalidParam, uint32_t);

  // 从环形缓冲区读取数据
  uint32_t readLen = RingBuffer_Read(&ringBuffer_Rx, data, size);
  
  return BspResult<uint32_t>::success(readLen);
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
  // 更新环形缓冲区的写指针
  // DMA已经把数据放在了正确的位置，我们只需要告诉环形缓冲区数据写到了哪里
  RingBuffer_Write(&ringBuffer_Rx, dmaRxBuffer, size);

  // 重新启动DMA空闲中断接收，准备接收下一次数据
  HAL_UARTEx_ReceiveToIdle_DMA(huart, dmaRxBuffer, DMA_RX_BUFFER_SIZE);
}

/**
 * @brief  设置用户自定义的接收完成回调函数
 * @param  _userCallback 用户提供的回调函数指针
 * @return BspResult<bool> 操作结果
 */
BspResult<bool> Uart::SetRxCallback(Callback_t _userCallback)
{
  BSP_CHECK(_userCallback != nullptr, BspError::InvalidParam, bool);
  BSP_CHECK(deviceID != DEVICE_NONE, BspError::InvalidDevice, bool);

  userRxCpltCallback = _userCallback;
  
  return BspResult<bool>::success(true);
} 

/**
 * @brief  获取接收缓冲区中待读取的数据长度
 * @return BspResult<uint32_t> 操作结果，成功返回待读取的数据长度
 */
BspResult<uint32_t> Uart::GetRxDataLength() const
{
  uint32_t length = RingBuffer_GetLength(&ringBuffer_Rx);
  return BspResult<uint32_t>::success(length);
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
    userRxCpltCallback();
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
  RingBuffer_Clear(&ringBuffer_Rx);
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
  RingBuffer_Clear(&ringBuffer_Tx);
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
