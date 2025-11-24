# BspCan 类设计说明

## 📋 目录
1. [核心功能概述](#核心功能概述)
2. [类成员详解](#类成员详解)
3. [功能分类](#功能分类)
4. [使用场景](#使用场景)
5. [实现要点](#实现要点)

---

## 核心功能概述

`BspCan` 类提供了完整的 CAN 总线通信功能，包括：

### ✅ 已包含的核心功能

#### 1. **初始化与控制**
- `Init(baudRate)` - 初始化CAN，配置波特率
- `Start()` - 启动CAN通信
- `Stop()` - 停止CAN通信

#### 2. **数据发送**
- `SendStdData()` - 发送标准帧数据（11位ID）
- `SendExtData()` - 发送扩展帧数据（29位ID）
- `SendMessage()` - 发送通用消息（支持所有格式）
- `SendRemoteFrame()` - 发送远程帧（请求数据）

#### 3. **滤波器配置**
- `ConfigFilter()` - 高级滤波器配置
- `ConfigFilterAcceptAll()` - 配置接受所有消息
- `ConfigFilterStdId()` - 标准ID滤波器
- `ConfigFilterExtId()` - 扩展ID滤波器

#### 4. **接收回调**
- `SetRxFifo0Callback()` - 设置FIFO0接收回调
- `SetRxFifo1Callback()` - 设置FIFO1接收回调
- `SetTxCallback()` - 设置发送完成回调

#### 5. **状态查询**
- `GetBaudRate()` - 获取当前波特率
- `GetErrorCount()` - 获取错误计数
- `IsErrorPassive()` - 检查错误被动状态
- `IsBusOff()` - 检查总线关闭状态
- `GetFreeTxMailboxes()` - 获取可用发送邮箱数

---

## 类成员详解

### 📦 数据结构

#### **CanMessage** - CAN消息结构体
```cpp
struct CanMessage
{
  uint32_t id;           // CAN ID（标准帧11位或扩展帧29位）
  uint8_t data[8];       // 数据（最多8字节）
  uint8_t len;           // 数据长度（0-8）
  bool isExtended;       // 是否为扩展帧
  bool isRemote;         // 是否为远程帧
};
```
**用途**: 封装CAN消息的所有信息，简化消息传递

#### **CanFilterConfig** - 滤波器配置结构体
```cpp
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
```
**用途**: 提供灵活的滤波器配置选项

### 🔧 私有成员

#### 硬件接口
```cpp
CAN_HandleTypeDef* hcan = nullptr;  // HAL库CAN句柄
BspDevice_t deviceID;               // 设备ID
uint32_t baudRate;                  // 波特率
```

#### 回调函数
```cpp
CanRxCallback_t userRxFifo0Callback = nullptr;  // FIFO0接收回调
CanRxCallback_t userRxFifo1Callback = nullptr;  // FIFO1接收回调
Callback_t userTxCallback = nullptr;            // 发送完成回调
```

**回调函数签名**:
```cpp
typedef void (*CanRxCallback_t)(uint32_t canId, uint8_t* data, uint8_t len);
```

#### 发送队列（可选特性）
```cpp
static const uint32_t TX_BUFFER_SIZE = 16;
CanMessage txBuffer[TX_BUFFER_SIZE];
volatile uint32_t txWriteIndex = 0;
volatile uint32_t txReadIndex = 0;
```

**用途**: 
- 当3个硬件邮箱都忙时，消息可以暂存在队列中
- 发送完成中断会自动从队列取消息继续发送
- 实现无阻塞的连续发送

### 📊 枚举类型

#### **CanBaudRate** - 波特率预设
```cpp
enum CanBaudRate : uint32_t
{
  BAUD_125K  = 125000,   // 125 kbps（常用于汽车CAN）
  BAUD_250K  = 250000,   // 250 kbps
  BAUD_500K  = 500000,   // 500 kbps（常用）
  BAUD_1M    = 1000000   // 1 Mbps（最高速度）
};
```

#### **CanFIFO** - FIFO选择
```cpp
enum CanFIFO : uint32_t
{
  FIFO_0 = CAN_RX_FIFO0,
  FIFO_1 = CAN_RX_FIFO1
};
```

---

## 功能分类

### 🚀 1. 初始化流程

```cpp
// 典型初始化流程
Can can1(DEVICE_CAN_1);
can1.Init(Can::BAUD_500K);          // 初始化为500K波特率
can1.ConfigFilterAcceptAll();       // 接受所有消息
can1.SetRxFifo0Callback(myCallback);// 设置接收回调
can1.Start();                       // 启动CAN
```

**Init() 需要实现的功能**:
1. 获取设备句柄 (`Bsp_GetDeviceHandle`)
2. 配置波特率参数（BSP1/BSP2/SJW/时间段）
3. 设置CAN工作模式（正常模式/环回模式/静默模式）
4. 配置中断（RX FIFO0/1中断，TX中断）
5. 调用 `HAL_CAN_Init()`

**波特率计算公式**:
```
CAN_Baudrate = APB1_Clock / (Prescaler * (1 + BS1 + BS2))
```

### 📤 2. 发送功能

#### **标准帧发送**
```cpp
uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
can1.SendStdData(0x123, data, 4);
```

**实现要点**:
- 检查可用的发送邮箱
- 如果邮箱忙，可以选择：
  - 立即返回错误
  - 加入发送队列（推荐）
- 配置TxHeader（IDE/RTR/DLC）
- 调用 `HAL_CAN_AddTxMessage()`

#### **扩展帧发送**
```cpp
can1.SendExtData(0x12345678, data, 4);
```

#### **远程帧发送**
```cpp
can1.SendRemoteFrame(0x123);  // 请求ID为0x123的数据
```

**实现要点**:
- 设置 `RTR = CAN_RTR_REMOTE`
- DLC 指定期望的数据长度
- 不需要发送实际数据

### 📥 3. 接收与滤波

#### **滤波器工作原理**

CAN滤波器有两种模式：
1. **掩码模式**（Mask Mode）
   - `FilterId`: 期望的ID
   - `FilterMask`: 掩码（1=必须匹配，0=不关心）
   - 例如：ID=0x100, Mask=0x700 → 接受0x100-0x1FF

2. **列表模式**（List Mode）
   - 直接列出允许的ID
   - 每个滤波器可配置2个或4个ID

#### **接受所有消息**
```cpp
can1.ConfigFilterAcceptAll(Can::FIFO_0);
```

**实现**:
```cpp
CAN_FilterTypeDef filter;
filter.FilterIdHigh = 0;
filter.FilterIdLow = 0;
filter.FilterMaskIdHigh = 0;
filter.FilterMaskIdLow = 0;
filter.FilterFIFOAssignment = fifo;
filter.FilterBank = 0;
filter.FilterMode = CAN_FILTERMODE_IDMASK;
filter.FilterScale = CAN_FILTERSCALE_32BIT;
filter.FilterActivation = CAN_FILTER_ENABLE;
HAL_CAN_ConfigFilter(hcan, &filter);
```

#### **只接受特定ID**
```cpp
// 只接受ID为0x123的标准帧
can1.ConfigFilterStdId(0x123, 0x7FF, Can::FIFO_0);
```

**掩码示例**:
- `0x7FF` - 完全匹配（11位标准帧）
- `0x700` - 只匹配高3位
- `0x0FF` - 只匹配低8位

#### **接收回调**
```cpp
void MyRxCallback(uint32_t canId, uint8_t* data, uint8_t len)
{
  // 处理接收到的数据
  if (canId == 0x123) {
    // 处理特定ID的消息
  }
}

can1.SetRxFifo0Callback(MyRxCallback);
```

### 🔍 4. 状态监控

#### **错误计数**
```cpp
auto result = can1.GetErrorCount();
if (result.ok()) {
  uint16_t rxError = (result.value >> 16) & 0xFFFF;
  uint16_t txError = result.value & 0xFFFF;
}
```

**实现**:
```cpp
uint32_t esr = hcan->Instance->ESR;
uint32_t rxErr = (esr >> 24) & 0xFF;
uint32_t txErr = (esr >> 16) & 0xFF;
return (rxErr << 16) | txErr;
```

#### **总线状态检测**
```cpp
// 检查总线是否关闭
if (can1.IsBusOff().value) {
  // 总线故障，需要重新初始化
  can1.Stop();
  can1.Start();
}

// 检查是否进入错误被动状态
if (can1.IsErrorPassive().value) {
  // 错误累积过多，需要注意
}
```

**CAN错误状态机**:
- **Error Active**: 正常工作，错误计数 < 128
- **Error Passive**: 错误计数 128-255，只能被动发送
- **Bus Off**: 错误计数 > 255，停止通信

### 🔄 5. 中断处理流程

#### **蹦床函数**
```cpp
void Can_RxFifo0Callback_Trampoline(void *_canHandle);
void Can_RxFifo1Callback_Trampoline(void *_canHandle);
void Can_TxMailboxCallback_Trampoline(void *_canHandle, uint32_t mailbox);
```

**实现流程**:
1. HAL库中断 → 蹦床函数
2. 通过句柄查找Can实例
3. 调用实例的Invoke方法
4. Invoke方法调用用户回调

**示例**:
```cpp
void Can_RxFifo0Callback_Trampoline(void *_canHandle)
{
  auto deviceResult = Bsp_FindDeviceByHandle(_canHandle);
  if (!deviceResult.ok()) return;
  
  BspDevice_t deviceID = deviceResult.value;
  if (deviceID >= DEVICE_CAN_START && deviceID < DEVICE_CAN_END) {
    Can* instance = canInstances[deviceID - DEVICE_CAN_START];
    if (instance != nullptr) {
      // 从FIFO读取消息
      CAN_RxHeaderTypeDef rxHeader;
      uint8_t rxData[8];
      HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
      
      // 构造消息
      CanMessage msg;
      msg.id = (rxHeader.IDE == CAN_ID_EXT) ? rxHeader.ExtId : rxHeader.StdId;
      msg.len = rxHeader.DLC;
      msg.isExtended = (rxHeader.IDE == CAN_ID_EXT);
      msg.isRemote = (rxHeader.RTR == CAN_RTR_REMOTE);
      memcpy(msg.data, rxData, msg.len);
      
      instance->InvokeRxFifo0Callback(msg);
    }
  }
}
```

---

## 使用场景

### 🚗 场景1: 汽车CAN网络（多ECU通信）
```cpp
Can canVehicle(DEVICE_CAN_1);
canVehicle.Init(Can::BAUD_500K);

// 配置滤波器只接受引擎控制单元的消息（ID范围：0x100-0x1FF）
canVehicle.ConfigFilterStdId(0x100, 0x700, Can::FIFO_0);

// 设置接收回调
canVehicle.SetRxFifo0Callback([](uint32_t id, uint8_t* data, uint8_t len) {
  switch (id) {
    case 0x100: // 引擎转速
      uint16_t rpm = (data[0] << 8) | data[1];
      break;
    case 0x110: // 引擎温度
      uint8_t temp = data[0];
      break;
  }
});

canVehicle.Start();

// 发送踏板位置
uint8_t throttle[] = {75}; // 75%
canVehicle.SendStdData(0x200, throttle, 1);
```

### 🤖 场景2: 机器人控制（多电机协同）
```cpp
Can canMotor(DEVICE_CAN_1);
canMotor.Init(Can::BAUD_1M); // 高速通信

// 接受所有电机反馈
canMotor.ConfigFilterAcceptAll();

// 发送速度指令给4个电机
for (uint8_t motorId = 1; motorId <= 4; motorId++) {
  int16_t speed = 1000; // RPM
  uint8_t cmd[] = {
    (uint8_t)(speed >> 8),
    (uint8_t)(speed & 0xFF)
  };
  canMotor.SendStdData(0x200 + motorId, cmd, 2);
}

// 接收电机状态
canMotor.SetRxFifo0Callback([](uint32_t id, uint8_t* data, uint8_t len) {
  uint8_t motorId = id - 0x140;
  int16_t position = (data[0] << 8) | data[1];
  int16_t velocity = (data[2] << 8) | data[3];
  int16_t current = (data[4] << 8) | data[5];
  // 更新电机状态
});
```

### 🏭 场景3: 工业设备监控（CANopen协议）
```cpp
Can canIndustrial(DEVICE_CAN_1);
canIndustrial.Init(Can::BAUD_250K);

// CANopen NMT (Network Management)
uint8_t nmtStart[] = {0x01, 0x00}; // Start Remote Node
canIndustrial.SendStdData(0x000, nmtStart, 2);

// CANopen SDO (Service Data Object) - 读取设备信息
uint8_t sdoRead[] = {0x40, 0x18, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00};
canIndustrial.SendStdData(0x600 + nodeId, sdoRead, 8);

// CANopen PDO (Process Data Object) - 实时数据传输
canIndustrial.SetRxFifo0Callback([](uint32_t id, uint8_t* data, uint8_t len) {
  if (id >= 0x180 && id < 0x200) {
    // TPDO1 - 传感器数据
    uint8_t nodeId = id - 0x180;
    // 处理数据
  }
});
```

### 🔬 场景4: 远程诊断（使用远程帧）
```cpp
// 主节点：请求从节点数据
canMaster.SendRemoteFrame(0x300); // 请求ID 0x300的数据

// 从节点：响应远程帧请求
canSlave.SetRxFifo0Callback([](uint32_t id, uint8_t* data, uint8_t len) {
  if (msg.isRemote && id == 0x300) {
    // 收到远程帧请求，发送诊断数据
    uint8_t diagnostics[] = {
      errorCount,
      temperature,
      voltage,
      status
    };
    canSlave.SendStdData(0x300, diagnostics, 4);
  }
});
```

---

## 实现要点

### ⚙️ 1. 波特率配置

**计算参数**（APB1时钟42MHz为例）:

```cpp
void CalculateBaudRateParams(uint32_t baudRate, 
                            uint32_t* prescaler,
                            uint32_t* bs1,
                            uint32_t* bs2,
                            uint32_t* sjw)
{
  uint32_t apb1Clock = HAL_RCC_GetPCLK1Freq();
  
  // 采样点设置在75%位置
  // BS1 = 12, BS2 = 4, Total = 1 + 12 + 4 = 17
  *bs1 = CAN_BS1_12TQ;
  *bs2 = CAN_BS2_4TQ;
  *sjw = CAN_SJW_1TQ;
  
  // 计算预分频器
  *prescaler = apb1Clock / (baudRate * 17);
}
```

**常用配置**（42MHz APB1）:
| 波特率 | Prescaler | BS1 | BS2 | 采样点 |
|--------|-----------|-----|-----|--------|
| 1M     | 3         | 11  | 2   | 85.7%  |
| 500K   | 6         | 11  | 2   | 85.7%  |
| 250K   | 12        | 11  | 2   | 85.7%  |
| 125K   | 24        | 11  | 2   | 85.7%  |

### 🔄 2. 发送队列实现

```cpp
void Can::ProcessTxQueue()
{
  // 检查是否有待发送的消息
  while (txReadIndex != txWriteIndex) {
    // 检查可用邮箱
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
      return; // 邮箱全忙，等待下次中断
    }
    
    // 取出消息
    CanMessage& msg = txBuffer[txReadIndex];
    
    // 发送
    CAN_TxHeaderTypeDef txHeader;
    txHeader.StdId = msg.isExtended ? 0 : msg.id;
    txHeader.ExtId = msg.isExtended ? msg.id : 0;
    txHeader.IDE = msg.isExtended ? CAN_ID_EXT : CAN_ID_STD;
    txHeader.RTR = msg.isRemote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    txHeader.DLC = msg.len;
    
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(hcan, &txHeader, msg.data, &mailbox);
    
    // 更新读指针
    txReadIndex = (txReadIndex + 1) % TX_BUFFER_SIZE;
  }
}
```

### 🛡️ 3. 错误处理

```cpp
// 在Callback中检测错误
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t errorCode = HAL_CAN_GetError(hcan);
  
  if (errorCode & HAL_CAN_ERROR_EWG) {
    // Error Warning：错误计数达到96
  }
  if (errorCode & HAL_CAN_ERROR_EPV) {
    // Error Passive：错误计数达到128
  }
  if (errorCode & HAL_CAN_ERROR_BOF) {
    // Bus Off：错误计数超过255
    // 需要重新初始化
  }
  if (errorCode & HAL_CAN_ERROR_STF) {
    // Stuff Error：填充错误
  }
  if (errorCode & HAL_CAN_ERROR_FOR) {
    // Form Error：格式错误
  }
  if (errorCode & HAL_CAN_ERROR_ACK) {
    // Ack Error：没有收到应答（总线上没有其他节点）
  }
  if (errorCode & HAL_CAN_ERROR_BR) {
    // Bit Recessive Error：位隐性错误
  }
  if (errorCode & HAL_CAN_ERROR_BD) {
    // Bit Dominant Error：位显性错误
  }
  if (errorCode & HAL_CAN_ERROR_CRC) {
    // CRC Error：CRC校验错误
  }
}
```

### 📊 4. 统计信息（可选扩展）

```cpp
// 可以在类中添加统计成员
struct CanStatistics
{
  uint32_t txCount;        // 发送计数
  uint32_t rxCount;        // 接收计数
  uint32_t txErrorCount;   // 发送错误计数
  uint32_t rxErrorCount;   // 接收错误计数
  uint32_t busOffCount;    // 总线关闭次数
};
```

---

## 💡 最佳实践

### ✅ DO
1. **初始化时先配置滤波器**，再启动CAN
2. **使用合适的波特率**：短距离用1M，长距离用125K/250K
3. **定期检查总线状态**，及时处理Bus Off
4. **使用发送队列**避免消息丢失
5. **在回调中快速处理**，避免阻塞中断

### ❌ DON'T
1. 不要在中断回调中执行耗时操作
2. 不要忘记配置滤波器（默认拒绝所有）
3. 不要在高速率下使用过长的线缆
4. 不要忽略错误计数和总线状态
5. 不要在未检查邮箱状态时连续发送

---

## 📚 参考资料
- STM32 CAN 外设参考手册
- ISO 11898-1 (CAN 2.0 规范)
- CANopen 协议标准
- J1939 协议标准（用于商用车辆）

