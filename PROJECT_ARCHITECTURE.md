# FOCDriver V2 - F405RGT6 项目架构文档

> **目的**：本文档供AI会话快速理解项目全局规范和架构。阅读本文后即可掌握项目结构、开发约定和关键设计决策。

---

## 1. 项目概述

**硬件**：STM32F405RGT6（ARM Cortex-M4 @ 168 MHz）
**开发工具**：EIDE（Embedded IDE）+ Keil MDK，编译器 ARM GCC
**操作系统**：FreeRTOS
**应用场景**：无刷电机FOC（磁场定向控制）驱动器，支持CAN总线MIT协议，适用于机器人关节控制

**核心原则**：
- `User/` 目录下全是用户自己编写的代码，其余均为 CubeMX 生成或第三方库，**修改应集中在 `User/` 目录**
- C++ 与 C 混用：BSP层、App层、部分Lib用C++；纯算法（pid、MT6701等）用C
- 实时控制在中断/ISR上下文中运行，严禁在ISR中调用阻塞函数

---

## 2. 目录结构

```
FOCDriver_V2_F405RGT6/
├── Core/                        # CubeMX 生成，勿随意修改
│   ├── Inc/                     # 外设头文件（main.h, can.h, adc.h, tim.h...）
│   └── Src/                     # 外设初始化（main.c, can.c, adc.c, stm32f4xx_it.c...）
├── User/                        # ★ 用户代码区
│   ├── App/
│   │   ├── Inc/
│   │   │   ├── common_inc.h     # 全局宏、电机型号选择、公共头文件
│   │   │   ├── CmdTask.h        # CAN指令任务声明
│   │   │   └── SafetyTask.h     # 安全任务（已实现但暂注释）
│   │   └── Src/
│   │       ├── user_main.cpp    # 用户入口，任务创建，主循环
│   │       ├── CmdTask.cpp      # CAN命令解析与MIT协议处理
│   │       ├── MotorTask.cpp    # 实时电机控制循环（10kHz ISR驱动）
│   │       └── Callback.cpp     # HAL中断路由（ADC/UART/CAN回调）
│   ├── Bsp/
│   │   ├── Inc/
│   │   │   ├── BspUart.h        # UART+DMA驱动
│   │   │   ├── BspCan.h         # CAN总线驱动
│   │   │   ├── BspPwm.h         # PWM控制
│   │   │   ├── BspTimer.h       # 定时器抽象
│   │   │   └── BspStatus.h      # 错误码/结果类型
│   │   └── Src/
│   │       ├── BspUart.cpp
│   │       ├── BspCan.cpp
│   │       ├── BspPwm.cpp
│   │       └── BspTimer.cpp
│   └── Lib/
│       ├── Inc/
│       │   ├── Foc.h            # ★ FOC控制器核心头文件
│       │   ├── pid.h            # PID控制器
│       │   ├── MT6701.h         # 磁编码器SPI驱动
│       │   ├── LowPassFilter.h  # 低通滤波器（CMSIS-DSP双二阶）
│       │   ├── Log.h            # 调试日志（新增，未完全集成）
│       │   ├── VOFA.h           # VOFA实时可视化协议
│       │   ├── RingBuffer.h     # 环形缓冲区
│       │   └── dwt.h            # ARM DWT周期计数器
│       └── Src/
│           ├── Foc.cpp          # ★ FOC算法实现
│           ├── pid.c
│           ├── MT6701.c
│           ├── LowPassFilter.c
│           ├── Log.cpp          # 日志实现（新增）
│           ├── VOFA.c
│           ├── RingBuffer.c
│           ├── Uart_DMA.c
│           ├── dwt.c
│           └── ws2812.cpp       # WS2812 RGB LED控制
├── Drivers/                     # STM32 HAL库，勿修改
├── Middlewares/                 # FreeRTOS + USB库，勿修改
├── MDK-ARM/                     # Keil工程文件
├── .eide/eide.yml               # EIDE工程配置
├── FOCDriver_V2_F405RGT6.ioc    # CubeMX工程文件
├── Timer_Usage.md               # 定时器使用文档
├── BSP_Driver_Doc.md            # BSP驱动文档
└── PROJECT_ARCHITECTURE.md      # 本文档
```

---

## 3. 硬件外设映射

| 外设 | 配置 | 用途 |
|------|------|------|
| **TIM1** | 168MHz，互补PWM，死区控制 | 三相桥 PWM 输出（20kHz） |
| **TIM1_CC4** | 触发ADC注入转换 | 20kHz 电流采样同步触发 |
| **TIM2** | 通用定时器 | 辅助计时 |
| **TIM3 + DMA** | DMA驱动PWM | WS2812 LED 控制 |
| **ADC1（注入）** | PA0/PA1/PA2/PA3 | IA/IB/IC 相电流 + VBUS 电压采样 |
| **ADC2** | - | 备用 |
| **CAN1** | 1 Mbps，标准帧 | MIT协议电机控制指令收发 |
| **SPI1** | - | MT6701 磁编码器（CS: PC4） |
| **SPI2** | - | 备用 |
| **USART1 + DMA** | PA9(TX)/PA10(RX) | 调试串口 + VOFA可视化 |
| **I2C2** | - | 备用 |

**PWM 引脚分配**：
- HIN1/2/3: PA8, PA9, PA10（高侧）
- LIN1/2/3: PA7, PB14, PB1（低侧）

---

## 4. 软件分层架构

```
┌─────────────────────────────────────────────────┐
│              Application Layer (App/)            │
│  user_main  │  CmdTask  │  MotorTask  │ Callback │
├─────────────────────────────────────────────────┤
│              Library Layer (Lib/)                │
│   Foc   │  PID  │  MT6701  │  LPF  │  VOFA/Log  │
├─────────────────────────────────────────────────┤
│              BSP Layer (Bsp/)                    │
│  BspUart │ BspCan │ BspPwm │ BspTimer │ BspStatus│
├─────────────────────────────────────────────────┤
│         HAL / CubeMX Generated (Core/)           │
│         STM32 HAL Drivers / FreeRTOS             │
└─────────────────────────────────────────────────┘
```

---

## 5. FOC 控制核心

### 5.1 Foc 类（`User/Lib/Inc/Foc.h`，`User/Lib/Src/Foc.cpp`）

FOC 控制器以 C++ 类封装，唯一实例通常为全局对象。

**关键状态成员**：
```cpp
float motorAngle;          // 机械角度 (rad)
float velocity;            // 转速 (rad/s)
float Ia, Ib, Ic;          // 三相电流 (A)
float Iq, Id;              // d-q 轴电流 (A)
float voltageA, voltageB, voltageC; // 相电压
float angleSingleTurn;     // 单圈角度

// 控制目标
float targetPosition;      // 位置目标 (rad)
float targetSpeed;         // 速度目标 (rad/s)
float targetCurrent;       // 电流目标 (A)
float targetTorque;        // 力矩目标 (Nm)
float iqSetpoint, idSetpoint;

// PID 控制器（3级级联）
PIDController currentPidController;   // 50µs，电流环
PIDController speedPidController;     // 1ms，速度环
PIDController positionPidController;  // 1ms，位置环

// 校准
float zeroElectricAngle;   // 电气零点（需校准）
```

**关键方法**：
```cpp
void Init();                     // 初始化，运行零点校准
void UpdateCurrent();            // 从ADC读取电流，Clarke-Park变换
void UpdateVelocity();           // 从编码器计算速度
void MotorControlTask();         // 根据控制模式执行对应控制律
void SetPwm(float Uq, float Ud); // SVPWM调制，输出PWM
```

### 5.2 控制流（实时，20kHz）

```
ADC注入完成中断（TIM1_CC4触发，20kHz）
    │
    ├─→ Callback.cpp: HAL_ADCEx_InjectedConvCpltCallback()
    │         │
    │         └─→ MotorTask.cpp: MotorTask()
    │                   │
    │                   ├─→ foc.UpdateCurrent()
    │                   │     ├─ 读 ADC 原始值 → 换算 A
    │                   │     ├─ Clarke 变换（abc → αβ）
    │                   │     ├─ Park 变换（αβ → dq）
    │                   │     └─ LPF 滤波 Iq/Id
    │                   │
    │                   ├─→ foc.UpdateMotorAngle()  ← MT6701读角度（每次）
    │                   │
    │                   ├─→ 每20次（1kHz）：
    │                   │     └─ foc.UpdateVelocity()
    │                   │
    │                   └─→ foc.MotorControlTask()
    │                         ├─ 电流环PID（20kHz）
    │                         └─ 速度/位置环PID（内部÷20，1kHz）
    │                               └─→ SVPWM → TIM1 CCR寄存器
```

### 5.3 控制模式

| 模式 | 枚举值 | 说明 |
|------|--------|------|
| `NONE` | 0 | 无控制，释放 |
| `TORQUE` | 1 | 直接力矩控制 |
| `VELOCITY` | 2 | 速度PID控制 |
| `POSITION` | 3 | 位置PID（级联速度） |
| `MIT` | 4 | MIT协议混合控制（位置+速度+力矩+增益） |
| `RATCHET` | 5 | 棘轮/触感反馈模式 |
| `INERTIA` | 6 | 惯性飞轮模拟 |

### 5.4 滤波器参数

| 滤波器 | 截止频率 | 采样率 | 用途 |
|--------|---------|--------|------|
| 速度滤波 | 100 Hz | 1 kHz | 速度平滑 |
| Iq 滤波 | 40 Hz | 20 kHz | 电流反馈 |
| Id 滤波 | 40 Hz | 20 kHz | 电流反馈 |
| 相电流滤波 | 1 kHz | 20 kHz | 谐波抑制 |

---

## 6. CAN 通信协议

**物理层**：CAN1，1 Mbps，标准11位帧ID

**帧格式**：
```
CAN ID = Motor Base ID | Ctrl Byte
  Base ID: 0x100/0x200/0x300/0x400（电机地址）
  Ctrl Byte:
    0x01 = TORQUE_CTRL     (力矩控制)
    0x02 = VELOCITY_CTRL   (速度控制)
    0x03 = POSITION_CTRL   (位置控制)
    0x04 = MIT_CTRL        (MIT混合控制)
    0x05 = STATUS_FEEDBACK (状态反馈)
```

**MIT 协议数据帧（8字节打包）**：
```
Byte[0:1]   → Position  : 16bit, 范围 [-π, π] rad
Byte[2:2.5] → Velocity  : 12bit, 范围 [-100, 100] rad/s
Byte[2.5:3] → KP        : 12bit, 范围 [0, 10]
Byte[4:4.5] → KD        : 12bit, 范围 [0, 0.1]
Byte[4.5:5] → Torque    : 12bit, 范围 [-Tmax, Tmax] Nm
```

**编码/解码**：`FloatToUint()` / `UintToFloat()` 定点转换函数

---

## 7. FreeRTOS 任务结构

| 任务 | 触发方式 | 优先级 | 作用 |
|------|---------|--------|------|
| `DefaultTask` | CubeMX创建 | 24 | 调用 `userMain()` |
| `CmdTask` | `ulTaskNotifyTake()` 等待CAN通知 | Normal | 解析CAN指令，更新控制目标 |
| `MotorTask` | ADC ISR直接调用（非独立任务） | ISR级 | 电流采样+控制输出 |
| `SafetyTask` | 10ms周期（已注释） | - | 安全检查、限幅、急停 |

**任务间同步**：
- CAN接收回调 → `vTaskNotifyGiveFromISR()` → 唤醒 `CmdTask`
- `TaskHandle_t cmdTaskHandle` 为全局句柄

---

## 8. 电机参数配置

通过 `User/App/Inc/common_inc.h` 切换电机型号：
```c
#define MOTOR_4621  1   // 4621 电机（默认）
#define MOTOR_2804  0   // 2804 电机
```

| 参数 | 4621 | 2804 |
|------|------|------|
| 极对数 | 11 | 7 |
| 力矩常数 | 0.14 Nm/A | 0.06 Nm/A |
| 力矩限制 | 0.186 Nm | 0.14 Nm |
| 转速限制 | 856 RPM (90 rad/s) | 2200 RPM |
| 供电电压 | 12.6V（3S） | - |

**重要可配置宏（`User/Lib/Inc/Foc.h`）**：
```c
VOLTAGE_POWER_SUPPLY   // 电源电压
PWM_MAX_VALUE          // PWM最大计数值（8400 = 20kHz @ 168MHz）
DEFAULT_CURRENT_KP/KI  // 电流环PID增益
DEFAULT_SPEED_KP/KI/KD // 速度环PID增益
DEFAULT_POS_KP/KI/KD   // 位置环PID增益
CURRENT_LIMIT          // 电流限制
SPEED_LIMIT            // 速度限制
VOLTAGE_LIMIT          // 电压限制
```

---

## 9. 调试与监控

### VOFA 实时可视化
- 协议：`VOFA+` JustFloat格式
- 接口：USART1（DMA）
- 用于：电机参数实时波形监控
- 函数：`VOFA_SendJustFloat()`，`VOFA_RegisterData_float()`

### Log 系统（新增，`User/Lib/Inc/Log.h`）
- 单例模式，宏接口：`LOG_INFO()`, `LOG_WARN()`, `LOG_ERROR()`
- 支持ANSI颜色码，非阻塞队列输出

### DWT 计时器
- 文件：`User/Lib/Src/dwt.c`
- 用途：µs 级精确计时，性能分析

---

## 10. 代码规范与约定

### 文件组织
- **头文件保护**：使用 `#pragma once`
- **C++类**：BSP层和部分Lib用C++封装HAL
- **C接口**：纯算法模块（pid.c、MT6701.c）保持C接口

### 命名规范
- 类名：`PascalCase`（如 `BspUart`, `Foc`）
- 成员变量：`camelCase`（如 `motorAngle`, `targetSpeed`）
- 宏/常量：`UPPER_SNAKE_CASE`（如 `PWM_MAX_VALUE`）
- 函数：`PascalCase`（类方法）或 `snake_case`（C函数）

### ISR 约束
- ISR 中**禁止**调用 FreeRTOS 阻塞 API（用 `FromISR` 变体）
- ISR 中**禁止**进行浮点运算以外的复杂逻辑（FPU硬件支持浮点）
- `MotorTask` 实质上在 ADC ISR 上下文中执行，需严格控制执行时间

### 错误处理
- BSP层使用 `BspResult<T>` 模板返回错误码
- 应用层通过 `BspStatus` 枚举标识状态

---

## 11. 构建系统

- **工程文件**：`.eide/eide.yml`（EIDE）+ `MDK-ARM/`（Keil）
- **编译器**：ARM GCC，目标 Cortex-M4F（硬浮点 `-mfpu=fpv4-sp-d16 -mfloat-abi=hard`）
- **输出**：`build/` 目录，生成 `.hex`/`.bin`/`.elf`
- **关键编译选项**：启用 CMSIS-DSP（`USE_HAL_DRIVER`, `STM32F405xx`）

---

## 12. 当前开发状态（截至文档创建）

**已实现功能**：
- [x] Clarke-Park 变换 + SVPWM
- [x] 三级级联PID（电流/速度/位置）
- [x] MT6701 SPI磁编码器读取
- [x] CAN MIT协议收发
- [x] VOFA实时调试
- [x] WS2812 LED状态指示
- [x] 多种控制模式（Torque/Velocity/Position/MIT/Ratchet/Inertia）

**待完善/注意事项**：
- [ ] SafetyTask 已实现但注释掉，需要启用并调试
- [ ] Log.h/Log.cpp 为新增文件，尚未完全集成
- [ ] 电气零点校准（`zeroElectricAngle`）需在每次上电后执行
- [ ] 电机型号通过宏切换，需确认 `common_inc.h` 中的选择正确

---

*文档由 Claude Code 根据代码库审查自动生成，请在代码发生重大变更后及时更新。*
