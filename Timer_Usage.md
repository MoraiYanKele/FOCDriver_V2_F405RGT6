# Timer 类参考文档

## 1. 概述

`Timer` 类是一个 C++ 封装器，用于简化 STM32 HAL 库中通用定时器的使用。它通过面向对象的方式，提供了一个易于理解和使用的接口来配置、启动、停止定时器，并设置中断回调函数。

该类依赖于一个板级支持包（BSP）层（由 `BspDevice.h` 定义），该层负责管理和提供硬件句柄（`TIM_HandleTypeDef*`）。`Timer` 类通过 `BspDevice_t` 枚举来识别和获取特定定时器的句柄。

其核心功能是根据用户指定的频率（Hz）自动计算定时器的预分频器（PSC）和自动重装载值（ARR），并处理中断的启动和停止。

## 2. 依赖项

-   `common_inc.h`: 项目通用头文件，应包含 `stm32f4xx_hal.h` 等。
-   `BspDevice.h`: 板级设备管理头文件，需要提供 `BspDevice_t` 枚举、`Bsp_GetDeviceHandle`、`Bsp_SetTimerCallback` 等函数的声明。

## 3. 如何使用

以下是一个基本的使用示例，假设我们想创建一个 10Hz 的定时器，并让它在每次中断时切换 LED 的状态。

### 步骤 1: 包含头文件

在你的应用代码中（例如 `user_main.cpp`），包含 `Timer.h`。

```cpp
#include "Timer.h"
#include "gpio.h" // 假设用于控制LED
```

### 步骤 2: 定义回调函数

创建一个无参数、无返回值的函数，作为定时器中断的回调。

```cpp
void MyTimerCallback()
{
  // 在这里编写中断服务程序中要执行的代码
  // 例如：翻转一个LED
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
```

### 步骤 3: 实例化并配置 Timer 对象

在你的主逻辑中（例如 `userMain` 函数），创建一个 `Timer` 类的实例。

```cpp
void userMain()
{
  // 创建一个Timer对象，关联到DEVICE_TIMER_2，并设置频率为10Hz
  Timer myTimer(DEVICE_TIMER_2, 10);

  // 设置当中断发生时要调用的函数
  myTimer.SetCallback(MyTimerCallback);

  // 启动定时器
  myTimer.Start();

  // ... 主循环 ...
  while(1)
  {
    // 定时器将在后台通过中断自动运行
  }
}
```

**重要**:
确保 `BspDevice.h` 中定义的 `DEVICE_TIMER_2` 与 STM32CubeMX 中配置的 `TIM2` 句柄 `htim2` 对应，并且 `Bsp_GetDeviceHandle` 函数能正确返回该句柄。同时，`HAL_TIM_PeriodElapsedCallback` 中断回调函数需要正确地调用通过 `Bsp_SetTimerCallback` 注册的函数。

## 4. API 详解

### `Timer::Timer(BspDevice_t _deviceID, uint32_t _freqHz = 100)`

**构造函数**

创建一个定时器对象。

-   **`_deviceID`**: `BspDevice_t` 枚举值，用于指定要使用的物理定时器（例如 `DEVICE_TIMER_2`）。
-   **`_freqHz`**: `uint32_t` 类型，指定定时器中断的触发频率，单位是赫兹（Hz）。默认值为 100Hz。频率会被限制在 1Hz 到 10MHz 之间。

构造函数会自动调用 `Bsp_GetDeviceHandle` 获取句柄，并根据指定的频率和芯片主频（`CHIP_FREQ_MHZ`）计算并设置 PSC 和 ARR 寄存器。

### `void Timer::Start()`

**启动定时器**

启动定时器并使能其中断。此后，注册的回调函数将在每次定时器溢出时被调用。

### `void Timer::Stop()`

**停止定时器**

停止定时器并禁用其中断。

### `void Timer::SetCallback(Callback_t _timerCallback)`

**设置回调函数**

注册一个函数，当定时器中断发生时，该函数将被执行。

-   **`_timerCallback`**: 一个函数指针，类型为 `Callback_t`（通常是 `void (*)(void)`)。这个函数应该是无参数、无返回值的。

## 5. 实现细节

-   **频率计算 (`CalcRegister`)**:
    -   该类内部的 `CalcRegister` 方法会根据目标频率动态计算 PSC 和 ARR 的值。
    -   它会尝试找到一个合适的 PSC 值，使得 ARR 的值在 16 位定时器的有效范围内（`0` 到 `65535`）。
    -   计算逻辑考虑了 STM32F4 系列中 APB1 和 APB2 总线上的定时器时钟频率差异。
-   **BSP 交互**:
    -   该类不直接持有硬件句柄，而是通过 `BspDevice` 层在运行时获取，这增强了代码的模块化和可移植性。
    -   回调函数的注册也委托给了 BSP 层，这意味着 BSP 层需要维护一个全局的回调函数指针数组，并在 `HAL_TIM_PeriodElapsedCallback` 中根据触发的定时器句柄来调用相应的回调。
