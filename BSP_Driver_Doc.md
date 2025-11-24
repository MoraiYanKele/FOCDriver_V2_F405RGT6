# BSP (板级支持包) 驱动层设计文档

## 1. 架构概述

本 BSP (Board Support Package) 层是一个基于 C++ 的面向对象驱动库，构建于 STMicroelectronics 提供的 `STM32 HAL` 库之上。其核心目标是将底层、面向过程的 HAL 库函数封装成高内聚、低耦合的 C++ 类，从而为上层应用提供一个更现代化、更易于使用和扩展的接口。

### 核心设计思想

1.  **设备抽象与注册**:
    *   通过一个中央设备枚举 `BspDevice_t` (`BspDevice.h`) 来统一管理所有硬件外设。
    *   提供一个中央注册表 `BspDevice.cpp`，将 `BspDevice_t` 枚举与具体的 HAL 句柄 (`UART_HandleTypeDef*`, `TIM_HandleTypeDef*` 等) 关联起来。这使得上层代码可以通过逻辑设备ID（如 `DEVICE_USART_1`）而非硬件句柄来操作外设。

2.  **C/C++ 混合编程桥梁 (蹦床函数)**:
    *   STM32 HAL 库的回调函数是标准的 C 函数。为了在 C++ 类的成员函数中处理这些回调，本 BSP 采用了一种名为“蹦床”(Trampoline)的设计模式。
    *   为每个需要回调的模块（如 `Uart`, `Timer`）实现一个全局的 C 函数（例如 `Uart_TxCpltCallback_Trampoline`）。
    *   这个 C 函数在 HAL 的中断服务程序中被调用，它的唯一职责是：通过中断源的句柄，从一个全局实例数组中找到对应的 C++ 对象实例，然后调用该实例的公共成员函数。

3.  **单例模式的变体**:
    *   每个驱动类（如 `Uart`, `Timer`）内部都维护一个静态的实例指针数组（如 `uartInstances`）。
    *   当一个驱动对象被构造时，它会根据自己的设备 ID 将 `this` 指针注册到这个静态数组中。
    *   这使得“蹦床函数”能够根据设备 ID 快速地定位到正确的对象实例，实现了从中断上下文到特定对象上下文的路由。

4.  **资源管理**:
    *   `BspDevice` 模块实现了一个简单的引用计数机制 (`deviceRefCount`)，用于追踪设备是否被占用，防止同一硬件资源被多个上层模块重复初始化或错误地释放。

## 2. 模块详解

### 2.1. BspDevice (核心设备管理器)

`BspDevice` 是整个 BSP 层的基石，它不直接控制硬件，而是作为硬件资源和上层驱动之间的“注册中心”和“查找服务”。

*   **文件**: `BspDevice.h`, `BspDevice.cpp`

*   **关键组件**:
    *   `BspDevice_t`: C 枚举类型，定义了系统中所有可用的外设。**扩展新设备时，必须在此处添加新的枚举成员。**
    *   `deviceHandles[]`: `void*` 类型的静态数组，是核心的映射表，存储了 `BspDevice_t` 对应的 HAL 句柄地址。
    *   `Bsp_GetDeviceHandle(BspDevice_t)`: 根据设备ID获取 HAL 句柄。
    *   `Bsp_FindDeviceByHandle(void*)`: 根据 HAL 句柄反向查找设备ID。这个函数是“蹦床”模式的关键，它使得中断处理函数可以知道是哪个设备触发了中断。

*   **如何扩展**:
    1.  在 `BspDevice.h` 的 `BspDevice_t` 枚举中添加新的设备ID。
    2.  在 `BspDevice.cpp` 的 `deviceHandles` 数组中，将新的设备ID与对应的 HAL 句柄（例如 `&hi2c1`）进行关联。

### 2.2. BspTimer (定时器驱动)

`BspTimer` 是对 HAL 定时器（工作在基本中断模式）的 C++ 封装。

*   **文件**: `BspTimer.h`, `BspTimer.cpp`

*   **功能特性**:
    *   **频率驱动**: 构造函数接受期望的中断频率 (Hz)，并自动计算最优的预分频值 (PSC) 和自动重装载值 (ARR)。
    *   **回调机制**: 支持通过 `SetCallback` 方法注册一个用户自定义的中断回调函数。

*   **工作流程**:
    1.  创建一个 `Timer` 对象: `Timer myTimer(DEVICE_TIMER_2, 1000);`
    2.  `Timer` 构造函数会从 `BspDevice` 获取 `&htim2` 句柄，并计算 PSC 和 ARR。
    3.  构造函数将 `this` 指针注册到静态的 `timerInstances` 数组中。
    4.  用户调用 `myTimer.SetCallback(MyCallback);` 来设置回调。
    5.  用户调用 `myTimer.Start();` 启动定时器中断。
    6.  当 TIM2 中断发生时，`HAL_TIM_PeriodElapsedCallback` 被调用。
    7.  在 `HAL_TIM_PeriodElapsedCallback` 中，我们调用 `Timer_Callback_Trampoline(&htim2);`。
    8.  `Timer_Callback_Trampoline` 通过 `&htim2` 句柄反向查找到 `DEVICE_TIMER_2`，然后从 `timerInstances` 数组中找到 `myTimer` 对象，并调用其 `InvokeCallback` 方法，最终执行用户设置的 `MyCallback`。

*   **使用示例**:
    ```cpp
    #include "BspTimer.h"

    // 定义一个回调函数
    void OnTimerTick()
    {
      // 每毫秒执行一次
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    // 在 main 或某个初始化函数中
    Timer timer2(DEVICE_TIMER_2, 1000); // 实例化一个1kHz的定时器
    timer2.SetCallback(OnTimerTick);
    timer2.Start();
    ```

### 2.3. BspUart (UART 驱动)

`BspUart` 是一个功能强大的 UART 驱动封装，充分利用了 DMA 和环形缓冲区，以实现高性能、非阻塞的串口通信。

*   **文件**: `BspUart.h`, `BspUart.cpp`

*   **功能特性**:
    *   **DMA 驱动**: 发送和接收均由 DMA 控制，极大地降低了 CPU 占用率。
    *   **环形缓冲区**: 发送和接收都使用环形缓冲区作为数据中转站，解决了 DMA 单次传输长度固定、数据易被覆盖的问题，实现了类似标准流的读写体验。
    *   **空闲中断接收 (IDLE Line Detection)**: 接收端采用 `HAL_UARTEx_ReceiveToIdle_DMA` 机制，可以在总线空闲时立即触发中断，处理不等长的数据帧，非常适合用于接收指令或未知长度的数据包。
    *   **printf 风格的格式化输出**: 提供了 `Printf` 成员函数，可以像标准 `printf` 一样方便地发送格式化字符串。

*   **工作流程 (发送)**:
    1.  用户调用 `uart1.SendData(data, size)` 或 `uart1.Printf(...)`。
    2.  数据被写入发送环形缓冲区 `ringBuffer_Tx`。
    3.  `StartDmaTxIfIdle()` 被调用，检查 DMA 是否空闲。
    4.  如果 DMA 空闲，它会计算出从环形缓冲区读指针到末尾（或写指针）的最大连续数据块，并启动 DMA 传输该数据块。
    5.  DMA 传输完成后，`HAL_UART_TxCpltCallback` 中断触发，调用 `Uart_TxCpltCallback_Trampoline`。
    6.  蹦床函数找到对应的 `Uart` 实例，调用其 `TxCpltCallback` 方法。
    7.  `TxCpltCallback` 更新环形缓冲区的读指针，并再次调用 `StartDmaTxIfIdle()`，检查是否还有剩余数据需要发送，形成一个自动的发送链。

*   **工作流程 (接收)**:
    1.  用户调用 `uart1.EnableRxDMA()` 启动接收。
    2.  `HAL_UARTEx_ReceiveToIdle_DMA` 开始监听，DMA 会将收到的数据存入一个临时的 `dmaRxBuffer`。
    3.  当总线空闲（一个字节的时间内没有新数据）或 `dmaRxBuffer` 满了，`HAL_UARTEx_RxEventCallback` 中断触发，调用 `Uart_RxEventCallback_Trampoline`。
    4.  蹦床函数找到 `Uart` 实例，调用其 `RxEventCallback` 方法。
    5.  `RxEventCallback` 将 `dmaRxBuffer` 中的有效数据（长度由 `size` 参数告知）写入主接收环形缓冲区 `ringBuffer_Rx`。
    6.  `RxEventCallback` 再次调用 `HAL_UARTEx_ReceiveToIdle_DMA`，准备下一次接收。
    7.  上层应用可以通过 `uart1.ReceiveData(...)` 从 `ringBuffer_Rx` 中主动读取数据。

*   **使用示例**:
    ```cpp
    #include "BspUart.h"

    // 在 main 或某个初始化函数中
    Uart uart1(DEVICE_USART_1);
    uart1.EnableRxDMA(); // 启动DMA接收

    // 发送数据
    uart1.Printf("System Initialized. VBUS = %.2fV\n", 12.05f);

    // 在主循环或任务中处理接收
    void ProcessUartCommands()
    {
      if (uart1.GetRxDataLength() > 0)
      {
        uint8_t buffer[64];
        uint32_t len = uart1.ReceiveData(buffer, 64);
        // ... 处理接收到的数据 ...
      }
    }
    ```

## 3. 总结

该 BSP 层通过巧妙的 C++ 封装和设计模式，成功地将复杂的 HAL 库操作简化为直观的对象方法调用。它不仅提高了代码的可读性和可维护性，还通过在底层处理 DMA 和中断的复杂性，让上层应用开发者可以更专注于业务逻辑的实现。
