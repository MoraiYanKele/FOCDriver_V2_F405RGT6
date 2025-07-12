# FOC电机安全监控系统使用指南

## 概述

本系统为FOC电机控制添加了完善的安全监控机制，主要在位置控制模式下工作，确保电机在安全范围内运行。

## 安全监控功能

### 1. 监控参数
- **速度限制**: 最大50 rad/s
- **位置限制**: ±π rad (-180° ~ +180°)  
- **电流限制**: 最大5A
- **电压限制**: 最大11V

### 2. 安全状态
```cpp
typedef enum {
    SAFETY_STATE_NORMAL = 0,     // 正常状态
    SAFETY_STATE_SPEED_LIMIT,    // 速度超限
    SAFETY_STATE_POSITION_LIMIT, // 位置超限
    SAFETY_STATE_CURRENT_LIMIT,  // 电流超限
    SAFETY_STATE_VOLTAGE_LIMIT,  // 电压超限
    SAFETY_STATE_EMERGENCY_STOP, // 紧急停止
    SAFETY_STATE_RECOVERING,     // 恢复中
} SafetyStateTypedef;
```

### 3. 保护机制
- 故障检测延时：100ms（避免误触发）
- 关断时间：500ms（让电机完全停止）
- 重启延时：2000ms（确保系统稳定）

## 任务架构

### 1. SafetyTask (安全监控任务)
- **优先级**: `osPriorityHigh`
- **周期**: 10ms
- **栈大小**: 512字节
- **功能**: 实时监控电机状态，执行安全保护

### 2. MotorTask (电机控制任务)
- **优先级**: `osPriorityHigh`  
- **周期**: 1ms
- **功能**: 执行FOC控制，集成安全检查

## 使用方法

### 1. 基本API

```cpp
// 检查安全状态
bool IsSafetyOK(void);

// 获取安全状态
SafetyStateTypedef GetSafetyState(void);

// 请求紧急停止
void RequestEmergencyStop(void);

// 清除安全错误
void ClearSafetyError(void);

// 设置安全限制
void SetSafetyLimits(float maxSpeed, float maxPos, float minPos, 
                     float maxCurrent, float maxVoltage);
```

### 2. 串口命令

#### 紧急停止命令
```
帧格式: 0xAA 0x10 0x00 0x55
说明: 立即停止电机
```

#### 清除安全错误命令  
```
帧格式: 0xAA 0x11 0x00 0x55
说明: 手动清除安全错误，允许重启
```

#### 获取安全状态命令
```
帧格式: 0xAA 0x13 0x00 0x55
说明: 查询当前安全状态
```

### 3. 配置安全参数

在代码中修改安全参数：
```cpp
// 在SafetyTask.cpp中修改safetyParams结构体
SafetyParamsTypedef safetyParams = {
    .maxSpeed = 30.0f,        // 降低最大速度到30 rad/s
    .maxPosition = 1.57f,     // 限制位置范围到±90°
    .minPosition = -1.57f,
    .maxCurrent = 3.0f,       // 降低电流限制到3A
    .maxVoltage = 10.0f,      // 降低电压限制到10V
    .shutdownTime = 300,      // 缩短关断时间到300ms
    .restartDelay = 1500,     // 缩短重启延时到1.5s
};
```

## 工作流程

### 1. 正常运行
```
SafetyTask监控 -> 参数正常 -> 允许MotorTask控制电机
```

### 2. 故障处理
```
检测到超限 -> 100ms确认 -> 紧急停止 -> 500ms关断 -> 2s延时 -> 自动重启
```

### 3. 手动干预
```
串口发送紧急停止命令 -> 立即停止 -> 串口发送清除错误命令 -> 重启
```

## 状态指示

可以通过VOFA或串口打印查看实时状态：
```cpp
// 在PrintTask中添加安全状态显示
SafetyStateTypedef safetyState = GetSafetyState();
bool safetyOK = IsSafetyOK();
Printf("Safety: %s, State: %d\n", safetyOK ? "OK" : "FAULT", safetyState);
```

## 注意事项

1. **任务优先级**: SafetyTask和MotorTask都设置为高优先级，确保实时性
2. **硬件限制**: 确保硬件电路有过流保护，软件保护是辅助手段
3. **参数调试**: 根据实际电机特性调整安全参数
4. **测试验证**: 部署前充分测试各种故障场景

## 扩展功能

### 1. 添加温度监控
```cpp
// 在SafetyTask中添加温度检查
float motorTemperature = GetMotorTemperature();
if (motorTemperature > TEMP_LIMIT) {
    safetyState = SAFETY_STATE_TEMPERATURE_LIMIT;
}
```

### 2. 添加振动检测
```cpp
// 检测电机异常振动
float vibrationLevel = GetVibrationLevel();
if (vibrationLevel > VIBRATION_LIMIT) {
    safetyState = SAFETY_STATE_VIBRATION_LIMIT;
}
```

### 3. 故障日志记录
```cpp
// 记录故障历史
typedef struct {
    uint32_t timestamp;
    SafetyStateTypedef faultType;
    float faultValue;
} FaultLogTypedef;
```
