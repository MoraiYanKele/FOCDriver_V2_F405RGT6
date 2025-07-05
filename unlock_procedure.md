# STM32F405RGT6 解锁步骤

## 硬件连接
1. BOOT0 (引脚60, PH3) -> 3.3V
2. BOOT1 (引脚37, PB2) -> GND
3. 重新上电

## 软件步骤

### 方法1: 使用STM32CubeProgrammer
1. 打开STM32CubeProgrammer
2. 选择ST-LINK连接
3. 点击"Connect"
4. 进入"Option Bytes"页面
5. 检查并修改以下设置：
   - RDP (Read Protection) = Level 0
   - WRP (Write Protection) = 清除所有保护
6. 点击"Apply"应用设置
7. 断电，恢复BOOT0和BOOT1为正常状态：
   - BOOT0 -> GND (引脚60)
   - BOOT1 -> 不连接或GND (引脚37)
8. 重新上电，重新烧录程序

### 方法2: 使用命令行
```batch
# 连接到bootloader
STM32_Programmer_CLI.exe -c port=SWD

# 清除读保护
STM32_Programmer_CLI.exe -c port=SWD -ob RDP=0xAA

# 全擦除
STM32_Programmer_CLI.exe -c port=SWD -e all

# 重新烧录
STM32_Programmer_CLI.exe -c port=SWD -w "FOCDriver_V2_F405RGT6.hex" 0x08000000
```

## 引脚对照表 (STM32F405RGT6 LQFP64)
- 引脚37: PB2 (BOOT1)
- 引脚60: PH3/BOOT0

## 注意事项
1. 确保BOOT0连接到3.3V，不是5V
2. 操作前务必断电
3. 如果仍无法连接，可能需要检查：
   - ST-LINK连接是否正常
   - 供电是否稳定
   - NRST引脚是否正常
