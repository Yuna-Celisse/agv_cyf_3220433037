# 接线说明（STM32F103C8 工程）

## 1. 适用范围

本文档基于当前固件引脚配置，说明本工程应如何接线。
配置来源：
- [Core/Inc/main.h](Core/Inc/main.h)
- [Core/Src/gpio.c](Core/Src/gpio.c)
- [Core/Src/tim.c](Core/Src/tim.c)
- [agv_cyf_3220433037.ioc](agv_cyf_3220433037.ioc)

## 2. 硬件假设

工程中 AIN1/AIN2/BIN1/BIN2/STBY 这组命名，符合双路 H 桥电机驱动器（例如 TB6612FNG 风格）的常见定义。
如果你使用的是其他型号驱动器，请按功能对应映射：PWM、方向、待机/使能。

## 3. MCU 引脚映射

| MCU 引脚 | 固件标签 | 方向 | 功能 |
|---|---|---|---|
| PC13 | LED | 输出 | 板载状态灯 |
| PA8 | TIM1_CH1 | 复用推挽输出 | PWM 输出通道 A |
| PA9 | TIM1_CH2 | 复用推挽输出 | PWM 输出通道 B |
| PA1 | IR1 | 输入 | 红外循迹 out1 |
| PA2 | IR2 | 输入 | 红外循迹 out2 |
| PA3 | IR3 | 输入 | 红外循迹 out3 |
| PA4 | IR4 | 输入 | 红外循迹 out4 |
| PA5 | IR5 | 输入 | 红外循迹 out5 |
| PB6 | AIN1 | 输出 | 电机 A 方向输入 1 |
| PB7 | AIN2 | 输出 | 电机 A 方向输入 2 |
| PB4 | BIN1 | 输出 | 电机 B 方向输入 1 |
| PB3 | BIN2 | 输出 | 电机 B 方向输入 2 |
| PB5 | STBY | 输出 | 驱动器待机/使能控制 |
| PB10 | USART3_TX | 复用推挽输出 | 串口发送 |
| PB11 | USART3_RX | 输入 | 串口接收 |
| PA0 | TIM2_CH1 | 复用推挽输出 | 预留 PWM 输出（当前主循环未启动） |
| PA13 | SWDIO | 调试 | SWD 数据脚 |
| PA14 | SWCLK | 调试 | SWD 时钟脚 |

## 4. 推荐接线（双路 H 桥驱动）

### 控制信号

- PA8 -> PWMA
- PA9 -> PWMB
- PB6 -> AIN1
- PB7 -> AIN2
- PB4 -> BIN1
- PB3 -> BIN2
- PB5 -> STBY

### 红外循迹

- out1 -> PA1
- out2 -> PA2
- out3 -> PA3
- out4 -> PA4
- out5 -> PA5

### 串口调试（USART3）

- USB 转串口 RX -> PB10（USART3_TX）
- USB 转串口 TX -> PB11（USART3_RX）
- USB 转串口 GND -> MCU GND

### 供电与电机侧

- MCU GND -> 驱动器 GND（必须共地）
- MCU 3V3 -> 驱动器逻辑电源 VCC
- 电机电源正极 -> 驱动器 VM（或 VIN_MOTOR）
- 电机电源负极 -> 驱动器 GND
- 驱动器 AO1/AO2 -> 电机 A 两端
- 驱动器 BO1/BO2 -> 电机 B 两端

## 5. 当前固件行为说明

根据 [Core/Src/main.c](Core/Src/main.c)：

- 已启动 TIM1 的 PWM CH1 和 CH2。
- STBY 被拉高（驱动器使能）。
- AIN1 为高、AIN2 为低（电机 A 方向已设定）。
- BIN1/BIN2 在初始化后没有在用户代码中更新，因此保持为 GPIO 初始化时的低电平。

实际效果：
- A 通道处于主动驱动状态。
- B 通道状态取决于你的驱动器真值表中 BIN1=0、BIN2=0 的定义（很多驱动器表现为刹车或滑行）。

## 6. PWM 参数（当前配置）

- TIM1：Prescaler = 72-1，Period = 1000-1，计数时钟 = 72 MHz
  - PWM 频率 = 72 MHz / 72 / 1000 = 1 kHz
- TIM2：Prescaler = 72-1，Period = 20000-1
  - PWM 频率 = 72 MHz / 72 / 20000 = 50 Hz

## 7. 上电检查清单

1. 确认 MCU 与驱动器共地。
2. 确认驱动器逻辑电平兼容 3.3 V。
3. 上电后确认 STBY 为高电平。
4. 用示波器检查 PA8/PA9 是否有 PWM 波形。
5. 如果只有一路电机响应，优先检查 BIN1/BIN2 逻辑是否按需求更新。
6. 如果下载或调试失败，检查 PA13/PA14 的 SWD 连线与供电稳定性。

## 8. 安全注意事项

- 不要从 MCU 的 3V3 引脚给电机供电。
- 建议在驱动器 VM 与 GND 旁增加大电容，抑制电机启动电流冲击。
- 首次测试请先用较小占空比，逐步提升。
