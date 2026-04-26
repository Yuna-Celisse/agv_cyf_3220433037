# 接线说明：STM32F103C8 + TB6612 编码器扩展板

## TB6612 控制线

| STM32 引脚 | 驱动板引脚 | 说明 |
|---|---|---|
| PA8 | PWMA | A 电机 PWM |
| PA9 | PWMB | B 电机 PWM |
| PB6 | AIN1 | A 电机方向 1 |
| PB7 | AIN2 | A 电机方向 2 |
| PB4 | BIN1 | B 电机方向 1 |
| PB3 | BIN2 | B 电机方向 2 |
| PB5 | STBY | TB6612 使能 |

## 电机与编码器

| 驱动板引脚 | STM32 引脚 | 说明 |
|---|---|---|
| AO1/AO2 | 电机 A 两端 | 第 1 路电机 |
| BO1/BO2 | 电机 B 两端 | 第 2 路电机 |
| E1A | PA12 | A 电机编码器 A 相，中断输入 |
| E1B | PA15 | A 电机编码器 B 相，方向判断 |
| E2A | PA10 | B 电机编码器 A 相，中断输入 |
| E2B | PA11 | B 电机编码器 B 相，方向判断 |

编码器 VCC 按电机编码器规格接 3.3 V 或 5 V。若编码器输出是 5 V 推挽信号，必须降压或确认模块输出兼容 STM32 的 3.3 V 输入。

## 供电

| 连接 | 说明 |
|---|---|
| MCU GND -> 驱动板 GND | 必须共地 |
| MCU 3V3 -> 驱动板 VCC | TB6612 逻辑电源 |
| 电池正极 -> 驱动板 VM | 电机电源 |
| 电池负极 -> 驱动板 GND | 电机电源地 |

## ADC 采样口

本次代码先实现编码器速度闭环，ADC 采样暂未启用。若后续要接 ADC，请另选未占用的 ADC 输入脚，并在 CubeMX 中启用 ADC1。

## 调方向

OLED 已占用 PA6/PA7 作为软 I2C 的 SCL/SDA，所以编码器不要再接 PA7。
PA15 默认属于 JTAG 引脚，本工程已关闭 JTAG 并保留 SWD，因此 PA15 可以作为普通输入使用；下载调试仍使用 PA13/PA14。
当前代码已启用 `MOTOR_SWAP_PWM_OUTPUTS`：逻辑左轮速度会输出到 PA9/PWMB，逻辑右轮速度会输出到 PA8/PWMA。若你的车实际是 PA8 控左轮、PA9 控右轮，把 `Core/Src/app_motor.c` 里的 `MOTOR_SWAP_PWM_OUTPUTS` 改为 `0U`。

当前左编码器方向已在代码中设为 `ENC_LEFT_DIR_SIGN = -1`。如果某一路前进时仍越补越快，优先交换该路编码器 A/B 两根线；也可以在 `Core/Src/app_encoder.c` 中调整 `ENC_LEFT_DIR_SIGN` 或 `ENC_RIGHT_DIR_SIGN`。
当前主程序默认关闭速度闭环，`ENABLE_MOTOR_CLOSED_LOOP = 0U`，先用开环 PWM 恢复红外巡线修正。确认 E1A/E2A 有稳定脉冲后，再把它改为 `1U`。

如果电机方向相反，交换该电机 AO1/AO2 或 BO1/BO2，或者调整 `Core/Src/app_motor.c` 里的方向 GPIO 输出逻辑。
