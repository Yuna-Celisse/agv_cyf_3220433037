#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct
{
  int32_t left_count;   /* 左轮累计编码器计数 */
  int32_t right_count;  /* 右轮累计编码器计数 */
} AppEncoder_Counts_t;

void AppEncoder_Init(void);                            /* 编码器GPIO与中断初始化 */
void AppEncoder_HandleExti(uint16_t gpio_pin);        /* 在 EXTI 回调中处理编码器引脚边沿 */
void AppEncoder_Reset(void);                           /* 清零左右编码器累计计数 */
AppEncoder_Counts_t AppEncoder_GetCounts(void);       /* 原子读取当前左右计数 */

#endif /* APP_ENCODER_H */
