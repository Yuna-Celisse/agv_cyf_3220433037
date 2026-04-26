#include "app_encoder.h"

#include "main.h"

/* Wiring used by this firmware:
 * E1A -> PA12, E1B -> PA15
 * E2A -> PA10, E2B -> PA11
 */
#define ENC_LEFT_A_Pin GPIO_PIN_12
#define ENC_LEFT_A_GPIO_Port GPIOA
#define ENC_LEFT_B_Pin GPIO_PIN_15
#define ENC_LEFT_B_GPIO_Port GPIOA
#define ENC_RIGHT_A_Pin GPIO_PIN_10
#define ENC_RIGHT_A_GPIO_Port GPIOA
#define ENC_RIGHT_B_Pin GPIO_PIN_11
#define ENC_RIGHT_B_GPIO_Port GPIOA

#define ENC_LEFT_DIR_SIGN -1  /* 左轮方向符号：用于统一正反方向 */
#define ENC_RIGHT_DIR_SIGN 1  /* 右轮方向符号 */

static volatile int32_t s_left_count = 0;  /* 左轮累计编码器计数 */
static volatile int32_t s_right_count = 0; /* 右轮累计编码器计数 */

static int8_t AppEncoder_ReadStep(GPIO_TypeDef *a_port,
                                  uint16_t a_pin,
                                  GPIO_TypeDef *b_port,
                                  uint16_t b_pin,
                                  int8_t dir_sign)
{
  GPIO_PinState a_state = HAL_GPIO_ReadPin(a_port, a_pin); /* 读取 A 相当前电平 */
  GPIO_PinState b_state = HAL_GPIO_ReadPin(b_port, b_pin); /* 读取 B 相当前电平 */
  int8_t step = (a_state == b_state) ? 1 : -1;             /* AB相同/不同判定方向增量 */

  return (int8_t)(step * dir_sign); /* 再乘轮子方向修正符号，得到最终步进 */
}

void AppEncoder_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE(); /* 使能 GPIOA 时钟 */

  GPIO_InitStruct.Pin = ENC_LEFT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_LEFT_B_GPIO_Port, &GPIO_InitStruct); /* 左轮 B 相为上拉输入 */

  GPIO_InitStruct.Pin = ENC_RIGHT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_RIGHT_B_GPIO_Port, &GPIO_InitStruct); /* 右轮 B 相为上拉输入 */

  GPIO_InitStruct.Pin = ENC_LEFT_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_LEFT_A_GPIO_Port, &GPIO_InitStruct); /* 左轮 A 相双沿中断 */

  GPIO_InitStruct.Pin = ENC_RIGHT_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_RIGHT_A_GPIO_Port, &GPIO_InitStruct); /* 右轮 A 相双沿中断 */

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2U, 0U); /* 设置 EXTI15_10 中断优先级 */
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);           /* 使能中断 */
}

void AppEncoder_HandleExti(uint16_t gpio_pin)
{
  if (gpio_pin == ENC_LEFT_A_Pin)
  {
    /* 左轮 A 相边沿到来时，结合 B 相判定方向并累加。 */
    s_left_count += AppEncoder_ReadStep(ENC_LEFT_A_GPIO_Port,
                                        ENC_LEFT_A_Pin,
                                        ENC_LEFT_B_GPIO_Port,
                                        ENC_LEFT_B_Pin,
                                        ENC_LEFT_DIR_SIGN);
  }
  else if (gpio_pin == ENC_RIGHT_A_Pin)
  {
    /* 右轮 A 相边沿到来时，结合 B 相判定方向并累加。 */
    s_right_count += AppEncoder_ReadStep(ENC_RIGHT_A_GPIO_Port,
                                         ENC_RIGHT_A_Pin,
                                         ENC_RIGHT_B_GPIO_Port,
                                         ENC_RIGHT_B_Pin,
                                         ENC_RIGHT_DIR_SIGN);
  }
}

void AppEncoder_Reset(void)
{
  __disable_irq();  /* 关中断，避免读写竞争 */
  s_left_count = 0; /* 清零左计数 */
  s_right_count = 0;/* 清零右计数 */
  __enable_irq();   /* 开中断 */
}

AppEncoder_Counts_t AppEncoder_GetCounts(void)
{
  AppEncoder_Counts_t counts;

  __disable_irq();                 /* 原子读取，防止中断更新打断 */
  counts.left_count = s_left_count;
  counts.right_count = s_right_count;
  __enable_irq();

  return counts; /* 返回当前双轮累计计数 */
}
