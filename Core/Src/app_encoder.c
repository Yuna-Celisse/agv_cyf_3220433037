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

#define ENC_LEFT_DIR_SIGN -1
#define ENC_RIGHT_DIR_SIGN 1

static volatile int32_t s_left_count = 0;
static volatile int32_t s_right_count = 0;

static int8_t AppEncoder_ReadStep(GPIO_TypeDef *a_port,
                                  uint16_t a_pin,
                                  GPIO_TypeDef *b_port,
                                  uint16_t b_pin,
                                  int8_t dir_sign)
{
  GPIO_PinState a_state = HAL_GPIO_ReadPin(a_port, a_pin);
  GPIO_PinState b_state = HAL_GPIO_ReadPin(b_port, b_pin);
  int8_t step = (a_state == b_state) ? 1 : -1;

  return (int8_t)(step * dir_sign);
}

void AppEncoder_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = ENC_LEFT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_LEFT_B_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ENC_RIGHT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_RIGHT_B_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ENC_LEFT_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_LEFT_A_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ENC_RIGHT_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_RIGHT_A_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2U, 0U);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void AppEncoder_HandleExti(uint16_t gpio_pin)
{
  if (gpio_pin == ENC_LEFT_A_Pin)
  {
    s_left_count += AppEncoder_ReadStep(ENC_LEFT_A_GPIO_Port,
                                        ENC_LEFT_A_Pin,
                                        ENC_LEFT_B_GPIO_Port,
                                        ENC_LEFT_B_Pin,
                                        ENC_LEFT_DIR_SIGN);
  }
  else if (gpio_pin == ENC_RIGHT_A_Pin)
  {
    s_right_count += AppEncoder_ReadStep(ENC_RIGHT_A_GPIO_Port,
                                         ENC_RIGHT_A_Pin,
                                         ENC_RIGHT_B_GPIO_Port,
                                         ENC_RIGHT_B_Pin,
                                         ENC_RIGHT_DIR_SIGN);
  }
}

void AppEncoder_Reset(void)
{
  __disable_irq();
  s_left_count = 0;
  s_right_count = 0;
  __enable_irq();
}

AppEncoder_Counts_t AppEncoder_GetCounts(void)
{
  AppEncoder_Counts_t counts;

  __disable_irq();
  counts.left_count = s_left_count;
  counts.right_count = s_right_count;
  __enable_irq();

  return counts;
}
