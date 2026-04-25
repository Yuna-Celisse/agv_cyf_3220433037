#include "app_ultrasonic.h"

#include "main.h"
#include "tim.h"
#include "usart.h"

#define APP_HCSR04_TIMEOUT_MS 35U
#define APP_HCSR04_CM_PER_US_DIVISOR 58U
#define APP_HCSR04_MIN_VALID_US 100U
#define APP_HCSR04_MAX_VALID_US 25000U

static volatile uint8_t hcsr04_busy = 0U;
static volatile uint8_t hcsr04_wait_falling = 0U;
static volatile uint8_t hcsr04_result_ready = 0U;
static volatile uint16_t hcsr04_pulse_start = 0U;
static volatile uint16_t hcsr04_pulse_width_us = 0U;
static volatile uint32_t hcsr04_trigger_tick = 0U;
static volatile uint8_t hcsr04_timeout_flag = 0U;
static uint8_t app_us_uart_report_enable = 0U;

static void AppUltrasonic_DelayUs(uint16_t us)
{
  uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) - start) < us)
  {
  }
}

static void AppUltrasonic_SendDistance(uint16_t distance_cm)
{
  uint8_t tx_buf[] = "US:000cm\r\n";

  if (app_us_uart_report_enable == 0U)
  {
    return;
  }

  if (distance_cm > 999U)
  {
    distance_cm = 999U;
  }

  tx_buf[3] = (uint8_t)('0' + ((distance_cm / 100U) % 10U));
  tx_buf[4] = (uint8_t)('0' + ((distance_cm / 10U) % 10U));
  tx_buf[5] = (uint8_t)('0' + (distance_cm % 10U));

  (void)HAL_UART_Transmit(&huart3, tx_buf, (uint16_t)(sizeof(tx_buf) - 1U), 20U);
}

static void AppUltrasonic_SendNoEcho(void)
{
  uint8_t tx_buf[] = "US:NONE\r\n";

  if (app_us_uart_report_enable == 0U)
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart3, tx_buf, (uint16_t)(sizeof(tx_buf) - 1U), 20U);
}

void AppUltrasonic_Init(uint8_t uart_report_enable)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  app_us_uart_report_enable = uart_report_enable;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = HCSR04_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HCSR04_TRIG_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = HCSR04_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HCSR04_ECHO_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void AppUltrasonic_StartMeasure(void)
{
  if (hcsr04_busy != 0U)
  {
    return;
  }

  hcsr04_busy = 1U;
  hcsr04_wait_falling = 0U;
  hcsr04_result_ready = 0U;
  hcsr04_timeout_flag = 0U;
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
  AppUltrasonic_DelayUs(2U);
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
  AppUltrasonic_DelayUs(10U);
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
  hcsr04_trigger_tick = HAL_GetTick();
}

void AppUltrasonic_Task(uint32_t now_ms)
{
  if ((hcsr04_busy != 0U) && ((now_ms - hcsr04_trigger_tick) >= APP_HCSR04_TIMEOUT_MS))
  {
    hcsr04_busy = 0U;
    hcsr04_timeout_flag = 1U;
  }
}

void AppUltrasonic_HandleEchoExti(uint16_t gpio_pin)
{
  if (gpio_pin != HCSR04_ECHO_Pin)
  {
    return;
  }

  if (hcsr04_busy == 0U)
  {
    return;
  }

  if (hcsr04_wait_falling == 0U)
  {
    if (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin) == GPIO_PIN_SET)
    {
      hcsr04_pulse_start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
      hcsr04_wait_falling = 1U;
    }
    return;
  }

  if (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin) == GPIO_PIN_RESET)
  {
    hcsr04_pulse_width_us = (uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim2) - hcsr04_pulse_start);
    hcsr04_busy = 0U;
    hcsr04_result_ready = 1U;
  }
}

uint8_t AppUltrasonic_IsBusy(void)
{
  return hcsr04_busy;
}

uint8_t AppUltrasonic_FetchResult(uint16_t *distance_cm, uint8_t *has_distance)
{
  uint16_t pulse_width_us;

  if ((distance_cm == 0) || (has_distance == 0))
  {
    return 0U;
  }

  if (hcsr04_result_ready != 0U)
  {
    hcsr04_result_ready = 0U;
    pulse_width_us = hcsr04_pulse_width_us;

    if ((pulse_width_us >= APP_HCSR04_MIN_VALID_US) && (pulse_width_us <= APP_HCSR04_MAX_VALID_US))
    {
      *distance_cm = (uint16_t)(pulse_width_us / APP_HCSR04_CM_PER_US_DIVISOR);
      *has_distance = 1U;
      AppUltrasonic_SendDistance(*distance_cm);
    }
    else
    {
      *has_distance = 0U;
      AppUltrasonic_SendNoEcho();
    }

    return 1U;
  }

  if (hcsr04_timeout_flag != 0U)
  {
    hcsr04_timeout_flag = 0U;
    *has_distance = 0U;
    AppUltrasonic_SendNoEcho();
    return 1U;
  }

  return 0U;
}
