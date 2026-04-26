#include "app_ultrasonic.h"

#include "main.h"
#include "tim.h"
#include "usart.h"

#define APP_HCSR04_TIMEOUT_MS 35U        /* 一次测量的超时时间（ms） */
#define APP_HCSR04_CM_PER_US_DIVISOR 58U /* 回波脉宽换算厘米的除数（经验值） */
#define APP_HCSR04_MIN_VALID_US 100U     /* 判定为有效回波的最小脉宽（us） */
#define APP_HCSR04_MAX_VALID_US 25000U   /* 判定为有效回波的最大脉宽（us） */

static volatile uint8_t hcsr04_busy = 0U;          /* 测量进行中标志 */
static volatile uint8_t hcsr04_wait_falling = 0U;  /* 已捕获上升沿，等待下降沿标志 */
static volatile uint8_t hcsr04_result_ready = 0U;  /* 新结果可读取标志 */
static volatile uint16_t hcsr04_pulse_start = 0U;  /* 回波上升沿时刻（us计数） */
static volatile uint16_t hcsr04_pulse_width_us = 0U; /* 回波脉宽（us） */
static volatile uint32_t hcsr04_trigger_tick = 0U; /* 触发测量时刻（ms） */
static volatile uint8_t hcsr04_timeout_flag = 0U;  /* 本次测量是否超时 */
static uint8_t app_us_uart_report_enable = 0U;     /* 串口打印开关 */

static void AppUltrasonic_DelayUs(uint16_t us)
{
  uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2); /* 记录起始计数 */

  while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) - start) < us)
  {
    /* 基于 TIM2 计数器忙等待，实现微秒级延时。 */
  }
}

static void AppUltrasonic_SendDistance(uint16_t distance_cm)
{
  uint8_t tx_buf[] = "US:000cm\r\n";

  if (app_us_uart_report_enable == 0U)
  {
    return; /* 串口上报关闭时直接返回 */
  }

  if (distance_cm > 999U)
  {
    distance_cm = 999U; /* 文本格式限制为三位数 */
  }

  tx_buf[3] = (uint8_t)('0' + ((distance_cm / 100U) % 10U)); /* 百位 */
  tx_buf[4] = (uint8_t)('0' + ((distance_cm / 10U) % 10U));  /* 十位 */
  tx_buf[5] = (uint8_t)('0' + (distance_cm % 10U));           /* 个位 */

  (void)HAL_UART_Transmit(&huart3, tx_buf, (uint16_t)(sizeof(tx_buf) - 1U), 20U); /* 发送距离文本 */
}

static void AppUltrasonic_SendNoEcho(void)
{
  uint8_t tx_buf[] = "US:NONE\r\n";

  if (app_us_uart_report_enable == 0U)
  {
    return; /* 串口上报关闭时直接返回 */
  }

  (void)HAL_UART_Transmit(&huart3, tx_buf, (uint16_t)(sizeof(tx_buf) - 1U), 20U); /* 发送无回波文本 */
}

void AppUltrasonic_Init(uint8_t uart_report_enable)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  app_us_uart_report_enable = uart_report_enable; /* 保存串口上报配置 */

  __HAL_RCC_GPIOB_CLK_ENABLE(); /* 使能 GPIOB 时钟 */

  GPIO_InitStruct.Pin = HCSR04_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HCSR04_TRIG_GPIO_Port, &GPIO_InitStruct); /* TRIG 配置为推挽输出 */
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET); /* 默认拉低 */

  GPIO_InitStruct.Pin = HCSR04_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HCSR04_ECHO_GPIO_Port, &GPIO_InitStruct); /* ECHO 配置为双沿中断输入 */

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1U, 0U); /* 配置 ECHO 外部中断优先级 */
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);           /* 使能 EXTI1 中断 */
}

void AppUltrasonic_StartMeasure(void)
{
  if (hcsr04_busy != 0U)
  {
    return; /* 正在测量时不重复触发 */
  }

  hcsr04_busy = 1U;               /* 标记开始测量 */
  hcsr04_wait_falling = 0U;       /* 等待先捕获上升沿 */
  hcsr04_result_ready = 0U;       /* 清结果就绪标志 */
  hcsr04_timeout_flag = 0U;       /* 清超时标志 */
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET); /* TRIG 先拉低 */
  AppUltrasonic_DelayUs(2U);      /* 稳定至少 2us */
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);   /* 发送触发高电平 */
  AppUltrasonic_DelayUs(10U);     /* 高电平维持 10us */
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET); /* 结束触发脉冲 */
  hcsr04_trigger_tick = HAL_GetTick(); /* 记录触发时刻用于超时判断 */
}

void AppUltrasonic_Task(uint32_t now_ms)
{
  if ((hcsr04_busy != 0U) && ((now_ms - hcsr04_trigger_tick) >= APP_HCSR04_TIMEOUT_MS))
  {
    hcsr04_busy = 0U;         /* 超时后结束本次测量 */
    hcsr04_timeout_flag = 1U; /* 置位超时标志，等待上层取走 */
  }
}

void AppUltrasonic_HandleEchoExti(uint16_t gpio_pin)
{
  if (gpio_pin != HCSR04_ECHO_Pin)
  {
    return; /* 只处理超声波 ECHO 引脚中断 */
  }

  if (hcsr04_busy == 0U)
  {
    return; /* 非测量期的中断直接忽略 */
  }

  if (hcsr04_wait_falling == 0U)
  {
    if (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin) == GPIO_PIN_SET)
    {
      hcsr04_pulse_start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2); /* 上升沿：记录起始时刻 */
      hcsr04_wait_falling = 1U;                                      /* 转入等待下降沿 */
    }
    return;
  }

  if (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin) == GPIO_PIN_RESET)
  {
    hcsr04_pulse_width_us = (uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim2) - hcsr04_pulse_start); /* 下降沿：得到脉宽 */
    hcsr04_busy = 0U;         /* 本次测量完成 */
    hcsr04_result_ready = 1U; /* 结果可读取 */
  }
}

uint8_t AppUltrasonic_IsBusy(void)
{
  return hcsr04_busy; /* 返回测量忙状态 */
}

uint8_t AppUltrasonic_FetchResult(uint16_t *distance_cm, uint8_t *has_distance)
{
  uint16_t pulse_width_us;

  if ((distance_cm == 0) || (has_distance == 0))
  {
    return 0U; /* 参数无效 */
  }

  if (hcsr04_result_ready != 0U)
  {
    hcsr04_result_ready = 0U;         /* 消费一次结果 */
    pulse_width_us = hcsr04_pulse_width_us; /* 复制脉宽 */

    if ((pulse_width_us >= APP_HCSR04_MIN_VALID_US) && (pulse_width_us <= APP_HCSR04_MAX_VALID_US))
    {
      *distance_cm = (uint16_t)(pulse_width_us / APP_HCSR04_CM_PER_US_DIVISOR); /* us -> cm */
      *has_distance = 1U;                                              /* 标记有效距离 */
      AppUltrasonic_SendDistance(*distance_cm);                        /* 可选串口上报 */
    }
    else
    {
      *has_distance = 0U;       /* 脉宽越界视为无效 */
      AppUltrasonic_SendNoEcho();
    }

    return 1U; /* 本次调用成功取到一条结果（有效或无效） */
  }

  if (hcsr04_timeout_flag != 0U)
  {
    hcsr04_timeout_flag = 0U; /* 消费超时事件 */
    *has_distance = 0U;       /* 超时视为无有效距离 */
    AppUltrasonic_SendNoEcho();
    return 1U;                /* 返回“有事件可处理” */
  }

  return 0U; /* 当前没有新结果也没有超时事件 */
}
