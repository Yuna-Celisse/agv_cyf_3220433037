/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include "rc522.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HCSR04_TRIG_GPIO_Port GPIOB
#define HCSR04_TRIG_Pin GPIO_PIN_0
#define HCSR04_ECHO_GPIO_Port GPIOB
#define HCSR04_ECHO_Pin GPIO_PIN_1
#define RFID_UID_LEN 4U
#define UART3_TX_BUF_SIZE 32U
#define ENABLE_RFID_UART_REPORT 0U
#define ENABLE_ULTRASONIC_UART_REPORT 0U
#define IR_REPORT_INTERVAL_MS 100U
#define RFID_POLL_INTERVAL_MS 100U
#define HCSR04_MEASURE_INTERVAL_MS 100U
#define HCSR04_TIMEOUT_MS 35U
#define HCSR04_CM_PER_US_DIVISOR 58U
#define MOTOR_DEFAULT_DUTY 0.2f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t last_rfid_poll_tick = 0U;
static uint8_t last_uid[RFID_UID_LEN] = {0};
static uint8_t has_last_uid = 0U;
static uint32_t last_ultrasonic_poll_tick = 0U;
static uint16_t last_distance_cm = 0U;
static uint8_t has_last_distance = 0U;
static volatile uint8_t hcsr04_busy = 0U;
static volatile uint8_t hcsr04_wait_falling = 0U;
static volatile uint8_t hcsr04_result_ready = 0U;
static volatile uint16_t hcsr04_pulse_start = 0U;
static volatile uint16_t hcsr04_pulse_width_us = 0U;
static volatile uint32_t hcsr04_trigger_tick = 0U;
static volatile uint8_t hcsr04_timeout_flag = 0U;
static volatile uint8_t uart3_tx_busy = 0U;
static volatile uint8_t uart3_tx_pending = 0U;
static uint8_t uart3_tx_active_buf[UART3_TX_BUF_SIZE] = {0};
static uint8_t uart3_tx_pending_buf[UART3_TX_BUF_SIZE] = {0};
static uint16_t uart3_tx_active_len = 0U;
static uint16_t uart3_tx_pending_len = 0U;
static uint32_t last_ir_report_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static char HexDigit(uint8_t value);
static uint8_t UidEquals(const uint8_t left[4], const uint8_t right[4]);
static void UidCopy(uint8_t dst[4], const uint8_t src[4]);
static void RFID_SendUid(const uint8_t uid[4]);
static void RFID_SendNoCard(void);
static void HCSR04_InitPins(void);
static void HCSR04_DelayUs(uint16_t us);
static void HCSR04_StartMeasure(void);
static void Ultrasonic_SendDistance(uint16_t distance_cm);
static void Ultrasonic_SendNoEcho(void);
static uint8_t IR_ReadMask(void);
static void IR_SendState(uint8_t mask);
static void UART3_SendAsync(const uint8_t *data, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static char HexDigit(uint8_t value)
{
  if (value < 10U)
  {
    return (char)('0' + value);
  }

  return (char)('A' + (value - 10U));
}

static uint8_t UidEquals(const uint8_t left[4], const uint8_t right[4])
{
  uint8_t i;

  for (i = 0U; i < RFID_UID_LEN; i++)
  {
    if (left[i] != right[i])
    {
      return 0U;
    }
  }

  return 1U;
}

static void UidCopy(uint8_t dst[4], const uint8_t src[4])
{
  uint8_t i;

  for (i = 0U; i < RFID_UID_LEN; i++)
  {
    dst[i] = src[i];
  }
}

static void RFID_SendUid(const uint8_t uid[4])
{
#if ENABLE_RFID_UART_REPORT
  uint8_t i;
  uint8_t tx_buf[] = "RFID:00000000\r\n";

  for (i = 0U; i < RFID_UID_LEN; i++)
  {
    tx_buf[5U + (2U * i)] = (uint8_t)HexDigit((uint8_t)((uid[i] >> 4U) & 0x0FU));
    tx_buf[6U + (2U * i)] = (uint8_t)HexDigit((uint8_t)(uid[i] & 0x0FU));
  }

  UART3_SendAsync(tx_buf, (uint16_t)(sizeof(tx_buf) - 1U));
#else
  (void)uid;
#endif
}

static void RFID_SendNoCard(void)
{
#if ENABLE_RFID_UART_REPORT
  uint8_t tx_buf[] = "RFID:NONE\r\n";

  UART3_SendAsync(tx_buf, (uint16_t)(sizeof(tx_buf) - 1U));
#endif
}

static void HCSR04_InitPins(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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

static void HCSR04_DelayUs(uint16_t us)
{
  uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) - start) < us)
  {
  }
}

static void HCSR04_StartMeasure(void)
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
  HCSR04_DelayUs(2U);
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
  HCSR04_DelayUs(10U);
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
  hcsr04_trigger_tick = HAL_GetTick();
}

static void Ultrasonic_SendDistance(uint16_t distance_cm)
{
#if ENABLE_ULTRASONIC_UART_REPORT
  uint8_t tx_buf[] = "US:000cm\r\n";

  if (distance_cm > 999U)
  {
    distance_cm = 999U;
  }

  tx_buf[3] = (uint8_t)('0' + ((distance_cm / 100U) % 10U));
  tx_buf[4] = (uint8_t)('0' + ((distance_cm / 10U) % 10U));
  tx_buf[5] = (uint8_t)('0' + (distance_cm % 10U));

  UART3_SendAsync(tx_buf, (uint16_t)(sizeof(tx_buf) - 1U));
#else
  (void)distance_cm;
#endif
}

static void Ultrasonic_SendNoEcho(void)
{
#if ENABLE_ULTRASONIC_UART_REPORT
  uint8_t tx_buf[] = "US:NONE\r\n";

  UART3_SendAsync(tx_buf, (uint16_t)(sizeof(tx_buf) - 1U));
#endif
}

static uint8_t IR_ReadMask(void)
{
  uint8_t mask = 0U;

  if (HAL_GPIO_ReadPin(IR1_GPIO_Port, IR1_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x01U;
  }
  if (HAL_GPIO_ReadPin(IR2_GPIO_Port, IR2_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x02U;
  }
  if (HAL_GPIO_ReadPin(IR3_GPIO_Port, IR3_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x04U;
  }
  if (HAL_GPIO_ReadPin(IR4_GPIO_Port, IR4_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x08U;
  }
  if (HAL_GPIO_ReadPin(IR5_GPIO_Port, IR5_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x10U;
  }

  return mask;
}

static void IR_SendState(uint8_t mask)
{
  uint8_t tx_buf[] = "IR:0,0,0,0,0\r\n";

  tx_buf[3] = (uint8_t)('0' + ((mask >> 0U) & 0x01U));
  tx_buf[5] = (uint8_t)('0' + ((mask >> 1U) & 0x01U));
  tx_buf[7] = (uint8_t)('0' + ((mask >> 2U) & 0x01U));
  tx_buf[9] = (uint8_t)('0' + ((mask >> 3U) & 0x01U));
  tx_buf[11] = (uint8_t)('0' + ((mask >> 4U) & 0x01U));
  UART3_SendAsync(tx_buf, (uint16_t)(sizeof(tx_buf) - 1U));
}

static void UART3_SendAsync(const uint8_t *data, uint16_t len)
{
  uint16_t send_len;

  if ((data == 0) || (len == 0U))
  {
    return;
  }

  send_len = len;
  if (send_len > (uint16_t)sizeof(uart3_tx_active_buf))
  {
    send_len = (uint16_t)sizeof(uart3_tx_active_buf);
  }

  __disable_irq();
  if ((uart3_tx_busy == 0U) && (huart3.gState == HAL_UART_STATE_READY))
  {
    memcpy(uart3_tx_active_buf, data, send_len);
    uart3_tx_active_len = send_len;
    uart3_tx_busy = 1U;
    __enable_irq();

    if (HAL_UART_Transmit_IT(&huart3, uart3_tx_active_buf, uart3_tx_active_len) != HAL_OK)
    {
      __disable_irq();
      uart3_tx_busy = 0U;
      __enable_irq();
    }
    return;
  }

  memcpy(uart3_tx_pending_buf, data, send_len);
  uart3_tx_pending_len = send_len;
  uart3_tx_pending = 1U;
  __enable_irq();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET); //STBY设置为高，使能驱动
  RC522_Init();
  HCSR04_InitPins();
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    const float duty1 = MOTOR_DEFAULT_DUTY;
    const float duty2 = MOTOR_DEFAULT_DUTY;
    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint16_t ccr1 = (uint16_t)(duty1 * (float)(arr + 1U));
    uint16_t ccr2 = (uint16_t)(duty2 * (float)(arr + 1U));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);

    if ((HAL_GetTick() - last_rfid_poll_tick) >= RFID_POLL_INTERVAL_MS)
    {
      uint8_t uid[4];

      last_rfid_poll_tick = HAL_GetTick();
      if (RC522_ReadUid(uid) != 0U)
      {
        if ((has_last_uid == 0U) || (UidEquals(last_uid, uid) == 0U))
        {
          UidCopy(last_uid, uid);
          has_last_uid = 1U;
          RFID_SendUid(uid);
        }
      }
      else if (has_last_uid != 0U)
      {
        has_last_uid = 0U;
        RFID_SendNoCard();
      }
    }

    if ((HAL_GetTick() - last_ultrasonic_poll_tick) >= HCSR04_MEASURE_INTERVAL_MS)
    {
      last_ultrasonic_poll_tick = HAL_GetTick();
      HCSR04_StartMeasure();
    }

    if ((hcsr04_busy != 0U) && ((HAL_GetTick() - hcsr04_trigger_tick) >= HCSR04_TIMEOUT_MS))
    {
      hcsr04_busy = 0U;
      hcsr04_timeout_flag = 1U;
    }

    if (hcsr04_result_ready != 0U)
    {
      uint16_t distance_cm;

      hcsr04_result_ready = 0U;
      distance_cm = (uint16_t)(hcsr04_pulse_width_us / HCSR04_CM_PER_US_DIVISOR);
      if ((has_last_distance == 0U) || (distance_cm != last_distance_cm))
      {
        last_distance_cm = distance_cm;
        has_last_distance = 1U;
        Ultrasonic_SendDistance(distance_cm);
      }
    }
    else if (hcsr04_timeout_flag != 0U)
    {
      hcsr04_timeout_flag = 0U;
      if (has_last_distance != 0U)
      {
        has_last_distance = 0U;
        Ultrasonic_SendNoEcho();
      }
    }

    if ((HAL_GetTick() - last_ir_report_tick) >= IR_REPORT_INTERVAL_MS)
    {
      last_ir_report_tick = HAL_GetTick();
      IR_SendState(IR_ReadMask());
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != HCSR04_ECHO_Pin)
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3)
  {
    return;
  }

  if (uart3_tx_pending != 0U)
  {
    __disable_irq();
    memcpy(uart3_tx_active_buf, uart3_tx_pending_buf, uart3_tx_pending_len);
    uart3_tx_active_len = uart3_tx_pending_len;
    uart3_tx_pending = 0U;
    uart3_tx_busy = 1U;
    __enable_irq();

    if (HAL_UART_Transmit_IT(&huart3, uart3_tx_active_buf, uart3_tx_active_len) != HAL_OK)
    {
      uart3_tx_busy = 0U;
    }
    return;
  }

  uart3_tx_busy = 0U;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
