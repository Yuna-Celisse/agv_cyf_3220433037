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
#include "gpio.h"
#include "rc522.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define HCSR04_MEASURE_INTERVAL_MS 100U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t last_rfid_poll_tick = 0;
uint8_t last_uid[4] = {0};
uint8_t has_last_uid = 0;
uint32_t last_ultrasonic_poll_tick = 0;
uint16_t last_distance_cm = 0;
uint8_t has_last_distance = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void MX_USART3_UART_Init(void);
static char HexDigit(uint8_t value);
static uint8_t UidEquals(const uint8_t left[4], const uint8_t right[4]);
static void UidCopy(uint8_t dst[4], const uint8_t src[4]);
static void RFID_SendUid(const uint8_t uid[4]);
static void RFID_SendNoCard(void);
static void HCSR04_InitPins(void);
static void HCSR04_DelayUs(uint16_t us);
static uint8_t HCSR04_WaitPinState(GPIO_PinState target_state, uint32_t timeout_us);
static uint8_t HCSR04_ReadDistanceCm(uint16_t *distance_cm);
static void Ultrasonic_SendDistance(uint16_t distance_cm);
static void Ultrasonic_SendNoEcho(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

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

  for (i = 0U; i < 4U; i++)
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

  for (i = 0U; i < 4U; i++)
  {
    dst[i] = src[i];
  }
}

static void RFID_SendUid(const uint8_t uid[4])
{
  uint8_t i;
  uint8_t tx_buf[] = "RFID:00000000\r\n";

  for (i = 0U; i < 4U; i++)
  {
    tx_buf[5U + (2U * i)] = (uint8_t)HexDigit((uint8_t)((uid[i] >> 4U) & 0x0FU));
    tx_buf[6U + (2U * i)] = (uint8_t)HexDigit((uint8_t)(uid[i] & 0x0FU));
  }

  HAL_UART_Transmit(&huart3, tx_buf, (uint16_t)(sizeof(tx_buf) - 1U), 50U);
}

static void RFID_SendNoCard(void)
{
  uint8_t tx_buf[] = "RFID:NONE\r\n";

  HAL_UART_Transmit(&huart3, tx_buf, (uint16_t)(sizeof(tx_buf) - 1U), 50U);
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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HCSR04_ECHO_GPIO_Port, &GPIO_InitStruct);
}

static void HCSR04_DelayUs(uint16_t us)
{
  uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) - start) < us)
  {
  }
}

static uint8_t HCSR04_WaitPinState(GPIO_PinState target_state, uint32_t timeout_us)
{
  uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  while (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin) != target_state)
  {
    if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) - start) >= timeout_us)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t HCSR04_ReadDistanceCm(uint16_t *distance_cm)
{
  uint16_t pulse_start;
  uint32_t pulse_width_us;

  if (distance_cm == 0)
  {
    return 0U;
  }

  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
  HCSR04_DelayUs(2U);
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
  HCSR04_DelayUs(10U);
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

  if (HCSR04_WaitPinState(GPIO_PIN_SET, 30000U) == 0U)
  {
    return 0U;
  }

  pulse_start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  if (HCSR04_WaitPinState(GPIO_PIN_RESET, 30000U) == 0U)
  {
    return 0U;
  }

  pulse_width_us = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) - pulse_start);
  *distance_cm = (uint16_t)(pulse_width_us / 58U);
  return 1U;
}

static void Ultrasonic_SendDistance(uint16_t distance_cm)
{
  uint8_t tx_buf[] = "US:000cm\r\n";

  if (distance_cm > 999U)
  {
    distance_cm = 999U;
  }

  tx_buf[3] = (uint8_t)('0' + ((distance_cm / 100U) % 10U));
  tx_buf[4] = (uint8_t)('0' + ((distance_cm / 10U) % 10U));
  tx_buf[5] = (uint8_t)('0' + (distance_cm % 10U));

  HAL_UART_Transmit(&huart3, tx_buf, (uint16_t)(sizeof(tx_buf) - 1U), 50U);
}

static void Ultrasonic_SendNoEcho(void)
{
  uint8_t tx_buf[] = "US:NONE\r\n";

  HAL_UART_Transmit(&huart3, tx_buf, (uint16_t)(sizeof(tx_buf) - 1U), 50U);
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
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //开启TIM1通道1 PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //开启TIM1通道2 PWM
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);  //AIN1设置为高
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);  //AIN2设置为低
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);  //BIN1设置为高
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);  //BIN2设置为低
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
    float duty1 = 0.2;  //占空比1 左轮
    float duty2 = 0.2;  //占空比2 右轮
    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);  //不用动
    uint16_t ccr1 = duty1 * (arr + 1);
    uint16_t ccr2 = duty2 * (arr + 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);

    if ((HAL_GetTick() - last_rfid_poll_tick) >= 100U)
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
      uint16_t distance_cm;

      last_ultrasonic_poll_tick = HAL_GetTick();
      if (HCSR04_ReadDistanceCm(&distance_cm) != 0U)
      {
        if ((has_last_distance == 0U) || (distance_cm != last_distance_cm))
        {
          last_distance_cm = distance_cm;
          has_last_distance = 1U;
          Ultrasonic_SendDistance(distance_cm);
        }
      }
      else if (has_last_distance != 0U)
      {
        has_last_distance = 0U;
        Ultrasonic_SendNoEcho();
      }
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
