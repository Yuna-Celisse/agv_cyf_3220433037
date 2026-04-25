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

#include "rc522.h"
#include "oled.h"

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
#define ENABLE_RFID_UART_REPORT 1U
#define ENABLE_ULTRASONIC_UART_REPORT 0U
#define RFID_POLL_INTERVAL_MS 100U
#define CARD_STANDBY_DELAY_MS 5000U
#define HCSR04_MEASURE_INTERVAL_MS 100U
#define HCSR04_TIMEOUT_MS 35U
#define HCSR04_CM_PER_US_DIVISOR 58U
/* Duty values are in per-mille: 0..1000 */
#define LINE_BASE_DUTY_PM 160U
#define LINE_MIN_DUTY_PM 50U
#define LINE_MAX_DUTY_PM 650U
#define LINE_PID_INTERVAL_MS 10U   /* control period */
/* Integer proportional gain: correction_pm = error_x10 * LINE_KP_PM_PER_ERR10 */
#define LINE_KP_PM_PER_ERR10 8
#define IR_ACTIVE_LOW 1U
#define SERVO_FRAME_MS 20U
#define SERVO_MIN_PULSE_US 500U
#define SERVO_MAX_PULSE_US 2500U
#define SERVO_DEFAULT_ANGLE_DEG 30U
#define SERVO_ENABLE 1U
#define STOP_DURATION_MS 5000U
#define SERVO_STOP_START_ANGLE_DEG 30U
#define SERVO_STOP_END_ANGLE_DEG 90U
#define LINE_BLACK_CONFIRM_FRAMES 3U
#define OBSTACLE_THRESHOLD_CM 15U
#define AVOID_RIGHT_MS 450U
#define AVOID_LEFT_MS 450U
#define AVOID_DUTY_FAST_PM 220U
#define AVOID_DUTY_SLOW_PM 80U

#define CARD_ID_NONE 0U
#define CARD_ID_A 1U
#define CARD_ID_B 2U

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
static uint8_t oled_line[17] = {0};
static uint8_t oled_last_line[17] = {0};
static uint8_t oled_rfid_line[17] = {0};
static uint8_t oled_last_rfid_line[17] = {0};
static uint32_t last_line_pid_tick = 0U;
static uint16_t servo_pulse_us = 1500U;

typedef enum
{
  VEHICLE_WAIT_CARD = 0,
  VEHICLE_CARD_STANDBY,
  VEHICLE_LINE_FOLLOW,
  VEHICLE_STOPPING,
  VEHICLE_AVOID_RIGHT,
  VEHICLE_AVOID_LEFT
} VehicleState_t;

static VehicleState_t vehicle_state = VEHICLE_WAIT_CARD;
static uint8_t target_card = CARD_ID_NONE;
static uint8_t target_stop_line = 0U;
static uint8_t crossed_line_count = 0U;
static uint8_t line_black_frame_count = 0U;
static uint8_t line_black_seen_mask = 0U;
static uint8_t line_cross_latched = 0U;
static uint32_t state_start_tick = 0U;
static uint32_t stop_start_tick = 0U;

static const uint8_t uid_card_a[RFID_UID_LEN] = {0x16U, 0x15U, 0x12U, 0x07U};
static const uint8_t uid_card_b[RFID_UID_LEN] = {0x1EU, 0xF1U, 0x2BU, 0x07U};

#define OLED_TEXT_X_OFFSET 16U

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static char HexDigit(uint8_t value);
static uint8_t UidEquals(const uint8_t left[4], const uint8_t right[4]);
static void UidCopy(uint8_t dst[4], const uint8_t src[4]);
static void RFID_SendUid(const uint8_t uid[4]);
static void HCSR04_InitPins(void);
static void HCSR04_DelayUs(uint16_t us);
static void HCSR04_StartMeasure(void);
static void Ultrasonic_SendDistance(uint16_t distance_cm);
static void Ultrasonic_SendNoEcho(void);
static uint8_t IR_ReadMask(void);
static uint8_t IR_NormalizeMask(uint8_t raw_mask);
static uint8_t IR_ComputeErrorX10(uint8_t raw_mask, int16_t *error_x10);
static void Servo_Init(void);
static void Servo_SetAngle(uint16_t angle_deg);
static void Servo_Task(void);
static void Motor_SetEnable(uint8_t enable);
static void Motor_SetForwardDirection(void);
static void Motor_SetDuty(uint16_t duty_left_pm, uint16_t duty_right_pm);
static void OLED_PushLine(const uint8_t *line, uint8_t len);
static void OLED_PushRfidLine(const uint8_t *line, uint8_t len);
static void OLED_RefreshMainLine(void);
static void OLED_RefreshRfidLine(void);
static void OLED_UpdateMainLine(void);
static void OLED_UpdateRfidLine(void);
static void OLED_ShowSwipePrompt(void);
static void OLED_ShowTarget(uint8_t card_id);
static void OLED_ShowDistance(void);
static uint8_t ResolveCardId(const uint8_t uid[4]);
static void StartMission(uint8_t card_id);
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
  uint8_t i;
  uint8_t display_buf[] = "IC:00000000";
  uint8_t changed = 0U;

  for (i = 0U; i < RFID_UID_LEN; i++)
  {
    display_buf[3U + (2U * i)] = (uint8_t)HexDigit((uint8_t)((uid[i] >> 4U) & 0x0FU));
    display_buf[4U + (2U * i)] = (uint8_t)HexDigit((uint8_t)(uid[i] & 0x0FU));
  }

  OLED_PushRfidLine(display_buf, 11U);

  for (i = 0U; i < 16U; i++)
  {
    if (oled_rfid_line[i] != oled_last_rfid_line[i])
    {
      changed = 1U;
      break;
    }
  }

  if (changed != 0U)
  {
    for (i = 0U; i < 17U; i++)
    {
      oled_last_rfid_line[i] = oled_rfid_line[i];
    }
    OLED_RefreshRfidLine();
  }

#if ENABLE_RFID_UART_REPORT
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

static uint8_t IR_NormalizeMask(uint8_t raw_mask)
{
#if IR_ACTIVE_LOW
  return (uint8_t)((~raw_mask) & 0x1FU);
#else
  return (uint8_t)(raw_mask & 0x1FU);
#endif
}

static uint8_t IR_ComputeErrorX10(uint8_t raw_mask, int16_t *error_x10)
{
  uint8_t mask;
  int8_t i;
  int16_t weighted_sum = 0;
  int16_t hit_count = 0;
  static const int8_t weights[5] = {-2, -1, 0, 1, 2};

  if (error_x10 == 0)
  {
    return 0U;
  }

  mask = IR_NormalizeMask(raw_mask);

  for (i = 0; i < 5; i++)
  {
    if ((mask & (uint8_t)(1U << i)) != 0U)
    {
      weighted_sum = (int16_t)(weighted_sum + weights[i]);
      hit_count++;
    }
  }

  if (hit_count == 0)
  {
    return 0U;
  }

  *error_x10 = (int16_t)((weighted_sum * 10) / hit_count);
  return 1U;
}

static void Servo_Init(void)
{
#if (SERVO_ENABLE == 0U)
  return;
#endif

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  Servo_SetAngle(SERVO_DEFAULT_ANGLE_DEG);
}

static void Servo_SetAngle(uint16_t angle_deg)
{
  uint16_t clamped_angle = angle_deg;
  uint32_t pulse_span = (uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);

  if (clamped_angle > 180U)
  {
    clamped_angle = 180U;
  }

  servo_pulse_us = (uint16_t)(SERVO_MIN_PULSE_US + ((pulse_span * clamped_angle) / 180U));
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_pulse_us);
}

static void Servo_Task(void)
{
  (void)servo_pulse_us;
}

static void Motor_SetEnable(uint8_t enable)
{
  if (enable != 0U)
  {
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    return;
  }

  Motor_SetDuty(0U, 0U);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
}

static void Motor_SetForwardDirection(void)
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

static void Motor_SetDuty(uint16_t duty_left_pm, uint16_t duty_right_pm)
{
  uint32_t arr = (uint32_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
  uint16_t ccr1;
  uint16_t ccr2;

  if (duty_left_pm > 1000U)
  {
    duty_left_pm = 1000U;
  }
  if (duty_right_pm > 1000U)
  {
    duty_right_pm = 1000U;
  }

  ccr1 = (uint16_t)(((arr + 1U) * duty_left_pm) / 1000U);
  ccr2 = (uint16_t)(((arr + 1U) * duty_right_pm) / 1000U);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
}

static void OLED_PushLine(const uint8_t *line, uint8_t len)
{
  uint8_t i;

  if (line == 0)
  {
    return;
  }

  if (len > 16U)
  {
    len = 16U;
  }

  for (i = 0U; i < 17U; i++)
  {
    oled_line[i] = 0U;
  }

  for (i = 0U; i < len; i++)
  {
    oled_line[i] = line[i];
  }
  oled_line[len] = '\0';
}

static void OLED_PushRfidLine(const uint8_t *line, uint8_t len)
{
  uint8_t i;

  if (line == 0)
  {
    return;
  }

  if (len > 16U)
  {
    len = 16U;
  }

  for (i = 0U; i < 17U; i++)
  {
    oled_rfid_line[i] = 0U;
  }

  for (i = 0U; i < len; i++)
  {
    oled_rfid_line[i] = line[i];
  }
  oled_rfid_line[len] = '\0';
}

static void OLED_RefreshMainLine(void)
{
  OLED_ClearPage(0U);
  OLED_ClearPage(1U);
  OLED_ShowString(OLED_TEXT_X_OFFSET, 0U, oled_line, 16U);
  OLED_RefreshPage(0U);
  OLED_RefreshPage(1U);
}

static void OLED_RefreshRfidLine(void)
{
  OLED_ClearPage(2U);
  OLED_ClearPage(3U);
  OLED_ShowString(OLED_TEXT_X_OFFSET, 16U, oled_rfid_line, 16U);
  OLED_RefreshPage(2U);
  OLED_RefreshPage(3U);
}

static void OLED_UpdateMainLine(void)
{
  uint8_t i;

  for (i = 0U; i < 16U; i++)
  {
    if (oled_line[i] != oled_last_line[i])
    {
      for (i = 0U; i < 17U; i++)
      {
        oled_last_line[i] = oled_line[i];
      }
      OLED_RefreshMainLine();
      return;
    }
  }
}

static void OLED_UpdateRfidLine(void)
{
  uint8_t i;

  for (i = 0U; i < 16U; i++)
  {
    if (oled_rfid_line[i] != oled_last_rfid_line[i])
    {
      for (i = 0U; i < 17U; i++)
      {
        oled_last_rfid_line[i] = oled_rfid_line[i];
      }
      OLED_RefreshRfidLine();
      return;
    }
  }
}

static void OLED_ShowSwipePrompt(void)
{
  OLED_PushLine((const uint8_t *)"SWIPE CARD", 10U);
  OLED_PushRfidLine((const uint8_t *)"TARGET: -", 9U);
  OLED_UpdateMainLine();
  OLED_UpdateRfidLine();
}

static void OLED_ShowTarget(uint8_t card_id)
{
  if (card_id == CARD_ID_A)
  {
    OLED_PushRfidLine((const uint8_t *)"TARGET: A", 9U);
  }
  else if (card_id == CARD_ID_B)
  {
    OLED_PushRfidLine((const uint8_t *)"TARGET: B", 9U);
  }
  else
  {
    OLED_PushRfidLine((const uint8_t *)"TARGET: -", 9U);
  }

  OLED_UpdateRfidLine();
}

static void OLED_ShowDistance(void)
{
  uint8_t line_buf[] = "US:---cm";

  if (has_last_distance != 0U)
  {
    uint16_t distance_cm = last_distance_cm;

    if (distance_cm > 999U)
    {
      distance_cm = 999U;
    }

    line_buf[3] = (uint8_t)('0' + ((distance_cm / 100U) % 10U));
    line_buf[4] = (uint8_t)('0' + ((distance_cm / 10U) % 10U));
    line_buf[5] = (uint8_t)('0' + (distance_cm % 10U));
  }

  OLED_PushLine(line_buf, 8U);
  OLED_UpdateMainLine();
}

static uint8_t ResolveCardId(const uint8_t uid[4])
{
  if (uid == 0)
  {
    return CARD_ID_NONE;
  }

  if (UidEquals(uid, uid_card_a) != 0U)
  {
    return CARD_ID_A;
  }

  if (UidEquals(uid, uid_card_b) != 0U)
  {
    return CARD_ID_B;
  }

  return CARD_ID_NONE;
}

static void StartMission(uint8_t card_id)
{
  if ((card_id != CARD_ID_A) && (card_id != CARD_ID_B))
  {
    return;
  }

  target_card = card_id;
  target_stop_line = (card_id == CARD_ID_A) ? 1U : 2U;
  crossed_line_count = 0U;
  line_black_frame_count = 0U;
  line_black_seen_mask = 0U;
  line_cross_latched = 0U;
  last_line_pid_tick = 0U;
  vehicle_state = VEHICLE_CARD_STANDBY;
  state_start_tick = HAL_GetTick();
  Motor_SetForwardDirection();
  Motor_SetEnable(0U);
  Motor_SetDuty(0U, 0U);
  Servo_SetAngle(SERVO_STOP_START_ANGLE_DEG);
  OLED_ShowTarget(card_id);
}

static void UART3_SendAsync(const uint8_t *data, uint16_t len)
{
  if ((data == 0) || (len == 0U))
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 20U);
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
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_ShowSwipePrompt();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  Motor_SetForwardDirection();
  Motor_SetEnable(0U);
  RC522_Init();
  HCSR04_InitPins();
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  Servo_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t ir_mask = IR_ReadMask();
    uint32_t now = HAL_GetTick();

    Servo_Task();

    if ((now - last_rfid_poll_tick) >= RFID_POLL_INTERVAL_MS)
    {
      uint8_t uid[4];

      last_rfid_poll_tick = now;
      if (RC522_ReadUid(uid) != 0U)
      {
        if ((has_last_uid == 0U) || (UidEquals(last_uid, uid) == 0U))
        {
          uint8_t card_id;

          UidCopy(last_uid, uid);
          has_last_uid = 1U;
          RFID_SendUid(uid);
          card_id = ResolveCardId(uid);

          if ((vehicle_state == VEHICLE_WAIT_CARD) && ((card_id == CARD_ID_A) || (card_id == CARD_ID_B)))
          {
            StartMission(card_id);
          }
        }
      }
      else
      {
        has_last_uid = 0U;
      }
    }

    if ((now - last_ultrasonic_poll_tick) >= HCSR04_MEASURE_INTERVAL_MS)
    {
      last_ultrasonic_poll_tick = now;
      HCSR04_StartMeasure();
    }

    if ((hcsr04_busy != 0U) && ((now - hcsr04_trigger_tick) >= HCSR04_TIMEOUT_MS))
    {
      hcsr04_busy = 0U;
      hcsr04_timeout_flag = 1U;
    }

    if (hcsr04_result_ready != 0U)
    {
      uint16_t distance_cm;

      hcsr04_result_ready = 0U;
      distance_cm = (uint16_t)(hcsr04_pulse_width_us / HCSR04_CM_PER_US_DIVISOR);
      last_distance_cm = distance_cm;
      has_last_distance = 1U;
    }
    else if (hcsr04_timeout_flag != 0U)
    {
      hcsr04_timeout_flag = 0U;
      has_last_distance = 0U;
    }

    switch (vehicle_state)
    {
      case VEHICLE_WAIT_CARD:
        Motor_SetEnable(0U);
        Servo_SetAngle(SERVO_STOP_START_ANGLE_DEG);
        OLED_ShowSwipePrompt();
        break;

      case VEHICLE_CARD_STANDBY:
        Motor_SetEnable(0U);
        Motor_SetDuty(0U, 0U);
        Servo_SetAngle(SERVO_STOP_START_ANGLE_DEG);
        if ((now - state_start_tick) >= CARD_STANDBY_DELAY_MS)
        {
          vehicle_state = VEHICLE_LINE_FOLLOW;
        }
        break;

      case VEHICLE_LINE_FOLLOW:
      {
        uint8_t norm_mask = IR_NormalizeMask(ir_mask);

        if ((has_last_distance != 0U) && (last_distance_cm <= OBSTACLE_THRESHOLD_CM))
        {
          vehicle_state = VEHICLE_AVOID_RIGHT;
          state_start_tick = now;
          break;
        }

        if ((now - last_line_pid_tick) >= LINE_PID_INTERVAL_MS)
        {
          int16_t error_x10 = 0;
          int16_t correction_pm;
          int32_t duty_left_pm;
          int32_t duty_right_pm;

          last_line_pid_tick = now;

          if (IR_ComputeErrorX10(ir_mask, &error_x10) == 0U)
          {
            error_x10 = 0;
          }

          correction_pm = (int16_t)(error_x10 * LINE_KP_PM_PER_ERR10);
          duty_left_pm = (int32_t)LINE_BASE_DUTY_PM + correction_pm;
          duty_right_pm = (int32_t)LINE_BASE_DUTY_PM - correction_pm;

          if (duty_left_pm < (int32_t)LINE_MIN_DUTY_PM)
          {
            duty_left_pm = LINE_MIN_DUTY_PM;
          }
          else if (duty_left_pm > (int32_t)LINE_MAX_DUTY_PM)
          {
            duty_left_pm = LINE_MAX_DUTY_PM;
          }

          if (duty_right_pm < (int32_t)LINE_MIN_DUTY_PM)
          {
            duty_right_pm = LINE_MIN_DUTY_PM;
          }
          else if (duty_right_pm > (int32_t)LINE_MAX_DUTY_PM)
          {
            duty_right_pm = LINE_MAX_DUTY_PM;
          }

          Motor_SetEnable(1U);
          Motor_SetDuty((uint16_t)duty_left_pm, (uint16_t)duty_right_pm);

          if (line_cross_latched == 0U)
          {
            if (norm_mask != 0U)
            {
              if (line_black_frame_count < 255U)
              {
                line_black_frame_count++;
              }
              line_black_seen_mask |= norm_mask;

              if ((line_black_frame_count >= LINE_BLACK_CONFIRM_FRAMES) && (line_black_seen_mask == 0x1FU))
              {
                crossed_line_count++;
                line_cross_latched = 1U;
                line_black_frame_count = 0U;
                line_black_seen_mask = 0U;

                if ((target_stop_line != 0U) && (crossed_line_count >= target_stop_line))
                {
                  vehicle_state = VEHICLE_STOPPING;
                  stop_start_tick = now;
                  Motor_SetEnable(0U);
                  Motor_SetDuty(0U, 0U);
                }
              }
            }
            else
            {
              line_black_frame_count = 0U;
              line_black_seen_mask = 0U;
            }
          }
          else if (norm_mask != 0x1FU)
          {
            line_cross_latched = 0U;
            line_black_frame_count = 0U;
            line_black_seen_mask = 0U;
          }
        }
        OLED_ShowDistance();
        break;
      }

      case VEHICLE_STOPPING:
      {
        uint32_t elapsed = now - stop_start_tick;
        uint16_t angle;

        Motor_SetEnable(0U);
        Motor_SetDuty(0U, 0U);

        if (elapsed >= STOP_DURATION_MS)
        {
          angle = SERVO_STOP_END_ANGLE_DEG;
          Servo_SetAngle(angle);
          vehicle_state = VEHICLE_WAIT_CARD;
          target_card = CARD_ID_NONE;
          target_stop_line = 0U;
          crossed_line_count = 0U;
          line_black_frame_count = 0U;
          line_black_seen_mask = 0U;
          line_cross_latched = 0U;
        }
        else
        {
          angle = (uint16_t)(SERVO_STOP_START_ANGLE_DEG +
            (((uint32_t)(SERVO_STOP_END_ANGLE_DEG - SERVO_STOP_START_ANGLE_DEG) * elapsed) / STOP_DURATION_MS));
          Servo_SetAngle(angle);
        }

        break;
      }

      case VEHICLE_AVOID_RIGHT:
        Motor_SetEnable(1U);
        Motor_SetDuty(AVOID_DUTY_FAST_PM, AVOID_DUTY_SLOW_PM);
        OLED_ShowDistance();
        if ((now - state_start_tick) >= AVOID_RIGHT_MS)
        {
          vehicle_state = VEHICLE_AVOID_LEFT;
          state_start_tick = now;
        }
        break;

      case VEHICLE_AVOID_LEFT:
        Motor_SetEnable(1U);
        Motor_SetDuty(AVOID_DUTY_SLOW_PM, AVOID_DUTY_FAST_PM);
        OLED_ShowDistance();
        if ((now - state_start_tick) >= AVOID_LEFT_MS)
        {
          vehicle_state = VEHICLE_LINE_FOLLOW;
        }
        break;

      default:
        vehicle_state = VEHICLE_WAIT_CARD;
        break;
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
