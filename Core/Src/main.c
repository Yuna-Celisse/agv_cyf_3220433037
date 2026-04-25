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
#include "app_line.h"
#include "app_motor.h"
#include "app_oled_ui.h"
#include "app_ultrasonic.h"
#include "app_rfid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ENABLE_RFID_UART_REPORT 0U
#define ENABLE_ULTRASONIC_UART_REPORT 1U
#define CARD_STANDBY_DELAY_MS 5000U
#define HCSR04_MEASURE_INTERVAL_MS 100U
#define HCSR04_MOVE_INTERVAL_MS 140U
#define HCSR04_MAX_JUMP_CM 25U
#define RESTORE_STAGE_US_ONLY 0U
#define RESTORE_STAGE_SENSOR_UI 1U
#define RESTORE_STAGE_FULL 2U
#define SYSTEM_RESTORE_STAGE RESTORE_STAGE_FULL
#define ENABLE_ULTRASONIC_ONLY ((SYSTEM_RESTORE_STAGE) == RESTORE_STAGE_US_ONLY)
#define ENABLE_NON_MOTION_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_SENSOR_UI)
#define ENABLE_MOTION_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_FULL)
#define ENABLE_RFID_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_FULL)
/* Duty values are in per-mille: 0..1000 */
#define LINE_BASE_DUTY_PM 160U
#define LINE_MIN_DUTY_PM 50U
#define LINE_MAX_DUTY_PM 650U
#define LINE_PID_INTERVAL_MS 10U   /* control period */
/* Integer proportional gain: correction_pm = error_x10 * LINE_KP_PM_PER_ERR10 */
#define LINE_KP_PM_PER_ERR10 8
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
static uint32_t last_ultrasonic_poll_tick = 0U;
static uint16_t last_distance_cm = 0U;
static uint8_t has_last_distance = 0U;
static uint8_t ultrasonic_jump_reject_count = 0U;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void HCSR04_InitPins(void);
static void Servo_Init(void);
static void Servo_SetAngle(uint16_t angle_deg);
static void Servo_Task(void);
static void StartMission(uint8_t card_id);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void HCSR04_InitPins(void)
{
  AppUltrasonic_Init(ENABLE_ULTRASONIC_UART_REPORT);
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
  AppMotor_SetForwardDirection();
  AppMotor_SetEnable(0U);
  AppMotor_SetDuty(0U, 0U);
  Servo_SetAngle(SERVO_STOP_START_ANGLE_DEG);
  AppOled_ShowTarget(card_id);
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
#if ENABLE_NON_MOTION_FEATURES
  OLED_Init();
  AppOled_ShowSwipePrompt();
#if ENABLE_RFID_FEATURES
  AppRfid_Init();
#endif
#endif
#if ENABLE_MOTION_FEATURES
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  AppMotor_SetForwardDirection();
  AppMotor_SetEnable(0U);
#endif
  HCSR04_InitPins();
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
#if ENABLE_MOTION_FEATURES
  Servo_Init();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t ir_mask = AppLine_ReadMask();
    uint32_t now = HAL_GetTick();

#if ENABLE_MOTION_FEATURES
    Servo_Task();
#endif

#if ENABLE_NON_MOTION_FEATURES
#if ENABLE_RFID_FEATURES
    {
      AppRfidEvent_t rfid_event;
      uint8_t allow_rfid_poll = (uint8_t)((AppUltrasonic_IsBusy() == 0U) &&
                                          (vehicle_state == VEHICLE_WAIT_CARD) &&
                                          (target_card == CARD_ID_NONE));

      if (AppRfid_Poll(now, allow_rfid_poll, &rfid_event) != 0U)
      {
        if (rfid_event.is_new_uid != 0U)
        {
          uint8_t card_id;

          AppOled_ShowUid(rfid_event.uid, ENABLE_RFID_UART_REPORT);
          card_id = rfid_event.card_id;

          if ((vehicle_state == VEHICLE_WAIT_CARD) && ((card_id == CARD_ID_A) || (card_id == CARD_ID_B)))
          {
            StartMission(card_id);
          }
        }
      }
    }
#endif
#endif

    uint32_t ultrasonic_interval_ms = HCSR04_MEASURE_INTERVAL_MS;
    uint8_t ultrasonic_enabled = 1U;

  #if ENABLE_MOTION_FEATURES
    if (vehicle_state == VEHICLE_WAIT_CARD)
    {
      ultrasonic_enabled = 0U;
    }

    if ((vehicle_state == VEHICLE_LINE_FOLLOW) ||
      (vehicle_state == VEHICLE_AVOID_RIGHT) ||
      (vehicle_state == VEHICLE_AVOID_LEFT))
    {
      ultrasonic_interval_ms = HCSR04_MOVE_INTERVAL_MS;
    }
  #endif

    if ((ultrasonic_enabled != 0U) && ((now - last_ultrasonic_poll_tick) >= ultrasonic_interval_ms))
    {
      last_ultrasonic_poll_tick = now;
      AppUltrasonic_StartMeasure();
    }

    AppUltrasonic_Task(now);

    {
      uint16_t distance_cm;
      uint8_t has_distance = 0U;

      if (AppUltrasonic_FetchResult(&distance_cm, &has_distance) != 0U)
      {
        if (has_distance != 0U)
        {
#if ENABLE_MOTION_FEATURES
          if ((has_last_distance != 0U) &&
              ((vehicle_state == VEHICLE_LINE_FOLLOW) ||
               (vehicle_state == VEHICLE_AVOID_RIGHT) ||
               (vehicle_state == VEHICLE_AVOID_LEFT)))
          {
            uint16_t diff_cm = (distance_cm > last_distance_cm) ?
              (uint16_t)(distance_cm - last_distance_cm) :
              (uint16_t)(last_distance_cm - distance_cm);

            if (diff_cm > HCSR04_MAX_JUMP_CM)
            {
              if (ultrasonic_jump_reject_count < 2U)
              {
                ultrasonic_jump_reject_count++;
                distance_cm = last_distance_cm;
              }
              else
              {
                ultrasonic_jump_reject_count = 0U;
              }
            }
            else
            {
              ultrasonic_jump_reject_count = 0U;
            }
          }
          else
          {
            ultrasonic_jump_reject_count = 0U;
          }
#endif

          last_distance_cm = distance_cm;
          has_last_distance = 1U;
        }
        else
        {
          has_last_distance = 0U;
        }
      }
    }

#if !ENABLE_MOTION_FEATURES
  #if ENABLE_NON_MOTION_FEATURES
    AppOled_ShowDistance(has_last_distance, last_distance_cm);
  #endif
    continue;
#endif

    switch (vehicle_state)
    {
      case VEHICLE_WAIT_CARD:
        AppMotor_SetEnable(0U);
        Servo_SetAngle(SERVO_STOP_START_ANGLE_DEG);
        AppOled_ShowSwipePrompt();
        break;

      case VEHICLE_CARD_STANDBY:
        AppMotor_SetEnable(0U);
        AppMotor_SetDuty(0U, 0U);
        Servo_SetAngle(SERVO_STOP_START_ANGLE_DEG);
        if ((now - state_start_tick) >= CARD_STANDBY_DELAY_MS)
        {
          vehicle_state = VEHICLE_LINE_FOLLOW;
        }
        break;

      case VEHICLE_LINE_FOLLOW:
      {
        uint8_t norm_mask = AppLine_NormalizeMask(ir_mask);

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

          if (AppLine_ComputeErrorX10(ir_mask, &error_x10) == 0U)
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

          AppMotor_SetEnable(1U);
          AppMotor_SetDuty((uint16_t)duty_left_pm, (uint16_t)duty_right_pm);

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
                  AppMotor_SetEnable(0U);
                  AppMotor_SetDuty(0U, 0U);
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
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
        break;
      }

      case VEHICLE_STOPPING:
      {
        uint32_t elapsed = now - stop_start_tick;
        uint16_t angle;

        AppMotor_SetEnable(0U);
        AppMotor_SetDuty(0U, 0U);

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
        AppMotor_SetEnable(1U);
        AppMotor_SetDuty(AVOID_DUTY_FAST_PM, AVOID_DUTY_SLOW_PM);
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
        if ((now - state_start_tick) >= AVOID_RIGHT_MS)
        {
          vehicle_state = VEHICLE_AVOID_LEFT;
          state_start_tick = now;
        }
        break;

      case VEHICLE_AVOID_LEFT:
        AppMotor_SetEnable(1U);
        AppMotor_SetDuty(AVOID_DUTY_SLOW_PM, AVOID_DUTY_FAST_PM);
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
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
  AppUltrasonic_HandleEchoExti(GPIO_Pin);
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
