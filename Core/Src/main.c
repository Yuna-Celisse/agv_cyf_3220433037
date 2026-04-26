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
#include "app_encoder.h"
#include "app_line.h"
#include "app_motor.h"
#include "app_servo.h"
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
#define CARD_STANDBY_DELAY_MS 2000U
#define HCSR04_MEASURE_INTERVAL_MS 100U
#define HCSR04_MOVE_INTERVAL_MS 140U
#define HCSR04_MAX_JUMP_CM 25U
#define HCSR04_INVALID_STOP_COUNT 3U
#define RESTORE_STAGE_US_ONLY 0U
#define RESTORE_STAGE_SENSOR_UI 1U
#define RESTORE_STAGE_FULL 2U
#define SYSTEM_RESTORE_STAGE RESTORE_STAGE_FULL
#define ENABLE_ULTRASONIC_ONLY ((SYSTEM_RESTORE_STAGE) == RESTORE_STAGE_US_ONLY)
#define ENABLE_NON_MOTION_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_SENSOR_UI)
#define ENABLE_MOTION_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_FULL)
#define ENABLE_RFID_FEATURES ((SYSTEM_RESTORE_STAGE) >= RESTORE_STAGE_FULL)
#define ENABLE_MOTOR_CLOSED_LOOP 0U
/* Duty values are in per-mille: 0..1000 */
#define LINE_BASE_DUTY_PM 160U
#define LINE_MIN_DUTY_PM 50U
#define LINE_MAX_DUTY_PM 650U
#define LINE_PID_INTERVAL_MS 10U   /* control period */
/* Integer proportional gain: correction_pm = error_x10 * LINE_KP_PM_PER_ERR10 */
#define LINE_KP_PM_PER_ERR10 8
#define SERVO_RUN_ANGLE_DEG 55U
#define SERVO_DEFAULT_ANGLE_DEG SERVO_RUN_ANGLE_DEG
#define SERVO_ENABLE 1U
#define SERVO_ROTATE_DURATION_MS 2000U
#define SERVO_HOLD_DURATION_MS 2000U
#define SERVO_STOP_START_ANGLE_DEG SERVO_RUN_ANGLE_DEG
#define SERVO_STOP_END_ANGLE_DEG 90U
#define LINE_BLACK_CONFIRM_FRAMES 3U
#define OBSTACLE_THRESHOLD_CM 20U
#define AVOID_RIGHT_MS 620U
#define AVOID_LEFT_MS 850U
#define AVOID_FORWARD_MS 180U
#define AVOID_RIGHT_ALIGN_MS 520U
#define AVOID_RETURN_FORWARD_MS 1400U
#define AVOID_RIGHT_SEARCH_MAX_MS 1600U
#define AVOID_LINE_CONFIRM_FRAMES 2U
#define AVOID_DUTY_FAST_PM 220U
#define AVOID_DUTY_SLOW_PM 80U
#define AVOID_RIGHT_DUTY_FAST_PM 260U
#define AVOID_RIGHT_DUTY_SLOW_PM 50U
#define AVOID_LEFT_DUTY_FAST_PM 260U
#define AVOID_LEFT_DUTY_SLOW_PM 50U
#define AVOID_FORWARD_DUTY_PM 230U
#define AVOID_RETURN_FORWARD_DUTY_PM 230U
#define AVOID_RIGHT_ALIGN_DUTY_FAST_PM 260U
#define AVOID_RIGHT_ALIGN_DUTY_SLOW_PM 80U

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
static uint8_t ultrasonic_invalid_count = 0U;
static uint32_t last_line_pid_tick = 0U;

typedef enum
{
  VEHICLE_WAIT_CARD = 0,
  VEHICLE_CARD_STANDBY,
  VEHICLE_LINE_FOLLOW,
  VEHICLE_STOPPING,
  VEHICLE_AVOID_RIGHT,
  VEHICLE_AVOID_LEFT,
  VEHICLE_AVOID_FORWARD,
  VEHICLE_AVOID_RIGHT_ALIGN,
  VEHICLE_AVOID_RETURN_FORWARD
} VehicleState_t;

static VehicleState_t vehicle_state = VEHICLE_WAIT_CARD;
static uint8_t target_card = CARD_ID_NONE;
static uint8_t target_stop_line = 0U;
static uint8_t crossed_line_count = 0U;
static uint8_t line_black_frame_count = 0U;
static uint8_t line_black_seen_mask = 0U;
static uint8_t line_cross_latched = 0U;
static uint8_t avoid_line_seen_count = 0U;
static uint32_t state_start_tick = 0U;
static uint32_t stop_start_tick = 0U;
static uint8_t stop_at_next_line = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void HCSR04_InitPins(void);
static void StartMission(uint8_t card_id);
static uint8_t IsUltrasonicMotionState(VehicleState_t state);
static void ResetUltrasonicState(void);
static void AppSetAvoidTurn(uint8_t turn_left, uint16_t fast_pm, uint16_t slow_pm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void HCSR04_InitPins(void)
{
  AppUltrasonic_Init(ENABLE_ULTRASONIC_UART_REPORT);
}

static uint8_t IsUltrasonicMotionState(VehicleState_t state)
{
  return (uint8_t)((state == VEHICLE_LINE_FOLLOW) ||
                   (state == VEHICLE_AVOID_RIGHT) ||
                   (state == VEHICLE_AVOID_LEFT) ||
                   (state == VEHICLE_AVOID_FORWARD) ||
                   (state == VEHICLE_AVOID_RIGHT_ALIGN) ||
                   (state == VEHICLE_AVOID_RETURN_FORWARD));
}

static void ResetUltrasonicState(void)
{
  last_distance_cm = 0U;
  has_last_distance = 0U;
  ultrasonic_jump_reject_count = 0U;
  ultrasonic_invalid_count = 0U;
}

static void AppSetAvoidTurn(uint8_t turn_left, uint16_t fast_pm, uint16_t slow_pm)
{
  if (turn_left != 0U)
  {
    AppMotor_SetTargetFromDuty(fast_pm, slow_pm);
  }
  else
  {
    AppMotor_SetTargetFromDuty(slow_pm, fast_pm);
  }
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
  avoid_line_seen_count = 0U;
  last_line_pid_tick = 0U;
  stop_at_next_line = 0U;
  ResetUltrasonicState();
  vehicle_state = VEHICLE_CARD_STANDBY;
  state_start_tick = HAL_GetTick();
  AppMotor_SetForwardDirection();
  AppMotor_SetEnable(0U);
  AppMotor_SetDuty(0U, 0U);
  AppServo_SetAngle(SERVO_STOP_START_ANGLE_DEG);
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
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  AppEncoder_Init();
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
  AppMotor_SetClosedLoop(ENABLE_MOTOR_CLOSED_LOOP);
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
  AppServo_Init(SERVO_DEFAULT_ANGLE_DEG);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t ir_mask = AppLine_ReadMask();
    uint32_t now = HAL_GetTick();

#if ENABLE_MOTION_FEATURES
  AppServo_Task();
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

    if (IsUltrasonicMotionState(vehicle_state) != 0U)
    {
      ultrasonic_interval_ms = HCSR04_MOVE_INTERVAL_MS;
    }
  #endif

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
              (IsUltrasonicMotionState(vehicle_state) != 0U))
          {
            uint16_t diff_cm = (distance_cm > last_distance_cm) ?
              (uint16_t)(distance_cm - last_distance_cm) :
              (uint16_t)(last_distance_cm - distance_cm);

            if (diff_cm > HCSR04_MAX_JUMP_CM)
            {
              if ((distance_cm < last_distance_cm) && (distance_cm <= OBSTACLE_THRESHOLD_CM))
              {
                ultrasonic_jump_reject_count = 0U;
              }
              else if (ultrasonic_jump_reject_count < 2U)
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
          ultrasonic_invalid_count = 0U;
        }
        else
        {
          has_last_distance = 0U;
#if ENABLE_MOTION_FEATURES
          if (IsUltrasonicMotionState(vehicle_state) != 0U)
          {
            if (ultrasonic_invalid_count < 255U)
            {
              ultrasonic_invalid_count++;
            }
          }
          else
          {
            ultrasonic_invalid_count = 0U;
          }
#else
          ultrasonic_invalid_count = 0U;
#endif
        }
      }
    }

    if ((ultrasonic_enabled != 0U) && ((now - last_ultrasonic_poll_tick) >= ultrasonic_interval_ms))
    {
      last_ultrasonic_poll_tick = now;
      AppUltrasonic_StartMeasure();
    }

#if !ENABLE_MOTION_FEATURES
  #if ENABLE_NON_MOTION_FEATURES
    AppOled_ShowDistance(has_last_distance, last_distance_cm);
  #endif
    continue;
#endif

    if ((IsUltrasonicMotionState(vehicle_state) != 0U) &&
        (ultrasonic_invalid_count >= HCSR04_INVALID_STOP_COUNT))
    {
      AppMotor_SetEnable(0U);
      AppMotor_SetDuty(0U, 0U);
      if (vehicle_state == VEHICLE_LINE_FOLLOW)
      {
        last_line_pid_tick = now;
      }
      else
      {
        state_start_tick = now;
      }
      AppOled_ShowDistance(has_last_distance, last_distance_cm);
      continue;
    }

    switch (vehicle_state)
    {
      case VEHICLE_WAIT_CARD:
        AppMotor_SetEnable(0U);
        AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
        AppOled_ShowSwipePrompt();
        break;

      case VEHICLE_CARD_STANDBY:
        AppMotor_SetEnable(0U);
        AppMotor_SetDuty(0U, 0U);
        AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
        if ((now - state_start_tick) >= CARD_STANDBY_DELAY_MS)
        {
          vehicle_state = VEHICLE_LINE_FOLLOW;
        }
        break;

      case VEHICLE_LINE_FOLLOW:
      {
        uint8_t norm_mask = AppLine_NormalizeMask(ir_mask);

        AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);

        if ((has_last_distance != 0U) && (last_distance_cm <= OBSTACLE_THRESHOLD_CM))
        {
          vehicle_state = VEHICLE_AVOID_LEFT;
          avoid_line_seen_count = 0U;
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
          duty_left_pm = (int32_t)LINE_BASE_DUTY_PM - correction_pm;
          duty_right_pm = (int32_t)LINE_BASE_DUTY_PM + correction_pm;

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
          AppMotor_SetTargetFromDuty((uint16_t)duty_left_pm, (uint16_t)duty_right_pm);

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

                  if (stop_at_next_line != 0U)
                  {
                    target_stop_line = 0U;
                  }
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

        if (stop_at_next_line != 0U)
        {
          AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
          vehicle_state = VEHICLE_WAIT_CARD;
          target_card = CARD_ID_NONE;
          target_stop_line = 0U;
          crossed_line_count = 0U;
          line_black_frame_count = 0U;
          line_black_seen_mask = 0U;
          line_cross_latched = 0U;
          stop_at_next_line = 0U;
        }
        else if (elapsed >= (SERVO_ROTATE_DURATION_MS + SERVO_HOLD_DURATION_MS))
        {
          AppServo_SetAngle(SERVO_RUN_ANGLE_DEG);
          stop_at_next_line = 1U;
          target_stop_line = (uint8_t)(crossed_line_count + 1U);
          line_black_frame_count = 0U;
          line_black_seen_mask = 0U;
          line_cross_latched = 0U;
          last_line_pid_tick = now;
          vehicle_state = VEHICLE_LINE_FOLLOW;
        }
        else if (elapsed >= SERVO_ROTATE_DURATION_MS)
        {
          AppServo_SetAngle(SERVO_STOP_END_ANGLE_DEG);
        }
        else
        {
          int32_t angle_delta = (int32_t)SERVO_STOP_END_ANGLE_DEG - (int32_t)SERVO_STOP_START_ANGLE_DEG;
          angle = (int16_t)((int32_t)SERVO_STOP_START_ANGLE_DEG +
            ((angle_delta * (int32_t)elapsed) / (int32_t)SERVO_ROTATE_DURATION_MS));
          AppServo_SetAngle((uint16_t)angle);
        }

        break;
      }

      case VEHICLE_AVOID_RIGHT:
      {
        uint8_t norm_mask = AppLine_NormalizeMask(ir_mask);
        uint32_t elapsed = now - state_start_tick;

        AppMotor_SetEnable(1U);
        AppSetAvoidTurn(0U, AVOID_RIGHT_DUTY_FAST_PM, AVOID_RIGHT_DUTY_SLOW_PM);
        AppOled_ShowDistance(has_last_distance, last_distance_cm);

        if (norm_mask != 0U)
        {
          if (avoid_line_seen_count < 255U)
          {
            avoid_line_seen_count++;
          }
        }
        else
        {
          avoid_line_seen_count = 0U;
        }

        if ((avoid_line_seen_count >= AVOID_LINE_CONFIRM_FRAMES) ||
            (elapsed >= AVOID_RIGHT_SEARCH_MAX_MS))
        {
          avoid_line_seen_count = 0U;
          line_black_frame_count = 0U;
          line_black_seen_mask = 0U;
          line_cross_latched = 0U;
          last_line_pid_tick = now;
          vehicle_state = VEHICLE_LINE_FOLLOW;
        }
        break;
      }

      case VEHICLE_AVOID_LEFT:
        AppMotor_SetEnable(1U);
        AppSetAvoidTurn(1U, AVOID_LEFT_DUTY_FAST_PM, AVOID_LEFT_DUTY_SLOW_PM);
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
        if ((now - state_start_tick) >= AVOID_LEFT_MS)
        {
          vehicle_state = VEHICLE_AVOID_FORWARD;
          state_start_tick = now;
        }
        break;

      case VEHICLE_AVOID_FORWARD:
        AppMotor_SetEnable(1U);
        AppMotor_SetTargetFromDuty(AVOID_FORWARD_DUTY_PM, AVOID_FORWARD_DUTY_PM);
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
        if ((now - state_start_tick) >= AVOID_FORWARD_MS)
        {
          vehicle_state = VEHICLE_AVOID_RIGHT_ALIGN;
          state_start_tick = now;
        }
        break;

      case VEHICLE_AVOID_RIGHT_ALIGN:
        AppMotor_SetEnable(1U);
        AppSetAvoidTurn(0U, AVOID_RIGHT_ALIGN_DUTY_FAST_PM, AVOID_RIGHT_ALIGN_DUTY_SLOW_PM);
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
        if ((now - state_start_tick) >= AVOID_RIGHT_ALIGN_MS)
        {
          vehicle_state = VEHICLE_AVOID_RETURN_FORWARD;
          state_start_tick = now;
        }
        break;

      case VEHICLE_AVOID_RETURN_FORWARD:
        AppMotor_SetEnable(1U);
        AppMotor_SetTargetFromDuty(AVOID_RETURN_FORWARD_DUTY_PM, AVOID_RETURN_FORWARD_DUTY_PM);
        AppOled_ShowDistance(has_last_distance, last_distance_cm);
        if ((now - state_start_tick) >= AVOID_RETURN_FORWARD_MS)
        {
          vehicle_state = VEHICLE_AVOID_RIGHT;
          avoid_line_seen_count = 0U;
          state_start_tick = now;
        }
        break;

      default:
        vehicle_state = VEHICLE_WAIT_CARD;
        break;
    }

    AppMotor_Task(now);

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
  AppEncoder_HandleExti(GPIO_Pin);
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
