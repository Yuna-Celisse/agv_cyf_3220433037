#include "app_motor.h"

#include "app_encoder.h"
#include "main.h"
#include "tim.h"

#define MOTOR_CONTROL_INTERVAL_MS 20U
#define MOTOR_MAX_TARGET_PPS 900
#define MOTOR_MIN_ACTIVE_DUTY_PM 40
#define MOTOR_MAX_DUTY_PM 500
#define MOTOR_KP_NUM 3
#define MOTOR_KI_NUM 1
#define MOTOR_GAIN_DEN 8
#define MOTOR_INTEGRAL_LIMIT 3000
#define MOTOR_SWAP_PWM_OUTPUTS 1U
#define MOTOR_NO_FEEDBACK_LIMIT 10U

static uint8_t s_closed_loop_enable = 0U;
static uint32_t s_last_control_tick = 0U;
static int32_t s_last_left_count = 0;
static int32_t s_last_right_count = 0;
static int16_t s_left_target_pps = 0;
static int16_t s_right_target_pps = 0;
static int32_t s_left_integral = 0;
static int32_t s_right_integral = 0;
static int32_t s_left_duty_pm = 0;
static int32_t s_right_duty_pm = 0;
static uint8_t s_left_no_feedback_count = 0U;
static uint8_t s_right_no_feedback_count = 0U;

static int32_t AppMotor_ClampI32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static void AppMotor_ResetLoopState(void)
{
  AppEncoder_Counts_t counts = AppEncoder_GetCounts();

  s_last_control_tick = HAL_GetTick();
  s_last_left_count = counts.left_count;
  s_last_right_count = counts.right_count;
  s_left_integral = 0;
  s_right_integral = 0;
  s_left_duty_pm = 0;
  s_right_duty_pm = 0;
  s_left_no_feedback_count = 0U;
  s_right_no_feedback_count = 0U;
}

void AppMotor_SetEnable(uint8_t enable)
{
  if (enable != 0U)
  {
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    return;
  }

  s_left_target_pps = 0;
  s_right_target_pps = 0;
  AppMotor_SetDuty(0U, 0U);
  AppMotor_ResetLoopState();
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
}

void AppMotor_SetForwardDirection(void)
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

void AppMotor_SetDuty(uint16_t duty_left_pm, uint16_t duty_right_pm)
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

#if MOTOR_SWAP_PWM_OUTPUTS
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr1);
#else
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
#endif
}

void AppMotor_SetClosedLoop(uint8_t enable)
{
  s_closed_loop_enable = (enable != 0U) ? 1U : 0U;
  AppMotor_ResetLoopState();
}

void AppMotor_SetTargetSpeed(int16_t left_pulse_per_sec, int16_t right_pulse_per_sec)
{
  s_left_target_pps = left_pulse_per_sec;
  s_right_target_pps = right_pulse_per_sec;
}

void AppMotor_SetTargetFromDuty(uint16_t duty_left_pm, uint16_t duty_right_pm)
{
  if (duty_left_pm > 1000U)
  {
    duty_left_pm = 1000U;
  }
  if (duty_right_pm > 1000U)
  {
    duty_right_pm = 1000U;
  }

  s_left_target_pps = (int16_t)(((int32_t)duty_left_pm * MOTOR_MAX_TARGET_PPS) / 1000);
  s_right_target_pps = (int16_t)(((int32_t)duty_right_pm * MOTOR_MAX_TARGET_PPS) / 1000);

  if (s_closed_loop_enable == 0U)
  {
    AppMotor_SetDuty(duty_left_pm, duty_right_pm);
  }
}

void AppMotor_Task(uint32_t now_ms)
{
  AppEncoder_Counts_t counts;
  uint32_t elapsed_ms;
  int32_t left_delta;
  int32_t right_delta;
  int32_t left_speed_pps;
  int32_t right_speed_pps;
  int32_t left_error;
  int32_t right_error;

  if (s_closed_loop_enable == 0U)
  {
    return;
  }

  elapsed_ms = now_ms - s_last_control_tick;
  if (elapsed_ms < MOTOR_CONTROL_INTERVAL_MS)
  {
    return;
  }

  counts = AppEncoder_GetCounts();
  left_delta = counts.left_count - s_last_left_count;
  right_delta = counts.right_count - s_last_right_count;
  s_last_left_count = counts.left_count;
  s_last_right_count = counts.right_count;
  s_last_control_tick = now_ms;

  left_speed_pps = (left_delta * 1000) / (int32_t)elapsed_ms;
  right_speed_pps = (right_delta * 1000) / (int32_t)elapsed_ms;

  if ((s_left_target_pps > 0) && (left_delta == 0) && (s_left_duty_pm > 200))
  {
    if (s_left_no_feedback_count < 255U)
    {
      s_left_no_feedback_count++;
    }
  }
  else
  {
    s_left_no_feedback_count = 0U;
  }

  if ((s_right_target_pps > 0) && (right_delta == 0) && (s_right_duty_pm > 200))
  {
    if (s_right_no_feedback_count < 255U)
    {
      s_right_no_feedback_count++;
    }
  }
  else
  {
    s_right_no_feedback_count = 0U;
  }

  if ((s_left_no_feedback_count >= MOTOR_NO_FEEDBACK_LIMIT) ||
      (s_right_no_feedback_count >= MOTOR_NO_FEEDBACK_LIMIT))
  {
    AppMotor_SetDuty(0U, 0U);
    AppMotor_ResetLoopState();
    return;
  }

  left_error = (int32_t)s_left_target_pps - left_speed_pps;
  right_error = (int32_t)s_right_target_pps - right_speed_pps;

  s_left_integral = AppMotor_ClampI32(s_left_integral + left_error,
                                      -MOTOR_INTEGRAL_LIMIT,
                                      MOTOR_INTEGRAL_LIMIT);
  s_right_integral = AppMotor_ClampI32(s_right_integral + right_error,
                                       -MOTOR_INTEGRAL_LIMIT,
                                       MOTOR_INTEGRAL_LIMIT);

  s_left_duty_pm += ((left_error * MOTOR_KP_NUM) + (s_left_integral * MOTOR_KI_NUM)) / MOTOR_GAIN_DEN;
  s_right_duty_pm += ((right_error * MOTOR_KP_NUM) + (s_right_integral * MOTOR_KI_NUM)) / MOTOR_GAIN_DEN;

  if (s_left_target_pps <= 0)
  {
    s_left_duty_pm = 0;
    s_left_integral = 0;
  }
  else
  {
    s_left_duty_pm = AppMotor_ClampI32(s_left_duty_pm, MOTOR_MIN_ACTIVE_DUTY_PM, MOTOR_MAX_DUTY_PM);
  }

  if (s_right_target_pps <= 0)
  {
    s_right_duty_pm = 0;
    s_right_integral = 0;
  }
  else
  {
    s_right_duty_pm = AppMotor_ClampI32(s_right_duty_pm, MOTOR_MIN_ACTIVE_DUTY_PM, MOTOR_MAX_DUTY_PM);
  }

  AppMotor_SetDuty((uint16_t)s_left_duty_pm, (uint16_t)s_right_duty_pm);
}
