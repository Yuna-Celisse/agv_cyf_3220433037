#include "app_servo.h"

#include "main.h"
#include "tim.h"

#define APP_SERVO_MIN_PULSE_US 500U
#define APP_SERVO_MAX_PULSE_US 2500U

static uint16_t s_servo_pulse_us = 1500U;

void AppServo_Init(uint16_t default_angle_deg)
{
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  AppServo_SetAngle(default_angle_deg);
}

void AppServo_SetAngle(uint16_t angle_deg)
{
  uint16_t clamped_angle = angle_deg;
  uint32_t pulse_span = (uint32_t)(APP_SERVO_MAX_PULSE_US - APP_SERVO_MIN_PULSE_US);

  if (clamped_angle > 180U)
  {
    clamped_angle = 180U;
  }

  s_servo_pulse_us = (uint16_t)(APP_SERVO_MIN_PULSE_US + ((pulse_span * (uint32_t)(180U - clamped_angle)) / 180U));
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, s_servo_pulse_us);
}

void AppServo_Task(void)
{
  (void)s_servo_pulse_us;
}
