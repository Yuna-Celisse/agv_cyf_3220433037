#include "app_servo.h"

#include "main.h"
#include "tim.h"

#define APP_SERVO_MIN_PULSE_US 500U  /* 舵机 0 度附近脉宽（us） */
#define APP_SERVO_MAX_PULSE_US 2500U /* 舵机 180 度附近脉宽（us） */

static uint16_t s_servo_pulse_us = 1500U; /* 当前输出脉宽缓存 */

void AppServo_Init(uint16_t default_angle_deg)
{
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  AppServo_SetAngle(default_angle_deg); /* 上电后设置默认角度 */
}

void AppServo_SetAngle(uint16_t angle_deg)
{
  uint16_t clamped_angle = angle_deg; /* 可修改的角度副本 */
  uint32_t pulse_span = (uint32_t)(APP_SERVO_MAX_PULSE_US - APP_SERVO_MIN_PULSE_US); /* 有效脉宽范围 */

  if (clamped_angle > 180U)
  {
    clamped_angle = 180U; /* 输入限幅到机械常见范围 */
  }

  s_servo_pulse_us = (uint16_t)(APP_SERVO_MIN_PULSE_US + ((pulse_span * (uint32_t)(180U - clamped_angle)) / 180U)); /* 角度线性映射到脉宽 */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, s_servo_pulse_us); /* 更新 PWM 比较值 */
}

void AppServo_Task(void)
{
  (void)s_servo_pulse_us; /* 预留周期任务接口，当前仅占位 */
}
