#include "app_motor.h"

#include "main.h"
#include "tim.h"

void AppMotor_SetEnable(uint8_t enable)
{
  if (enable != 0U)
  {
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    return;
  }

  AppMotor_SetDuty(0U, 0U);
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

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
}
