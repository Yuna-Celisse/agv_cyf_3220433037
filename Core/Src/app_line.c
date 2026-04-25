#include "app_line.h"

#define APP_LINE_IR_ACTIVE_LOW 1U

uint8_t AppLine_ReadMask(void)
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

uint8_t AppLine_NormalizeMask(uint8_t raw_mask)
{
#if APP_LINE_IR_ACTIVE_LOW
  return (uint8_t)((~raw_mask) & 0x1FU);
#else
  return (uint8_t)(raw_mask & 0x1FU);
#endif
}

uint8_t AppLine_ComputeErrorX10(uint8_t raw_mask, int16_t *error_x10)
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

  mask = AppLine_NormalizeMask(raw_mask);

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
