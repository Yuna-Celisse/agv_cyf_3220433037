#include "app_line.h"

/* Line sensors are read as a 5-bit mask and converted into a tracking error. */

#define APP_LINE_IR_ACTIVE_LOW 1U /* 红外巡线传感器是否低电平有效：1=低有效，0=高有效 */

/* Pack IR1..IR5 into bit0..bit4 for later processing. */
uint8_t AppLine_ReadMask(void)
{
  uint8_t mask = 0U; /* 5路红外状态位图，bit0~bit4 对应 IR1~IR5 */

  if (HAL_GPIO_ReadPin(IR1_GPIO_Port, IR1_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x01U; /* IR1 为高电平时置位 bit0 */
  }
  if (HAL_GPIO_ReadPin(IR2_GPIO_Port, IR2_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x02U; /* IR2 为高电平时置位 bit1 */
  }
  if (HAL_GPIO_ReadPin(IR3_GPIO_Port, IR3_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x04U; /* IR3 为高电平时置位 bit2 */
  }
  if (HAL_GPIO_ReadPin(IR4_GPIO_Port, IR4_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x08U; /* IR4 为高电平时置位 bit3 */
  }
  if (HAL_GPIO_ReadPin(IR5_GPIO_Port, IR5_Pin) == GPIO_PIN_SET)
  {
    mask |= 0x10U; /* IR5 为高电平时置位 bit4 */
  }

  return mask; /* 返回原始电平位图 */
}

/* Convert raw electrical level into a logical "line detected" bit mask. */
uint8_t AppLine_NormalizeMask(uint8_t raw_mask)
{
#if APP_LINE_IR_ACTIVE_LOW
  return (uint8_t)((~raw_mask) & 0x1FU); /* 低有效时取反并保留低5位，统一成“1=检测到黑线” */
#else
  return (uint8_t)(raw_mask & 0x1FU); /* 高有效时直接使用低5位 */
#endif
}

/* Compute a weighted average error without floating-point math. */
uint8_t AppLine_ComputeErrorX10(uint8_t raw_mask, int16_t *error_x10)
{
  uint8_t mask;
  int8_t i;
  int16_t weighted_sum = 0;                        /* 命中探头的权重和 */
  int16_t hit_count = 0;                           /* 命中探头个数 */
  static const int8_t weights[5] = {-4, -1, 0, 1, 4}; /* 从左到右的位置权重 */

  if (error_x10 == 0)
  {
    return 0U; /* 输出指针无效，计算失败 */
  }

  mask = AppLine_NormalizeMask(raw_mask); /* 先转换为统一极性位图 */

  for (i = 0; i < 5; i++)
  {
    if ((mask & (uint8_t)(1U << i)) != 0U)
    {
      weighted_sum = (int16_t)(weighted_sum + weights[i]); /* 累加该路的空间权重 */
      hit_count++;                                          /* 记录命中数量 */
    }
  }

  if (hit_count == 0)
  {
    return 0U; /* 一条线都没看到，无法给出有效误差 */
  }

  *error_x10 = (int16_t)((weighted_sum * 10) / hit_count); /* 平均权重并放大10倍，减少浮点运算 */
  return 1U;                                               /* 计算成功 */
}
