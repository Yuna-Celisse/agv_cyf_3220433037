#ifndef APP_LINE_H
#define APP_LINE_H

#include "main.h"
#include <stdint.h>

uint8_t AppLine_ReadMask(void);                                  /* 读取5路巡线红外原始位图 */
uint8_t AppLine_NormalizeMask(uint8_t raw_mask);                /* 统一位图极性（1=检测到线） */
uint8_t AppLine_ComputeErrorX10(uint8_t raw_mask, int16_t *error_x10); /* 计算巡线偏差并放大10倍 */

#endif /* APP_LINE_H */
