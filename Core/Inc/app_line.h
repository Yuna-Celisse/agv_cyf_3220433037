#ifndef APP_LINE_H
#define APP_LINE_H

#include "main.h"
#include <stdint.h>

uint8_t AppLine_ReadMask(void);
uint8_t AppLine_NormalizeMask(uint8_t raw_mask);
uint8_t AppLine_ComputeErrorX10(uint8_t raw_mask, int16_t *error_x10);

#endif /* APP_LINE_H */
