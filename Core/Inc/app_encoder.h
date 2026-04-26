#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct
{
  int32_t left_count;
  int32_t right_count;
} AppEncoder_Counts_t;

void AppEncoder_Init(void);
void AppEncoder_HandleExti(uint16_t gpio_pin);
void AppEncoder_Reset(void);
AppEncoder_Counts_t AppEncoder_GetCounts(void);

#endif /* APP_ENCODER_H */
