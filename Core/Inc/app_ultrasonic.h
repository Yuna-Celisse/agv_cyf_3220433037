#ifndef APP_ULTRASONIC_H
#define APP_ULTRASONIC_H

#include <stdint.h>

void AppUltrasonic_Init(uint8_t uart_report_enable);
void AppUltrasonic_StartMeasure(void);
void AppUltrasonic_Task(uint32_t now_ms);
void AppUltrasonic_HandleEchoExti(uint16_t gpio_pin);
uint8_t AppUltrasonic_IsBusy(void);
uint8_t AppUltrasonic_FetchResult(uint16_t *distance_cm, uint8_t *has_distance);

#endif /* APP_ULTRASONIC_H */
