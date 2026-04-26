#ifndef APP_ULTRASONIC_H
#define APP_ULTRASONIC_H

#include <stdint.h>

void AppUltrasonic_Init(uint8_t uart_report_enable); /* 超声波模块初始化 */
void AppUltrasonic_StartMeasure(void);               /* 触发一次测距 */
void AppUltrasonic_Task(uint32_t now_ms);            /* 超声波周期任务：处理超时 */
void AppUltrasonic_HandleEchoExti(uint16_t gpio_pin);/* EXTI 回调中处理 ECHO 脉冲 */
uint8_t AppUltrasonic_IsBusy(void);                  /* 查询当前是否正在测距 */
uint8_t AppUltrasonic_FetchResult(uint16_t *distance_cm, uint8_t *has_distance); /* 读取一次测距结果 */

#endif /* APP_ULTRASONIC_H */
