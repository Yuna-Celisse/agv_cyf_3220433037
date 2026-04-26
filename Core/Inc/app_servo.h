#ifndef APP_SERVO_H
#define APP_SERVO_H

#include <stdint.h>

void AppServo_Init(uint16_t default_angle_deg); /* 舵机 PWM 初始化并设置默认角度 */
void AppServo_SetAngle(uint16_t angle_deg);     /* 设置舵机角度（度） */
void AppServo_Task(void);                        /* 舵机周期任务（预留） */

#endif /* APP_SERVO_H */
