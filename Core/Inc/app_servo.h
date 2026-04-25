#ifndef APP_SERVO_H
#define APP_SERVO_H

#include <stdint.h>

void AppServo_Init(uint16_t default_angle_deg);
void AppServo_SetAngle(uint16_t angle_deg);
void AppServo_Task(void);

#endif /* APP_SERVO_H */
