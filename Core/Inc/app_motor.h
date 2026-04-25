#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include <stdint.h>

void AppMotor_SetEnable(uint8_t enable);
void AppMotor_SetForwardDirection(void);
void AppMotor_SetDuty(uint16_t duty_left_pm, uint16_t duty_right_pm);

#endif /* APP_MOTOR_H */
