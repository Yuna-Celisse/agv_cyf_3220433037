#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include <stdint.h>

void AppMotor_SetEnable(uint8_t enable);
void AppMotor_SetForwardDirection(void);
void AppMotor_SetDuty(uint16_t duty_left_pm, uint16_t duty_right_pm);
void AppMotor_SetClosedLoop(uint8_t enable);
void AppMotor_SetTargetSpeed(int16_t left_pulse_per_sec, int16_t right_pulse_per_sec);
void AppMotor_SetTargetFromDuty(uint16_t duty_left_pm, uint16_t duty_right_pm);
void AppMotor_Task(uint32_t now_ms);

#endif /* APP_MOTOR_H */
