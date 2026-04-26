#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include <stdint.h>

void AppMotor_SetEnable(uint8_t enable); /* 电机驱动使能控制：0关断，非0使能 */
void AppMotor_SetForwardDirection(void); /* 设置左右电机前进方向引脚 */
void AppMotor_SetDuty(uint16_t duty_left_pm, uint16_t duty_right_pm); /* 直接设置左右PWM占空比（千分比） */
void AppMotor_SetClosedLoop(uint8_t enable); /* 开/关速度闭环控制 */
void AppMotor_SetTargetSpeed(int16_t left_pulse_per_sec, int16_t right_pulse_per_sec); /* 设置闭环目标速度 */
void AppMotor_SetTargetFromDuty(uint16_t duty_left_pm, uint16_t duty_right_pm); /* 按占空比映射并设置目标 */
void AppMotor_Task(uint32_t now_ms); /* 电机周期任务：闭环计算与输出更新 */

#endif /* APP_MOTOR_H */
