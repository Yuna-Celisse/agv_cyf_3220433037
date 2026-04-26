#ifndef APP_OLED_UI_H
#define APP_OLED_UI_H

#include <stdint.h>

#define APP_CARD_ID_NONE 0U /* 无目标卡 */
#define APP_CARD_ID_A 1U    /* A 目标卡 */
#define APP_CARD_ID_B 2U    /* B 目标卡 */

void AppOled_ShowUid(const uint8_t uid[4], uint8_t uart_report_enable); /* 显示 UID，并可选串口上报 */
void AppOled_ShowSwipePrompt(void);                                      /* 显示“请刷卡”提示 */
void AppOled_ShowTarget(uint8_t card_id);                                /* 显示当前目标卡 */
void AppOled_ShowDistance(uint8_t has_distance, uint16_t distance_cm);   /* 显示超声波距离 */
void AppOled_ShowStatus(const uint8_t *status, uint8_t len);             /* 显示运行/避障状态 */

#endif /* APP_OLED_UI_H */
