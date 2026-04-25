#ifndef APP_OLED_UI_H
#define APP_OLED_UI_H

#include <stdint.h>

#define APP_CARD_ID_NONE 0U
#define APP_CARD_ID_A 1U
#define APP_CARD_ID_B 2U

void AppOled_ShowUid(const uint8_t uid[4], uint8_t uart_report_enable);
void AppOled_ShowSwipePrompt(void);
void AppOled_ShowTarget(uint8_t card_id);
void AppOled_ShowDistance(uint8_t has_distance, uint16_t distance_cm);

#endif /* APP_OLED_UI_H */
