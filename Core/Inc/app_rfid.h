#ifndef APP_RFID_H
#define APP_RFID_H

#include <stdint.h>

#define APP_RFID_UID_LEN 4U
#define APP_RFID_CARD_ID_NONE 0U
#define APP_RFID_CARD_ID_A 1U
#define APP_RFID_CARD_ID_B 2U

typedef struct
{
  uint8_t uid[APP_RFID_UID_LEN];
  uint8_t card_id;
  uint8_t is_new_uid;
} AppRfidEvent_t;

void AppRfid_Init(void);
uint8_t AppRfid_Poll(uint32_t now_ms, uint8_t allow_poll, AppRfidEvent_t *event);

#endif /* APP_RFID_H */
