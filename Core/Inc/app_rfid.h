#ifndef APP_RFID_H
#define APP_RFID_H

#include <stdint.h>

#define APP_RFID_UID_LEN 4U        /* UID 字节长度 */
#define APP_RFID_CARD_ID_NONE 0U   /* 非业务卡或未知卡 */
#define APP_RFID_CARD_ID_A 1U      /* A 任务卡 */
#define APP_RFID_CARD_ID_B 2U      /* B 任务卡 */

typedef struct
{
  uint8_t uid[APP_RFID_UID_LEN]; /* 本次读到的 UID */
  uint8_t card_id;               /* 解析出的业务卡 ID */
  uint8_t is_new_uid;            /* 是否为相对上次的新卡事件 */
} AppRfidEvent_t;

void AppRfid_Init(void); /* RFID 模块初始化 */
uint8_t AppRfid_Poll(uint32_t now_ms, uint8_t allow_poll, AppRfidEvent_t *event); /* 周期轮询并输出读卡事件 */

#endif /* APP_RFID_H */
