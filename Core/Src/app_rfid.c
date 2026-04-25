#include "app_rfid.h"

#include "rc522.h"

#define APP_RFID_POLL_INTERVAL_MS 100U

static uint32_t last_rfid_poll_tick = 0U;
static uint8_t last_uid[APP_RFID_UID_LEN] = {0};
static uint8_t has_last_uid = 0U;

static const uint8_t uid_card_a[APP_RFID_UID_LEN] = {0x16U, 0x15U, 0x12U, 0x07U};
static const uint8_t uid_card_b[APP_RFID_UID_LEN] = {0x1EU, 0xF1U, 0x2BU, 0x07U};

static uint8_t AppRfid_UidEquals(const uint8_t left[APP_RFID_UID_LEN], const uint8_t right[APP_RFID_UID_LEN])
{
  uint8_t i;

  for (i = 0U; i < APP_RFID_UID_LEN; i++)
  {
    if (left[i] != right[i])
    {
      return 0U;
    }
  }

  return 1U;
}

static void AppRfid_UidCopy(uint8_t dst[APP_RFID_UID_LEN], const uint8_t src[APP_RFID_UID_LEN])
{
  uint8_t i;

  for (i = 0U; i < APP_RFID_UID_LEN; i++)
  {
    dst[i] = src[i];
  }
}

static uint8_t AppRfid_ResolveCardId(const uint8_t uid[APP_RFID_UID_LEN])
{
  if (uid == 0)
  {
    return APP_RFID_CARD_ID_NONE;
  }

  if (AppRfid_UidEquals(uid, uid_card_a) != 0U)
  {
    return APP_RFID_CARD_ID_A;
  }

  if (AppRfid_UidEquals(uid, uid_card_b) != 0U)
  {
    return APP_RFID_CARD_ID_B;
  }

  return APP_RFID_CARD_ID_NONE;
}

void AppRfid_Init(void)
{
  RC522_Init();
  has_last_uid = 0U;
  last_rfid_poll_tick = 0U;
}

uint8_t AppRfid_Poll(uint32_t now_ms, uint8_t allow_poll, AppRfidEvent_t *event)
{
  uint8_t uid[APP_RFID_UID_LEN];

  if ((event == 0) || (allow_poll == 0U))
  {
    return 0U;
  }

  if ((now_ms - last_rfid_poll_tick) < APP_RFID_POLL_INTERVAL_MS)
  {
    return 0U;
  }

  last_rfid_poll_tick = now_ms;

  if (RC522_ReadUid(uid) == 0U)
  {
    has_last_uid = 0U;
    return 0U;
  }

  AppRfid_UidCopy(event->uid, uid);
  event->card_id = AppRfid_ResolveCardId(uid);
  event->is_new_uid = 0U;

  if ((has_last_uid == 0U) || (AppRfid_UidEquals(last_uid, uid) == 0U))
  {
    AppRfid_UidCopy(last_uid, uid);
    has_last_uid = 1U;
    event->is_new_uid = 1U;
  }

  return 1U;
}
