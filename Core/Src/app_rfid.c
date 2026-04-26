#include "app_rfid.h"

#include "rc522.h"

/*
 * RFID 模块对 RC522 做了一层业务封装：
 * - 轮询读 UID；
 * - 识别是否是已登记卡片；
 * - 避免同一张卡持续贴近时被重复触发。
 */

#define APP_RFID_POLL_INTERVAL_MS 100U /* RFID 轮询最小间隔（ms） */

static uint32_t last_rfid_poll_tick = 0U;            /* 上次轮询时间戳 */
static uint8_t last_uid[APP_RFID_UID_LEN] = {0};     /* 上一次识别到的 UID */
static uint8_t has_last_uid = 0U;                    /* 是否已有上一次 UID 缓存 */

static const uint8_t uid_card_a[APP_RFID_UID_LEN] = {0x16U, 0x15U, 0x12U, 0x07U}; /* A 卡 UID */
static const uint8_t uid_card_b[APP_RFID_UID_LEN] = {0x1EU, 0xF1U, 0x2BU, 0x07U}; /* B 卡 UID */

/* 比较两个 4 字节 UID 是否完全一致。 */
static uint8_t AppRfid_UidEquals(const uint8_t left[APP_RFID_UID_LEN], const uint8_t right[APP_RFID_UID_LEN])
{
  uint8_t i;

  for (i = 0U; i < APP_RFID_UID_LEN; i++)
  {
    if (left[i] != right[i])
    {
      return 0U; /* 任一字节不同即判定不相等 */
    }
  }

  return 1U; /* 全部字节相同 */
}

/* 逐字节复制 UID，避免额外依赖标准库内存函数。 */
static void AppRfid_UidCopy(uint8_t dst[APP_RFID_UID_LEN], const uint8_t src[APP_RFID_UID_LEN])
{
  uint8_t i;

  for (i = 0U; i < APP_RFID_UID_LEN; i++)
  {
    dst[i] = src[i]; /* 逐字节复制 UID */
  }
}

/* 把底层 UID 映射成业务层的卡片编号，例如 A 卡、B 卡。 */
static uint8_t AppRfid_ResolveCardId(const uint8_t uid[APP_RFID_UID_LEN])
{
  if (uid == 0)
  {
    return APP_RFID_CARD_ID_NONE; /* 空指针保护 */
  }

  if (AppRfid_UidEquals(uid, uid_card_a) != 0U)
  {
    return APP_RFID_CARD_ID_A; /* 命中 A 卡 */
  }

  if (AppRfid_UidEquals(uid, uid_card_b) != 0U)
  {
    return APP_RFID_CARD_ID_B; /* 命中 B 卡 */
  }

  return APP_RFID_CARD_ID_NONE; /* 未登记卡片 */
}

/* 初始化 RC522，并清空上一次卡号缓存。 */
void AppRfid_Init(void)
{
  RC522_Init();               /* 初始化 RC522 硬件与寄存器 */
  has_last_uid = 0U;          /* 清空去重缓存 */
  last_rfid_poll_tick = 0U;   /* 重置轮询时间 */
}

/*
 * 轮询读卡。
 * 只有在允许轮询、到达轮询周期且成功读到卡时，才会向上层返回事件；
 * 同时利用 last_uid 去重，避免同一张卡被反复当成新卡。
 */
uint8_t AppRfid_Poll(uint32_t now_ms, uint8_t allow_poll, AppRfidEvent_t *event)
{
  uint8_t uid[APP_RFID_UID_LEN];

  if ((event == 0) || (allow_poll == 0U))
  {
    return 0U; /* 参数无效或当前不允许轮询 */
  }

  if ((now_ms - last_rfid_poll_tick) < APP_RFID_POLL_INTERVAL_MS)
  {
    return 0U; /* 未到轮询周期 */
  }

  last_rfid_poll_tick = now_ms; /* 更新时间戳 */

  if (RC522_ReadUid(uid) == 0U)
  {
    has_last_uid = 0U; /* 读卡失败时清空去重状态，避免下一次被误判旧卡 */
    return 0U;
  }

  AppRfid_UidCopy(event->uid, uid);          /* 回传本次 UID */
  event->card_id = AppRfid_ResolveCardId(uid); /* 解析业务卡 ID */
  event->is_new_uid = 0U;                    /* 默认先标记为非新卡 */

  if ((has_last_uid == 0U) || (AppRfid_UidEquals(last_uid, uid) == 0U))
  {
    AppRfid_UidCopy(last_uid, uid); /* 更新去重缓存 */
    has_last_uid = 1U;
    event->is_new_uid = 1U;         /* 与上次不同，标记新卡事件 */
  }

  return 1U; /* 本次轮询获得了 UID 事件 */
}
