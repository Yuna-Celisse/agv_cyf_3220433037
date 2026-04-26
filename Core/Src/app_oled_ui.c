#include "app_oled_ui.h"

#include "oled.h"
#include "usart.h"

#define APP_OLED_TEXT_X_OFFSET 16U              /* 文本 X 偏移，留出左侧边距 */
#define APP_OLED_LINE_LEN 16U                   /* 单行字符长度 */
#define APP_OLED_LINE_BUF_LEN (APP_OLED_LINE_LEN + 1U) /* 额外 +1 用于字符串结束符 */

static uint8_t oled_line[APP_OLED_LINE_BUF_LEN] = {0};           /* 主显示行当前内容 */
static uint8_t oled_last_line[APP_OLED_LINE_BUF_LEN] = {0};      /* 主显示行上次内容 */
static uint8_t oled_rfid_line[APP_OLED_LINE_BUF_LEN] = {0};      /* RFID/目标行当前内容 */
static uint8_t oled_last_rfid_line[APP_OLED_LINE_BUF_LEN] = {0}; /* RFID/目标行上次内容 */

static char AppOled_HexDigit(uint8_t value)
{
  if (value < 10U)
  {
    return (char)('0' + value); /* 0~9 转 ASCII 数字 */
  }

  return (char)('A' + (value - 10U)); /* 10~15 转 ASCII A~F */
}

static void AppOled_SendAsync(const uint8_t *data, uint16_t len)
{
  if ((data == 0) || (len == 0U))
  {
    return; /* 空指针或空长度保护 */
  }

  (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 20U); /* 串口同步发送 */
}

static void AppOled_LoadLine(uint8_t *dst, const uint8_t *line, uint8_t len)
{
  uint8_t i;

  if ((dst == 0) || (line == 0))
  {
    return; /* 参数合法性保护 */
  }

  if (len > APP_OLED_LINE_LEN)
  {
    len = APP_OLED_LINE_LEN; /* 防止越界写入 */
  }

  for (i = 0U; i < APP_OLED_LINE_BUF_LEN; i++)
  {
    dst[i] = 0U; /* 先清空目标行缓冲 */
  }

  for (i = 0U; i < len; i++)
  {
    dst[i] = line[i]; /* 拷贝有效字符 */
  }
  dst[len] = '\0'; /* 结尾补字符串结束符 */
}

static void AppOled_RefreshMainLine(void)
{
  OLED_ClearPage(0U);                                                   /* 清主行上半页 */
  OLED_ClearPage(1U);                                                   /* 清主行下半页 */
  OLED_ShowString(APP_OLED_TEXT_X_OFFSET, 0U, oled_line, APP_OLED_LINE_LEN); /* 重绘主行文本 */
  OLED_RefreshPage(0U);                                                 /* 刷新页0 */
  OLED_RefreshPage(1U);                                                 /* 刷新页1 */
}

static void AppOled_RefreshRfidLine(void)
{
  OLED_ClearPage(2U);                                                     /* 清 RFID 行上半页 */
  OLED_ClearPage(3U);                                                     /* 清 RFID 行下半页 */
  OLED_ShowString(APP_OLED_TEXT_X_OFFSET, 16U, oled_rfid_line, APP_OLED_LINE_LEN); /* 重绘 RFID 行 */
  OLED_RefreshPage(2U);                                                   /* 刷新页2 */
  OLED_RefreshPage(3U);                                                   /* 刷新页3 */
}

static void AppOled_UpdateMainLine(void)
{
  uint8_t i;

  for (i = 0U; i < APP_OLED_LINE_LEN; i++)
  {
    if (oled_line[i] != oled_last_line[i])
    {
      for (i = 0U; i < APP_OLED_LINE_BUF_LEN; i++)
      {
        oled_last_line[i] = oled_line[i]; /* 记录新的缓存快照 */
      }
      AppOled_RefreshMainLine(); /* 仅在内容变化时刷新屏幕，减少闪烁 */
      return;
    }
  }
}

static void AppOled_UpdateRfidLine(void)
{
  uint8_t i;

  for (i = 0U; i < APP_OLED_LINE_LEN; i++)
  {
    if (oled_rfid_line[i] != oled_last_rfid_line[i])
    {
      for (i = 0U; i < APP_OLED_LINE_BUF_LEN; i++)
      {
        oled_last_rfid_line[i] = oled_rfid_line[i]; /* 记录新的缓存快照 */
      }
      AppOled_RefreshRfidLine(); /* 仅在内容变化时刷新屏幕，减少闪烁 */
      return;
    }
  }
}

void AppOled_ShowUid(const uint8_t uid[4], uint8_t uart_report_enable)
{
  uint8_t i;
  uint8_t display_buf[] = "IC:00000000";

  if (uid == 0)
  {
    return; /* 空指针保护 */
  }

  for (i = 0U; i < 4U; i++)
  {
    display_buf[3U + (2U * i)] = (uint8_t)AppOled_HexDigit((uint8_t)((uid[i] >> 4U) & 0x0FU)); /* 高4位 */
    display_buf[4U + (2U * i)] = (uint8_t)AppOled_HexDigit((uint8_t)(uid[i] & 0x0FU));         /* 低4位 */
  }

  AppOled_LoadLine(oled_rfid_line, display_buf, 11U); /* OLED 显示 IC UID */
  AppOled_UpdateRfidLine();

  if (uart_report_enable != 0U)
  {
    uint8_t tx_buf[] = "RFID:00000000\r\n";

    for (i = 0U; i < 4U; i++)
    {
      tx_buf[5U + (2U * i)] = (uint8_t)AppOled_HexDigit((uint8_t)((uid[i] >> 4U) & 0x0FU)); /* 高4位 */
      tx_buf[6U + (2U * i)] = (uint8_t)AppOled_HexDigit((uint8_t)(uid[i] & 0x0FU));         /* 低4位 */
    }

    AppOled_SendAsync(tx_buf, (uint16_t)(sizeof(tx_buf) - 1U)); /* 串口同步上报 UID */
  }
}

void AppOled_ShowSwipePrompt(void)
{
  AppOled_LoadLine(oled_line, (const uint8_t *)"SWIPE CARD", 10U);      /* 主行提示刷卡 */
  AppOled_LoadLine(oled_rfid_line, (const uint8_t *)"TARGET: -", 9U);   /* 目标行显示未选择 */
  AppOled_UpdateMainLine();
  AppOled_UpdateRfidLine();
}

void AppOled_ShowTarget(uint8_t card_id)
{
  if (card_id == APP_CARD_ID_A)
  {
    AppOled_LoadLine(oled_rfid_line, (const uint8_t *)"TARGET: A", 9U); /* A 任务目标 */
  }
  else if (card_id == APP_CARD_ID_B)
  {
    AppOled_LoadLine(oled_rfid_line, (const uint8_t *)"TARGET: B", 9U); /* B 任务目标 */
  }
  else
  {
    AppOled_LoadLine(oled_rfid_line, (const uint8_t *)"TARGET: -", 9U); /* 无任务目标 */
  }

  AppOled_UpdateRfidLine();
}

void AppOled_ShowDistance(uint8_t has_distance, uint16_t distance_cm)
{
  uint8_t line_buf[] = "US:---cm";

  if (has_distance != 0U)
  {
    if (distance_cm > 999U)
    {
      distance_cm = 999U; /* 三位数显示上限 */
    }

    line_buf[3] = (uint8_t)('0' + ((distance_cm / 100U) % 10U)); /* 百位 */
    line_buf[4] = (uint8_t)('0' + ((distance_cm / 10U) % 10U));  /* 十位 */
    line_buf[5] = (uint8_t)('0' + (distance_cm % 10U));           /* 个位 */
  }

  AppOled_LoadLine(oled_line, line_buf, 8U); /* 主行显示超声波距离 */
  AppOled_UpdateMainLine();
}
