#include "app_oled_ui.h"

#include "oled.h"
#include "usart.h"

/*
 * 这是 OLED 显示的业务封装层。
 * 它不关心底层像素怎么画，只负责把“刷卡提示、目标点、距离、动作状态”
 * 这些应用层信息整理成文本并显示出来。
 */

#define APP_OLED_TEXT_X_OFFSET 16U
#define APP_OLED_LINE_LEN 16U
#define APP_OLED_LINE_BUF_LEN (APP_OLED_LINE_LEN + 1U)

static uint8_t oled_line[APP_OLED_LINE_BUF_LEN] = {0};
static uint8_t oled_last_line[APP_OLED_LINE_BUF_LEN] = {0};
static uint8_t oled_rfid_line[APP_OLED_LINE_BUF_LEN] = {0};
static uint8_t oled_last_rfid_line[APP_OLED_LINE_BUF_LEN] = {0};
static uint8_t oled_action_line[APP_OLED_LINE_BUF_LEN] = {0};
static uint8_t oled_last_action_line[APP_OLED_LINE_BUF_LEN] = {0};

/* 把 0~15 的数值转换成大写十六进制字符。 */
static char AppOled_HexDigit(uint8_t value)
{
  if (value < 10U)
  {
    return (char)('0' + value);
  }

  return (char)('A' + (value - 10U));
}

/* 某些显示信息会顺便通过串口输出，方便没有屏幕时调试。 */
static void AppOled_SendAsync(const uint8_t *data, uint16_t len)
{
  if ((data == 0) || (len == 0U))
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 20U);
}

/* 把任意长度文本整理到固定行缓冲区，并保证字符串正常结束。 */
static void AppOled_LoadLine(uint8_t *dst, const uint8_t *line, uint8_t len)
{
  uint8_t i;

  if ((dst == 0) || (line == 0))
  {
    return;
  }

  if (len > APP_OLED_LINE_LEN)
  {
    len = APP_OLED_LINE_LEN;
  }

  for (i = 0U; i < APP_OLED_LINE_BUF_LEN; i++)
  {
    dst[i] = 0U;
  }

  for (i = 0U; i < len; i++)
  {
    dst[i] = line[i];
  }
  dst[len] = '\0';
}

static void AppOled_RefreshMainLine(void)
{
  OLED_ClearPage(0U);
  OLED_ClearPage(1U);
  OLED_ShowString(APP_OLED_TEXT_X_OFFSET, 0U, oled_line, APP_OLED_LINE_LEN);
  OLED_RefreshPage(0U);
  OLED_RefreshPage(1U);
}

static void AppOled_RefreshRfidLine(void)
{
  OLED_ClearPage(2U);
  OLED_ClearPage(3U);
  OLED_ShowString(APP_OLED_TEXT_X_OFFSET, 16U, oled_rfid_line, APP_OLED_LINE_LEN);
  OLED_RefreshPage(2U);
  OLED_RefreshPage(3U);
}

static void AppOled_RefreshActionLine(void)
{
  OLED_ClearPage(4U);
  OLED_ClearPage(5U);
  OLED_ShowString(APP_OLED_TEXT_X_OFFSET, 32U, oled_action_line, APP_OLED_LINE_LEN);
  OLED_RefreshPage(4U);
  OLED_RefreshPage(5U);
}

/* 只有当某一行内容真的变化时才刷新 OLED，减少闪烁和重复写屏。 */
static void AppOled_UpdateCachedLine(uint8_t *current, uint8_t *last, void (*refresh_fn)(void))
{
  uint8_t i;

  for (i = 0U; i < APP_OLED_LINE_LEN; i++)
  {
    if (current[i] != last[i])
    {
      for (i = 0U; i < APP_OLED_LINE_BUF_LEN; i++)
      {
        last[i] = current[i];
      }
      refresh_fn();
      return;
    }
  }
}

/* 显示刷到的 UID，并可选地通过串口同步输出。 */
void AppOled_ShowUid(const uint8_t uid[4], uint8_t uart_report_enable)
{
  uint8_t i;
  uint8_t display_buf[] = "IC:00000000";

  if (uid == 0)
  {
    return;
  }

  for (i = 0U; i < 4U; i++)
  {
    display_buf[3U + (2U * i)] = (uint8_t)AppOled_HexDigit((uint8_t)((uid[i] >> 4U) & 0x0FU));
    display_buf[4U + (2U * i)] = (uint8_t)AppOled_HexDigit((uint8_t)(uid[i] & 0x0FU));
  }

  AppOled_LoadLine(oled_rfid_line, display_buf, 11U);
  AppOled_UpdateCachedLine(oled_rfid_line, oled_last_rfid_line, AppOled_RefreshRfidLine);

  if (uart_report_enable != 0U)
  {
    uint8_t tx_buf[] = "RFID:00000000\r\n";

    for (i = 0U; i < 4U; i++)
    {
      tx_buf[5U + (2U * i)] = (uint8_t)AppOled_HexDigit((uint8_t)((uid[i] >> 4U) & 0x0FU));
      tx_buf[6U + (2U * i)] = (uint8_t)AppOled_HexDigit((uint8_t)(uid[i] & 0x0FU));
    }

    AppOled_SendAsync(tx_buf, (uint16_t)(sizeof(tx_buf) - 1U));
  }
}

/* 待机界面：提示用户刷卡，并清空动作行。 */
void AppOled_ShowSwipePrompt(void)
{
  AppOled_LoadLine(oled_line, (const uint8_t *)"SWIPE CARD", 10U);
  AppOled_LoadLine(oled_rfid_line, (const uint8_t *)"TARGET: -", 9U);
  AppOled_LoadLine(oled_action_line, (const uint8_t *)"", 0U);
  AppOled_UpdateCachedLine(oled_line, oled_last_line, AppOled_RefreshMainLine);
  AppOled_UpdateCachedLine(oled_rfid_line, oled_last_rfid_line, AppOled_RefreshRfidLine);
  AppOled_UpdateCachedLine(oled_action_line, oled_last_action_line, AppOled_RefreshActionLine);
}

/* 根据当前目标卡片更新“目标站点”这一行文本。 */
void AppOled_ShowTarget(uint8_t card_id)
{
  if (card_id == APP_CARD_ID_A)
  {
    AppOled_LoadLine(oled_rfid_line, (const uint8_t *)"TARGET: A", 9U);
  }
  else if (card_id == APP_CARD_ID_B)
  {
    AppOled_LoadLine(oled_rfid_line, (const uint8_t *)"TARGET: B", 9U);
  }
  else
  {
    AppOled_LoadLine(oled_rfid_line, (const uint8_t *)"TARGET: -", 9U);
  }

  AppOled_UpdateCachedLine(oled_rfid_line, oled_last_rfid_line, AppOled_RefreshRfidLine);
}

/* 把超声波距离格式化成固定宽度文本，便于 OLED 稳定显示。 */
void AppOled_ShowDistance(uint8_t has_distance, uint16_t distance_cm)
{
  uint8_t line_buf[] = "US:---cm";

  if (has_distance != 0U)
  {
    if (distance_cm > 999U)
    {
      distance_cm = 999U;
    }

    line_buf[3] = (uint8_t)('0' + ((distance_cm / 100U) % 10U));
    line_buf[4] = (uint8_t)('0' + ((distance_cm / 10U) % 10U));
    line_buf[5] = (uint8_t)('0' + (distance_cm % 10U));
  }

  AppOled_LoadLine(oled_line, line_buf, 8U);
  AppOled_UpdateCachedLine(oled_line, oled_last_line, AppOled_RefreshMainLine);
}

void AppOled_ShowStatus(const uint8_t *status, uint8_t len)
{
  AppOled_LoadLine(oled_rfid_line, status, len);
  AppOled_UpdateCachedLine(oled_rfid_line, oled_last_rfid_line, AppOled_RefreshRfidLine);
}

void AppOled_ShowAction(const uint8_t *action, uint8_t len)
{
  AppOled_LoadLine(oled_action_line, action, len);
  AppOled_UpdateCachedLine(oled_action_line, oled_last_action_line, AppOled_RefreshActionLine);
}

void AppOled_ClearAction(void)
{
  AppOled_LoadLine(oled_action_line, (const uint8_t *)"", 0U);
  AppOled_UpdateCachedLine(oled_action_line, oled_last_action_line, AppOled_RefreshActionLine);
}
