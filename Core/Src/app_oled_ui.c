#include "app_oled_ui.h"

#include "oled.h"
#include "usart.h"

#define APP_OLED_TEXT_X_OFFSET 16U
#define APP_OLED_LINE_LEN 16U
#define APP_OLED_LINE_BUF_LEN (APP_OLED_LINE_LEN + 1U)

static uint8_t oled_line[APP_OLED_LINE_BUF_LEN] = {0};
static uint8_t oled_last_line[APP_OLED_LINE_BUF_LEN] = {0};
static uint8_t oled_rfid_line[APP_OLED_LINE_BUF_LEN] = {0};
static uint8_t oled_last_rfid_line[APP_OLED_LINE_BUF_LEN] = {0};

static char AppOled_HexDigit(uint8_t value)
{
  if (value < 10U)
  {
    return (char)('0' + value);
  }

  return (char)('A' + (value - 10U));
}

static void AppOled_SendAsync(const uint8_t *data, uint16_t len)
{
  if ((data == 0) || (len == 0U))
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 20U);
}

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

static void AppOled_UpdateMainLine(void)
{
  uint8_t i;

  for (i = 0U; i < APP_OLED_LINE_LEN; i++)
  {
    if (oled_line[i] != oled_last_line[i])
    {
      for (i = 0U; i < APP_OLED_LINE_BUF_LEN; i++)
      {
        oled_last_line[i] = oled_line[i];
      }
      AppOled_RefreshMainLine();
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
        oled_last_rfid_line[i] = oled_rfid_line[i];
      }
      AppOled_RefreshRfidLine();
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
    return;
  }

  for (i = 0U; i < 4U; i++)
  {
    display_buf[3U + (2U * i)] = (uint8_t)AppOled_HexDigit((uint8_t)((uid[i] >> 4U) & 0x0FU));
    display_buf[4U + (2U * i)] = (uint8_t)AppOled_HexDigit((uint8_t)(uid[i] & 0x0FU));
  }

  AppOled_LoadLine(oled_rfid_line, display_buf, 11U);
  AppOled_UpdateRfidLine();

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

void AppOled_ShowSwipePrompt(void)
{
  AppOled_LoadLine(oled_line, (const uint8_t *)"SWIPE CARD", 10U);
  AppOled_LoadLine(oled_rfid_line, (const uint8_t *)"TARGET: -", 9U);
  AppOled_UpdateMainLine();
  AppOled_UpdateRfidLine();
}

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

  AppOled_UpdateRfidLine();
}

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
  AppOled_UpdateMainLine();
}
