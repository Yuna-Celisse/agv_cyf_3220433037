#include "rc522.h"

#include "main.h"

/*
 * RC522 底层驱动采用 GPIO 模拟 SPI 的方式访问寄存器。
 * 这一层不直接处理整车任务，只负责：
 * - 读写 RC522 寄存器
 * - 发送寻卡、防冲撞、停止等基础命令
 * - 返回 UID 等底层数据
 */

#define RC522_STATUS_OK      0U
#define RC522_STATUS_NOTAG   1U
#define RC522_STATUS_ERR     2U

#define RC522_CMD_IDLE        0x00U
#define RC522_CMD_AUTHENT     0x0EU
#define RC522_CMD_TRANSCEIVE  0x0CU
#define RC522_CMD_RESETPHASE  0x0FU
#define RC522_CMD_CALCCRC     0x03U

#define RC522_PICC_REQIDL     0x26U
#define RC522_PICC_ANTICOLL   0x93U
#define RC522_PICC_HALT        0x50U

#define RC522_REG_COMMAND       0x01U
#define RC522_REG_COMM_IE_N     0x02U
#define RC522_REG_COMM_IRQ      0x04U
#define RC522_REG_DIV_IRQ       0x05U
#define RC522_REG_ERROR         0x06U
#define RC522_REG_STATUS2       0x08U
#define RC522_REG_FIFO_DATA     0x09U
#define RC522_REG_FIFO_LEVEL    0x0AU
#define RC522_REG_CONTROL       0x0CU
#define RC522_REG_BIT_FRAMING   0x0DU
#define RC522_REG_MODE          0x11U
#define RC522_REG_TX_CONTROL    0x14U
#define RC522_REG_TX_AUTO       0x15U
#define RC522_REG_RX_THRESHOLD  0x18U
#define RC522_REG_RF_CFG        0x26U
#define RC522_REG_T_MODE        0x2AU
#define RC522_REG_T_PRESCALER   0x2BU
#define RC522_REG_T_RELOAD_H    0x2CU
#define RC522_REG_T_RELOAD_L    0x2DU
#define RC522_REG_CRC_RESULT_M  0x21U
#define RC522_REG_CRC_RESULT_L  0x22U

#define RC522_MAX_LEN           16U

/* 软件 SPI 每个时钟边沿之间留出少量延时，保证 RC522 能稳定采样。 */
static void RC522_SpiDelay(void)
{
  __NOP();
  __NOP();
  __NOP();
  __NOP();
}

/* 片选拉低，开始一次 RC522 通信。 */
static void RC522_CsLow(void)
{
  HAL_GPIO_WritePin(RC522_SDA_GPIO_Port, RC522_SDA_Pin, GPIO_PIN_RESET);
}

/* 片选拉高，结束一次 RC522 通信。 */
static void RC522_CsHigh(void)
{
  HAL_GPIO_WritePin(RC522_SDA_GPIO_Port, RC522_SDA_Pin, GPIO_PIN_SET);
}

/* 软件 SPI 时钟线拉低。 */
static void RC522_SckLow(void)
{
  HAL_GPIO_WritePin(RC522_SCK_GPIO_Port, RC522_SCK_Pin, GPIO_PIN_RESET);
}

/* 软件 SPI 时钟线拉高。 */
static void RC522_SckHigh(void)
{
  HAL_GPIO_WritePin(RC522_SCK_GPIO_Port, RC522_SCK_Pin, GPIO_PIN_SET);
}

/* 向 MOSI 引脚输出 0/1 电平。 */
static void RC522_MosiWrite(uint8_t value)
{
  HAL_GPIO_WritePin(RC522_MOSI_GPIO_Port, RC522_MOSI_Pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 从 MISO 引脚读取当前 0/1 电平。 */
static uint8_t RC522_MisoRead(void)
{
  return (uint8_t)(HAL_GPIO_ReadPin(RC522_MISO_GPIO_Port, RC522_MISO_Pin) == GPIO_PIN_SET ? 1U : 0U);
}

static uint8_t RC522_SpiTransferByte(uint8_t data)
{
  /* 以 MSB first 方式发送并接收 1 个字节。 */
  uint8_t bit;
  uint8_t rx = 0U;

  for (bit = 0U; bit < 8U; bit++)
  {
    RC522_MosiWrite((uint8_t)((data & 0x80U) != 0U));
    RC522_SpiDelay();

    RC522_SckHigh();
    rx <<= 1U;
    rx |= RC522_MisoRead();
    RC522_SpiDelay();

    RC522_SckLow();
    data <<= 1U;
  }

  return rx;
}

static void RC522_WriteRegister(uint8_t reg, uint8_t value)
{
  /* RC522 写寄存器：先发地址，再发数据。 */
  uint8_t addr = (uint8_t)((reg << 1U) & 0x7EU);

  RC522_CsLow();
  (void)RC522_SpiTransferByte(addr);
  (void)RC522_SpiTransferByte(value);
  RC522_CsHigh();
}

static uint8_t RC522_ReadRegister(uint8_t reg)
{
  /* RC522 读寄存器：地址最高位置 1，随后读回 1 字节。 */
  uint8_t addr = (uint8_t)(((reg << 1U) & 0x7EU) | 0x80U);
  uint8_t value;

  RC522_CsLow();
  (void)RC522_SpiTransferByte(addr);
  value = RC522_SpiTransferByte(0x00U);
  RC522_CsHigh();

  return value;
}

static void RC522_SetBitMask(uint8_t reg, uint8_t mask)
{
  /* 将寄存器中的某些位位置 1，其他位保持不变。 */
  RC522_WriteRegister(reg, (uint8_t)(RC522_ReadRegister(reg) | mask));
}

static void RC522_ClearBitMask(uint8_t reg, uint8_t mask)
{
  /* 将寄存器中的某些位清 0，其他位保持不变。 */
  RC522_WriteRegister(reg, (uint8_t)(RC522_ReadRegister(reg) & (uint8_t)(~mask)));
}

static void RC522_CalculateCrc(const uint8_t* inData, uint8_t len, uint8_t outData[2])
{
  /* 使用 RC522 芯片内部 CRC 单元计算给定数据的 CRC 结果。 */
  uint8_t i;
  uint8_t n;

  RC522_ClearBitMask(RC522_REG_DIV_IRQ, 0x04U);
  RC522_SetBitMask(RC522_REG_FIFO_LEVEL, 0x80U);

  for (i = 0U; i < len; i++)
  {
    RC522_WriteRegister(RC522_REG_FIFO_DATA, inData[i]);
  }

  RC522_WriteRegister(RC522_REG_COMMAND, RC522_CMD_CALCCRC);

  i = 0xFFU;
  do
  {
    n = RC522_ReadRegister(RC522_REG_DIV_IRQ);
    i--;
  } while ((i != 0U) && ((n & 0x04U) == 0U));

  outData[0] = RC522_ReadRegister(RC522_REG_CRC_RESULT_L);
  outData[1] = RC522_ReadRegister(RC522_REG_CRC_RESULT_M);
}

static uint8_t RC522_ToCard(uint8_t command,
                            const uint8_t* sendData,
                            uint8_t sendLen,
                            uint8_t* backData,
                            uint16_t* backLen)
{
  /*
   * 这是与卡片交互的核心收发函数。
   * 它会把待发送数据装入 FIFO，启动指定命令，等待完成标志，
   * 然后再把返回数据从 FIFO 中读出。
   */
  uint8_t status = RC522_STATUS_ERR;
  uint8_t irqEn = 0x00U;
  uint8_t waitIrq = 0x00U;
  uint8_t n;
  uint8_t lastBits;
  uint8_t i;
  uint16_t loop;

  if (command == RC522_CMD_AUTHENT)
  {
    irqEn = 0x12U;
    waitIrq = 0x10U;
  }
  else if (command == RC522_CMD_TRANSCEIVE)
  {
    irqEn = 0x77U;
    waitIrq = 0x30U;
  }

  RC522_WriteRegister(RC522_REG_COMM_IE_N, (uint8_t)(irqEn | 0x80U));
  RC522_ClearBitMask(RC522_REG_COMM_IRQ, 0x80U);
  RC522_SetBitMask(RC522_REG_FIFO_LEVEL, 0x80U);
  RC522_WriteRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);

  for (i = 0U; i < sendLen; i++)
  {
    RC522_WriteRegister(RC522_REG_FIFO_DATA, sendData[i]);
  }

  RC522_WriteRegister(RC522_REG_COMMAND, command);
  if (command == RC522_CMD_TRANSCEIVE)
  {
    RC522_SetBitMask(RC522_REG_BIT_FRAMING, 0x80U);
  }

  loop = 2000U;
  do
  {
    n = RC522_ReadRegister(RC522_REG_COMM_IRQ);
    loop--;
  } while ((loop != 0U) && ((n & 0x01U) == 0U) && ((n & waitIrq) == 0U));

  RC522_ClearBitMask(RC522_REG_BIT_FRAMING, 0x80U);

  if (loop == 0U)
  {
    return RC522_STATUS_ERR;
  }

  if ((RC522_ReadRegister(RC522_REG_ERROR) & 0x1BU) != 0U)
  {
    return RC522_STATUS_ERR;
  }

  status = RC522_STATUS_OK;
  if ((n & irqEn & 0x01U) != 0U)
  {
    status = RC522_STATUS_NOTAG;
  }

  if (command == RC522_CMD_TRANSCEIVE)
  {
    n = RC522_ReadRegister(RC522_REG_FIFO_LEVEL);
    lastBits = (uint8_t)(RC522_ReadRegister(RC522_REG_CONTROL) & 0x07U);

    if (lastBits != 0U)
    {
      *backLen = (uint16_t)((n - 1U) * 8U + lastBits);
    }
    else
    {
      *backLen = (uint16_t)(n * 8U);
    }

    if (n == 0U)
    {
      n = 1U;
    }
    if (n > RC522_MAX_LEN)
    {
      n = RC522_MAX_LEN;
    }

    for (i = 0U; i < n; i++)
    {
      backData[i] = RC522_ReadRegister(RC522_REG_FIFO_DATA);
    }
  }

  return status;
}

static uint8_t RC522_Request(uint8_t reqMode, uint8_t* tagType)
{
  uint8_t status;
  uint16_t backBits;

  RC522_WriteRegister(RC522_REG_BIT_FRAMING, 0x07U);
  tagType[0] = reqMode;

  status = RC522_ToCard(RC522_CMD_TRANSCEIVE, tagType, 1U, tagType, &backBits);
  if ((status != RC522_STATUS_OK) || (backBits != 0x10U))
  {
    status = RC522_STATUS_ERR;
  }

  return status;
}

static uint8_t RC522_Anticoll(uint8_t* serNum)
{
  uint8_t status;
  uint8_t i;
  uint8_t check = 0U;
  uint16_t recvBits;

  RC522_WriteRegister(RC522_REG_BIT_FRAMING, 0x00U);

  serNum[0] = RC522_PICC_ANTICOLL;
  serNum[1] = 0x20U;

  status = RC522_ToCard(RC522_CMD_TRANSCEIVE, serNum, 2U, serNum, &recvBits);
  if (status != RC522_STATUS_OK)
  {
    return status;
  }

  for (i = 0U; i < 4U; i++)
  {
    check ^= serNum[i];
  }

  if (check != serNum[4])
  {
    return RC522_STATUS_ERR;
  }

  return RC522_STATUS_OK;
}

static void RC522_Halt(void)
{
  uint8_t buffer[4];
  uint16_t recvBits;

  buffer[0] = RC522_PICC_HALT;
  buffer[1] = 0x00U;
  RC522_CalculateCrc(buffer, 2U, &buffer[2]);
  (void)RC522_ToCard(RC522_CMD_TRANSCEIVE, buffer, 4U, buffer, &recvBits);
}

static void RC522_AntennaOn(void)
{
  uint8_t value = RC522_ReadRegister(RC522_REG_TX_CONTROL);

  if ((value & 0x03U) == 0U)
  {
    RC522_SetBitMask(RC522_REG_TX_CONTROL, 0x03U);
  }
}

void RC522_Init(void)
{
  RC522_CsHigh();
  RC522_SckLow();
  RC522_MosiWrite(0U);

  RC522_WriteRegister(RC522_REG_COMMAND, RC522_CMD_RESETPHASE);
  RC522_WriteRegister(RC522_REG_T_MODE, 0x8DU);
  RC522_WriteRegister(RC522_REG_T_PRESCALER, 0x3EU);
  RC522_WriteRegister(RC522_REG_T_RELOAD_L, 30U);
  RC522_WriteRegister(RC522_REG_T_RELOAD_H, 0U);
  RC522_WriteRegister(RC522_REG_RF_CFG, 0x7FU);
  RC522_WriteRegister(RC522_REG_RX_THRESHOLD, 0x84U);
  RC522_WriteRegister(RC522_REG_TX_AUTO, 0x40U);
  RC522_WriteRegister(RC522_REG_MODE, 0x3DU);

  RC522_AntennaOn();
}

uint8_t RC522_ReadUid(uint8_t uid[4])
{
  uint8_t status;
  uint8_t tagType[2];
  uint8_t serial[5];

  status = RC522_Request(RC522_PICC_REQIDL, tagType);
  if (status != RC522_STATUS_OK)
  {
    return 0U;
  }

  status = RC522_Anticoll(serial);
  if (status != RC522_STATUS_OK)
  {
    return 0U;
  }

  uid[0] = serial[0];
  uid[1] = serial[1];
  uid[2] = serial[2];
  uid[3] = serial[3];

  RC522_Halt();
  return 1U;
}
