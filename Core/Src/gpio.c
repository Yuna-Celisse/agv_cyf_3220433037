/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/*
 * GPIO 模块负责把板上所有普通引脚配置到正确模式。
 * 这些引脚主要分为几类：
 * 1. 传感器输入：红外循迹、RC522 MISO、超声波 ECHO；
 * 2. 控制输出：电机驱动方向脚、待机脚、RC522 软件 SPI、超声波 TRIG；
 * 3. 指示输出：板载 LED。
 */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  /*
   * 这里只做“静态 GPIO 初始化”：
   * 哪些脚是输入、哪些脚是输出、默认拉高还是拉低，都在这里统一确定。
   * 后续业务模块直接使用这些已经配置好的引脚即可。
   */

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 先打开所有会被使用到的 GPIO 端口时钟。 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 板载 LED 默认先拉高，具体亮灭取决于硬件接法。 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /* 
   * 给电机控制脚、RC522 软件 SPI 脚和超声波 TRIG 脚设置上电默认电平。
   * 这里先统一拉低，避免刚上电时执行器误动作。
   */
  HAL_GPIO_WritePin(GPIOB, HCSR04_TRIG_Pin|RC522_MOSI_Pin|RC522_SCK_Pin|BIN2_Pin
                          |BIN1_Pin|STBY_Pin|AIN1_Pin|AIN2_Pin, GPIO_PIN_RESET);

  /* RC522 的片选默认拉高，表示当前未选中设备。 */
  HAL_GPIO_WritePin(RC522_SDA_GPIO_Port, RC522_SDA_Pin, GPIO_PIN_SET);

  /* LED 配置为普通推挽输出。 */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* 5 路循迹红外探头全部配置为上拉输入。 */
  GPIO_InitStruct.Pin = IR1_Pin|IR2_Pin|IR3_Pin|IR4_Pin
                          |IR5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*
   * 这一组 GPIO 都作为普通推挽输出：
   * - HCSR04_TRIG：触发超声波测距
   * - RC522_MOSI / SCK / SDA：RC522 软件 SPI/片选
   * - BIN2 / BIN1 / STBY / AIN1 / AIN2：电机驱动控制脚
   */
  GPIO_InitStruct.Pin = HCSR04_TRIG_Pin|RC522_MOSI_Pin|RC522_SCK_Pin|RC522_SDA_Pin
                          |BIN2_Pin|BIN1_Pin|STBY_Pin|AIN1_Pin
                          |AIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 超声波 ECHO 用双边沿 EXTI，中断里测量高电平脉宽。 */
  GPIO_InitStruct.Pin = HCSR04_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HCSR04_ECHO_GPIO_Port, &GPIO_InitStruct);

  /* RC522 的 MISO 为输入脚，用于读取软件 SPI 返回数据。 */
  GPIO_InitStruct.Pin = RC522_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RC522_MISO_GPIO_Port, &GPIO_InitStruct);

  /* 配置 EXTI1 优先级，真正使能在超声波模块里完成。 */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
