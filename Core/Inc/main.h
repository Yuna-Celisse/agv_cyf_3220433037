/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define IR1_Pin GPIO_PIN_1
#define IR1_GPIO_Port GPIOA
#define IR2_Pin GPIO_PIN_2
#define IR2_GPIO_Port GPIOA
#define IR3_Pin GPIO_PIN_3
#define IR3_GPIO_Port GPIOA
#define IR4_Pin GPIO_PIN_4
#define IR4_GPIO_Port GPIOA
#define IR5_Pin GPIO_PIN_5
#define IR5_GPIO_Port GPIOA
#define HCSR04_TRIG_Pin GPIO_PIN_0
#define HCSR04_TRIG_GPIO_Port GPIOB
#define HCSR04_ECHO_Pin GPIO_PIN_1
#define HCSR04_ECHO_GPIO_Port GPIOB
#define HCSR04_ECHO_EXTI_IRQn EXTI1_IRQn
#define RC522_MISO_Pin GPIO_PIN_12
#define RC522_MISO_GPIO_Port GPIOB
#define RC522_MOSI_Pin GPIO_PIN_13
#define RC522_MOSI_GPIO_Port GPIOB
#define RC522_SCK_Pin GPIO_PIN_14
#define RC522_SCK_GPIO_Port GPIOB
#define RC522_SDA_Pin GPIO_PIN_15
#define RC522_SDA_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_3
#define BIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_5
#define STBY_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_6
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_7
#define AIN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
