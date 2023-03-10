/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define EEPROM_CS_Pin GPIO_PIN_14
#define EEPROM_CS_GPIO_Port GPIOC
#define BTN_BW_A_Pin GPIO_PIN_0
#define BTN_BW_A_GPIO_Port GPIOC
#define LED_BW_A_Pin GPIO_PIN_1
#define LED_BW_A_GPIO_Port GPIOC
#define LS2_A_Pin GPIO_PIN_2
#define LS2_A_GPIO_Port GPIOC
#define LS1_A_Pin GPIO_PIN_3
#define LS1_A_GPIO_Port GPIOC
#define DIR_A_Pin GPIO_PIN_2
#define DIR_A_GPIO_Port GPIOA
#define STEP_A_Pin GPIO_PIN_3
#define STEP_A_GPIO_Port GPIOA
#define SLEEP_A_Pin GPIO_PIN_5
#define SLEEP_A_GPIO_Port GPIOA
#define RESET_A_Pin GPIO_PIN_6
#define RESET_A_GPIO_Port GPIOA
#define MS3_A_Pin GPIO_PIN_7
#define MS3_A_GPIO_Port GPIOA
#define MS2_A_Pin GPIO_PIN_4
#define MS2_A_GPIO_Port GPIOC
#define MS1_A_Pin GPIO_PIN_5
#define MS1_A_GPIO_Port GPIOC
#define ENABLE_A_Pin GPIO_PIN_0
#define ENABLE_A_GPIO_Port GPIOB
#define DIR_B_Pin GPIO_PIN_1
#define DIR_B_GPIO_Port GPIOB
#define LS2_B_Pin GPIO_PIN_2
#define LS2_B_GPIO_Port GPIOB
#define LS1_B_Pin GPIO_PIN_10
#define LS1_B_GPIO_Port GPIOB
#define BTN_FW_A_Pin GPIO_PIN_13
#define BTN_FW_A_GPIO_Port GPIOB
#define STEP_B_Pin GPIO_PIN_14
#define STEP_B_GPIO_Port GPIOB
#define SLEEP_B_Pin GPIO_PIN_15
#define SLEEP_B_GPIO_Port GPIOB
#define RESET_B_Pin GPIO_PIN_8
#define RESET_B_GPIO_Port GPIOC
#define MS3_B_Pin GPIO_PIN_9
#define MS3_B_GPIO_Port GPIOC
#define MS2_B_Pin GPIO_PIN_8
#define MS2_B_GPIO_Port GPIOA
#define MS1_B_Pin GPIO_PIN_9
#define MS1_B_GPIO_Port GPIOA
#define ENABLE_B_Pin GPIO_PIN_10
#define ENABLE_B_GPIO_Port GPIOA
#define LED_FW_B_Pin GPIO_PIN_11
#define LED_FW_B_GPIO_Port GPIOA
#define BTN_FW_B_Pin GPIO_PIN_12
#define BTN_FW_B_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define ETH_INT_Pin GPIO_PIN_2
#define ETH_INT_GPIO_Port GPIOD
#define ETH_RST_Pin GPIO_PIN_4
#define ETH_RST_GPIO_Port GPIOB
#define BTN_BW_B_Pin GPIO_PIN_7
#define BTN_BW_B_GPIO_Port GPIOB
#define LED_BW_B_Pin GPIO_PIN_8
#define LED_BW_B_GPIO_Port GPIOB
#define LED_FW_A_Pin GPIO_PIN_9
#define LED_FW_A_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define IP_ADDR {192,168,1,15}
#define IP_GATE {192,168,1,1}
#define IP_MASK {255,255,255,0}
#define LOCAL_PORT 80
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
