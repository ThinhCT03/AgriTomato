/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define NSS_Pin GPIO_PIN_0
#define NSS_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_1
#define RST_GPIO_Port GPIOB
#define DIO0_Pin GPIO_PIN_10
#define DIO0_GPIO_Port GPIOB
#define DIO0_EXTI_IRQn EXTI15_10_IRQn
#define light_relay_Pin GPIO_PIN_9
#define light_relay_GPIO_Port GPIOA
#define water_relay_Pin GPIO_PIN_10
#define water_relay_GPIO_Port GPIOA
#define fan_relay_Pin GPIO_PIN_11
#define fan_relay_GPIO_Port GPIOA
#define light_sw_Pin GPIO_PIN_3
#define light_sw_GPIO_Port GPIOB
#define water_sw_Pin GPIO_PIN_4
#define water_sw_GPIO_Port GPIOB
#define fan_sw_Pin GPIO_PIN_5
#define fan_sw_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
