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
#define DIO0_Pin GPIO_PIN_2
#define DIO0_GPIO_Port GPIOB
#define DIO0_EXTI_IRQn EXTI2_IRQn
#define Fan_SW_Pin GPIO_PIN_12
#define Fan_SW_GPIO_Port GPIOB
#define Fan_SW_EXTI_IRQn EXTI15_10_IRQn
#define Next_State_Pin GPIO_PIN_13
#define Next_State_GPIO_Port GPIOB
#define Next_State_EXTI_IRQn EXTI15_10_IRQn
#define Prev_State_Pin GPIO_PIN_14
#define Prev_State_GPIO_Port GPIOB
#define Prev_State_EXTI_IRQn EXTI15_10_IRQn
#define Manual_Pin GPIO_PIN_15
#define Manual_GPIO_Port GPIOB
#define Manual_EXTI_IRQn EXTI15_10_IRQn
#define Light_SW_Pin GPIO_PIN_3
#define Light_SW_GPIO_Port GPIOB
#define Light_SW_EXTI_IRQn EXTI3_IRQn
#define Water_pump_Pin GPIO_PIN_4
#define Water_pump_GPIO_Port GPIOB
#define Water_pump_EXTI_IRQn EXTI4_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
