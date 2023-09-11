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
#include "stm32f0xx_hal.h"

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
#define Rele_1_Pin GPIO_PIN_0
#define Rele_1_GPIO_Port GPIOA
#define Rele_2_Pin GPIO_PIN_1
#define Rele_2_GPIO_Port GPIOA
#define Rele_3_Pin GPIO_PIN_2
#define Rele_3_GPIO_Port GPIOA
#define Rele_4_Pin GPIO_PIN_3
#define Rele_4_GPIO_Port GPIOA
#define Rele_5_Pin GPIO_PIN_4
#define Rele_5_GPIO_Port GPIOA
#define Rele_6_Pin GPIO_PIN_5
#define Rele_6_GPIO_Port GPIOA
#define Rele_7_Pin GPIO_PIN_6
#define Rele_7_GPIO_Port GPIOA
#define Rele_8_Pin GPIO_PIN_7
#define Rele_8_GPIO_Port GPIOA
#define Led_Status_Pin GPIO_PIN_0
#define Led_Status_GPIO_Port GPIOB
#define Led_Error_Pin GPIO_PIN_1
#define Led_Error_GPIO_Port GPIOB
#define Speed_b0_Pin GPIO_PIN_8
#define Speed_b0_GPIO_Port GPIOA
#define Speed_b1_Pin GPIO_PIN_9
#define Speed_b1_GPIO_Port GPIOA
#define Speed_b3_Pin GPIO_PIN_10
#define Speed_b3_GPIO_Port GPIOA
#define Addr_b0_Pin GPIO_PIN_3
#define Addr_b0_GPIO_Port GPIOB
#define Addr_b1_Pin GPIO_PIN_4
#define Addr_b1_GPIO_Port GPIOB
#define Addr_b2_Pin GPIO_PIN_5
#define Addr_b2_GPIO_Port GPIOB
#define Addr_b3_Pin GPIO_PIN_6
#define Addr_b3_GPIO_Port GPIOB
#define Addr_b4_Pin GPIO_PIN_7
#define Addr_b4_GPIO_Port GPIOB
#define Addr_b5_Pin GPIO_PIN_8
#define Addr_b5_GPIO_Port GPIOB
#define Addr_b6_Pin GPIO_PIN_9
#define Addr_b6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
