/* USER CODE BEGIN Header */
/**
      PA12 LED HEART

      FDCAN1 PB9 TX
      FDCAN1 PA11 RX

      USART1 RX PB7
      USART1 TX PB6

      BMP280
      PA9  I2C2 SCL
      PA8  I2C2_SDA

      IIM-42652
      PB15 Spi2 MOSI
      PB14 SPI2 MiSO
      PB13 SPI2 SCK
      PB12 IMU_int1
      PB13 IMU int2
      PA10 SPI2 CS

      LS3MDl
      PA5  SPI1 _SCK
      PA6  SPI1 _MISO
      PA7  SPI1 _MOSI
      PB0  SPI1 _CS
      PB1  ReadyOk
      PB2  Warning

      USART RX
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx.h"

void delay_ms(uint32_t ms);
void SystemClock_Config(void);
void CAN_GPIO_Config(void);
void CAN_Config(void);
void GPIO_INIT(void);

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
#define ENABLE_Dalnomer_Pin GPIO_PIN_8
#define ENABLE_Dalnomer_GPIO_Port GPIOA
#define USART1_RX_RS422_Pin GPIO_PIN_10
#define USART1_RX_RS422_GPIO_Port GPIOA
#define CS_SPI3_Pin GPIO_PIN_6
#define CS_SPI3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
