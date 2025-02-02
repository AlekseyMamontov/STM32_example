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



void CAN_GPIO_Config(void);
void CAN_Config(void);


//////////////////////
/*Device address*/








#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
