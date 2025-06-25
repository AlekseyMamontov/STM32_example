/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#include "stm32f0xx.h"
#include <stddef.h>
#include "CANOpen.h"





struct Adapter_DCmotor{

	uint16_t 	      num;
	uint16_t 	  in_data; // addr 0x2000
	uint16_t  in1_drv_pwm;
	uint16_t  in2_drv_pwm;
	uint16_t in1_drv2_pwm;
	uint16_t in2_drv2_pwm;
	uint32_t 	   can_id;
	uint32_t    can_speed;
	uint32_t    timer_psc;
	uint32_t    timer_arr;
	uint8_t    gpio_ms[4];
	uint32_t 	     none;

};
void init_controller_STM32F042(void);
uint8_t CAN_transmit (CAN_TxMailBox_TypeDef *tx);
//void Processing_SDO_Object(CAN_FIFOMailBox_TypeDef*);
void SDO_object(struct Adapter_DCmotor *DCmotor);

//#define ADDR_SAVE_FLASH 0x08003C00 //042x4 16 кб
#define ADDR_SAVE_FLASH   0x08007C00 //042x6 32 кб

void Load_default_data(struct Adapter_DCmotor *ram_data);
void init_CAN_module();
void Error_Handler(void);



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
