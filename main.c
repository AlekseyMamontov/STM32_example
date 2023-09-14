/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "CANOpen.h"

/* USER CODE BEGIN PD */
#define Rele_1 0x01
#define Rele_2 0x02
#define Rele_3 0x04
#define Rele_4 0x08
#define Rele_5 0x10
#define Rele_6 0x20
#define Rele_7 0x40
#define Rele_8 0x80
#define Block_rele GPIOA
#define led_error  0x01
#define led_status 0x02



#define send_msg 0x01
#define n_Sync_Object 1
/* USER CODE END PD */

/* USER CODE BEGIN PV */

void init_controller_STM32F042(void);
uint8_t CAN_transmit (CAN_TxMailBox_TypeDef *tx);
void Processing_SDO_Object(CAN_FIFOMailBox_TypeDef*);

CAN_FIFOMailBox_TypeDef rx_mailbox;
CAN_TxMailBox_TypeDef 	tx_mailbox;

uint8_t  gpio_rele = 0, mask_gpio = 0xFF;
uint32_t id  = 0;

// CanOpen
uint32_t can_id,can_speed,id_rxPDO1,id_txPDO1,id_rxSDO,heartbroken;
uint8_t  NMT_command = 0,NMT_status = NMT_status_Operational;

uint32_t *sync_data,*Sync_obj[n_Sync_Object]={NULL};





int main(void){

	uint8_t command,sub_index,toggle_bit = 0, cycle = 0,cycle2=0,tg=0;
	uint16_t index;

	uint32_t id_message = 0, dlc_message =0,rtr_bit, block_data0_3, block_data4_7;

	init_controller_STM32F042();// Init RCC 48Mhz, GPIOx, bxCAN
	SystemCoreClockUpdate();
	HAL_InitTick(TICK_INT_PRIORITY);

	GPIOB->BSRR = led_status; //

   //Bootup Protocol

   tx_mailbox.TIR = ((0x700 + can_id)<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
   tx_mailbox.TDTR = 1;// n на_отправку
   tx_mailbox.TDLR = 0;// d0-d3
   tx_mailbox.TDHR = 0;// d4-d7
   CAN_transmit(&tx_mailbox);

   GPIOB->BSRR = led_status; // Block on

  while (1){

	  if(CAN->RF1R&0b0011){

		  rx_mailbox.RIR = CAN->sFIFOMailBox[1].RIR;
		  rx_mailbox.RDTR = CAN->sFIFOMailBox[1].RDTR;
		  rx_mailbox.RDLR = CAN->sFIFOMailBox[1].RDLR;
		  rx_mailbox.RDHR = CAN->sFIFOMailBox[1].RDHR;
		  SET_BIT(CAN->RF1R, CAN_RF1R_RFOM1);

		  	// Ext  1 / Std 0
			id_message = rx_mailbox.RIR&0b0100?rx_mailbox.RIR >> 3: rx_mailbox.RIR >> 21;
			dlc_message = (rx_mailbox.RDTR &0b01111)?8 : rx_mailbox.RDTR &0b01111;// получение из регистра длины кадра


			switch(id_message){

				case 0x80:  // SYNC

					for(uint8_t i=0;i > n_Sync_Object; i++){
						if(Sync_obj[i] != NULL ) continue;
						*Sync_obj[i]?*Sync_obj[i]-- : 0;
					};

				break;

				case 0x100: // TIME
				break;



				default:

					if(id_rxSDO == id_message){  //SDO

						command = rx_mailbox.RIR&0xEC;
						if(command == 0x40 || 0x20){ Processing_SDO_Object(&rx_mailbox);
						}else{
							rx_mailbox.RDLR = (rx_mailbox.RDLR&0xFFFFFF00)|Error_answer;
							rx_mailbox.RDHR = error_msg[ERROR_SDO_SERVER];}

						tx_mailbox.TIR = ((txSDO + can_id)<< 21)| send_msg;
						tx_mailbox.TDTR = rx_mailbox.RDTR;
					    tx_mailbox.TDLR = rx_mailbox.RDLR;
						tx_mailbox.TDHR = rx_mailbox.RDHR;
						CAN_transmit (&tx_mailbox);

					break;}

					/* Node Guarding (0x700 + id) + rtr
					 * answer (0x700 + id) + dlc=1 + (toggle bit | NMT_status) one bayt
					 * struct byte
 	 	 	 	 	 bit_7    80 toggle_bit   change bit new message
 	 	 	 	 	 bit_0_6  7Fh nmt_state
 	 	  	  	  	  	  	  04h Stopped
		  	  	  	  	  	  05h Operational
		  	  	  	  	  	  7Fh Pre-Operational*/

				  if(heartbroken == id_message){

						if(!(rx_mailbox.RIR&0x02)) break; //=data? exit

					     tx_mailbox.TIR = ((0x700 + can_id)<< 21)| send_msg; // addr,std0,data0,TXRQ - отправка сообщения
					     tx_mailbox.TDTR = 0x01;// n на_отправку
					     tx_mailbox.TDLR = toggle_bit | NMT_status;
					     tx_mailbox.TDHR = 0;// d4-d7
					     CAN_transmit(&tx_mailbox);
					     toggle_bit = toggle_bit?0:80;
					}

			   break;
	   };
   };

	  // test


		  HAL_Delay(1);

		  if(!cycle2){
			  tg=tg?0:1;
			  if (tg){GPIOB->BSRR = 0x02;} else{GPIOB->BRR = 0x02;};
			  cycle2=500;
		  };
		  cycle2--;

		  if(!cycle){

			     tx_mailbox.TIR = ((0x80 + can_id)<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
			     tx_mailbox.TDTR = 2;// n на_отправку
			     tx_mailbox.TDLR = can_id ;// d0-d3
			     tx_mailbox.TDHR = 0;// d4-d7
			     CAN_transmit(&tx_mailbox);

		     cycle = 10000;}

		  cycle --;
  };
};


/*------------------Function_Block_Rele-------------------------------------------------*/

void Processing_SDO_Object(CAN_FIFOMailBox_TypeDef* rx){


};


void CEC_CAN_IRQHandler(void){

	uint32_t id,dlc,gpio = 0;
	uint8_t data0;

		if((CAN->RF0R & 0b0011) == 0) return;
		// индефикатор = тип сообщения ? extd : std
		id = CAN->sFIFOMailBox[0].RIR&0b0100?CAN->sFIFOMailBox[0].RIR >> 3:CAN->sFIFOMailBox[0].RIR >> 21;
		dlc = CAN->sFIFOMailBox[0].RDTR &0b01111;// получение из регистра длины кадра
		if((CAN->sFIFOMailBox[0].RIR & 0b010) == 0 && dlc != 0){// тип сообщения 0 - data / 1- rtr

			//dlc = dlc>8? 8:dlc;
			data0 = CAN->sFIFOMailBox[0].RDLR&0xff;

			switch (id){

				 case 0:
				  if(dlc < 2) break;
				  if(data0 == 0 || data0== can_id ){
					  NMT_command  = (CAN->sFIFOMailBox[0].RDLR&0xff00) >>8;
					  switch(NMT_command){
				  	  	  case NMT_start : NMT_status = NMT_status_Operational;break;
				  	  	  case NMT_pre_operational: NMT_status = NMT_status_Pre_Operational;break;
				  	  	  case NMT_stop: NMT_status = NMT_status_Stopped;break;
				  	  	  default:break;}
				  };
				 break;

				default:
				 if(id != id_rxPDO1) break;
				 if(NMT_status != NMT_status_Operational) break;
				 if(dlc>1) mask_gpio = (CAN->sFIFOMailBox[0].RDLR&0xFF00)>>8;
				 gpio |= ((~data0)&mask_gpio);
				 gpio = gpio<<16;
				 gpio |= (data0&mask_gpio);
				 Block_rele->BSRR = gpio;
			     break;
			};
		};
		SET_BIT(CAN->RF0R, CAN_RF0R_RFOM0);//CAN->RF0R |= 0b0100000;  сообщение прочитано.


};

uint8_t CAN_transmit (CAN_TxMailBox_TypeDef *tx){

	uint8_t n=0;

	if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0){
	}else if((CAN->TSR & CAN_TSR_TME1) == CAN_TSR_TME1){ n=1;
	}else if((CAN->TSR & CAN_TSR_TME2) == CAN_TSR_TME2){ n=2;
	}else return 1; //busy

	CAN->sTxMailBox[n].TDTR = tx->TDTR;
	CAN->sTxMailBox[n].TDLR = tx->TDLR;
	CAN->sTxMailBox[n].TDHR = tx->TDHR;
	CAN->sTxMailBox[n].TIR = tx->TIR ;

	return 0;
}


void init_controller_STM32F042(void){

	/*registr ACR  str.69
	Bit 4 PRFTBE: Prefetch buffer enable
	0: Prefetch is disabled /
	1: Prefetch is enabled
    Bit 3 Reserved, must be kept at reset value.
    Bits 2:0 LATENCY[2:0]: Latency
    These bits represent the ratio of the SYSCLK (system clock) period to the flash access time.
    000: Zero wait state, if SYSCLK ≤ 24 MHz
    001: One wait state, if 24 MHz < SYSCLK ≤ 48 MHz
	 * */
	FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY; // включаем буфер и частота 48мгц

	NVIC_SetPriority(RCC_CRS_IRQn,0);
	NVIC_EnableIRQ(RCC_CRS_IRQn);

	RCC->CR |= RCC_CR_HSEON;  			//включаем генератор HSE
	while(!(RCC->CR & RCC_CR_HSERDY)){};//ожидание готовности HSE
    RCC->CR |= RCC_CR_CSSON; 			//следить за кварцем

	RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV1;  // Prediv предделитель для PLL
	//завести HSE на PLL и 8 Мгц x6 = 48 Мгц
	RCC->CFGR |=(RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL6);

	RCC->CR |= RCC_CR_PLLON;  			//включаем PLL
	while(!(RCC->CR & RCC_CR_PLLRDY));  // ожидание готовности PLL
	RCC->CFGR |= RCC_CFGR_SW_PLL;  		// переводим тактирование от PLL

	//MODIFY_REG(RCC->CFGR,RCC_CFGR_HPRE,RCC_CFGR_HPRE_DIV1);//AHB Prescaler /1
	//MODIFY_REG(RCC->CFGR,RCC_CFGR_PPRE,RCC_CFGR_PPRE_DIV1);//APB1 Prescaler /1,

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // включить блок syscfg
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;	  // управление питанием



	/*
	 GPIO port mode register (GPIOx_MODER)
		Address offset:0x00
		Reset value: 0x2800 0000 for port A
		Reset value: 0x0000 0000 for other ports
			00: Input mode (reset state)
			01: General purpose output mode
			10: Alternate function mode
			11: Analog mode
	 GPIO port output type register (GPIOx_OTYPER)
	 Address offset: 0x04
	 Reset value: 0x0000 0000
			0: Output push-pull (reset state)
			1: Output open-drain

	GPIO port output speed register (GPIOx_OSPEEDR)
		Address offset: 0x08
		Reset value: 0x0C00 0000 (for port A)
		Reset value: 0x0000 0000 (for other ports)
			x0: Low speed
			01: Medium speed
			11: High speed

	GPIO port pull-up/pull-down register (GPIOx_PUPDR)
		Address offset: 0x0C
		Reset value: 0x2400 0000 (for port A)
		Reset value: 0x0000 0000 (for other ports)
			00: No pull-up, pull-down
			01: Pull-up
			10: Pull-down
			11: Reserved
	GPIO port input data register (GPIOx_IDR)
		Address offset: 0x10
		Reset value: 0x0000 XXXX
	GPIO port output data register (GPIOx_ODR)
		Address offset: 0x14
		Reset value: 0x0000 0000
	GPIO port bit set/reset register (GPIOx_BSRR) Reset value: 0x0000 0000
	GPIO alternate function low register (GPIOx_AFRL) Reset value: 0x0000 0000
	GPIO port configuration lock register (GPIOx_LCKR) Reset value: 0x0000 0000

	 */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;// включить порт GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // включить порт GPIOВ

	// Выходы реле и светодиоды статус и реле - сбрасываем.

	GPIOA->BRR = 0x00FF; // bit0-7 size 8bit Rele
	GPIOB->BRR = 0x0003;// bit 0-1 Led_Status_Pin/Led_Error_Pin
	//пины 0-7 порта  А7A6A5A4A3A2A1A0  на выход
    GPIOA->MODER |=0x5555; //0b010101010101010101: General purpose output mode
    //GPIOA->OTYPER &=0b1111111100000000;//0 - push-pull, reset default
    GPIOA->OSPEEDR |= 0x5555;//0b0101010101010101  speed medium
    // пины Led_Status_Pin bit 0| Led_Error_Pin bit 1
    GPIOB->MODER |= 0x05;//0b0101 01 -General purpose output mode
    GPIOB->OTYPER |=0x03; // 0b11 1 - open drain
    GPIOB->OSPEEDR |= 0x05; //0101  speed medium

    //id_can pull_down  b3-9
	GPIOB->PUPDR |= 0xAAA80;//0b10101010101010000000;
	//speed pull_down  pin A10,A9,A8
	GPIOA->PUPDR |= 0x2A0000;//0b1010100000000000000000;

/*
   	CAN GPIO Configuration
   	   	   PA11     ------> CAN_RX
   	   	   PA12     ------> CAN_TX

	GPIO configuration for CAN signals
	   	   (1) Select AF mode (10) on PA11 and PA12
	   	   (2) AF4 for CAN signals
 	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12)) |
	  	  	  	   	   (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);
 	GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH3 | GPIO_AFRH_AFRH4)) |
	  	  	  	     	 (4 << (3 * 4)) | (4 << (4 * 4));

	 CAN master control register (CAN_MCR) Reset value: 0x0001 0002
	 CAN master status register (CAN_MSR)  Reset value: 0x0000 0C02
*/

	// включить порт А
	// SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
    uint32_t speed;

	//   выставить порты на альтернативные функции 0x10
	GPIOA->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);
	// скорость порта на максимум
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR11|GPIO_OSPEEDR_OSPEEDR12;
	//  пины 11,12 включить модуль bxCAN
	GPIOA->AFR[1] |= ((0x04 << GPIO_AFRH_AFSEL11_Pos) | (0x04 << GPIO_AFRH_AFSEL12_Pos));

	// включить модуль Can
	RCC->APB1ENR |= RCC_APB1ENR_CANEN;

	// перевод в режим иницилизации
	GPIOB->BSRR |= led_error; //on_error

	SET_BIT(CAN->MCR, CAN_MCR_INRQ);
	while ((CAN->MSR & CAN_MSR_INAK) == 0){};
	// выход из сна.
	CLEAR_BIT(CAN->MCR, CAN_MCR_SLEEP);
	while (CAN->MSR & CAN_MSR_SLAK){};

	/*// reset default
	CLEAR_BIT(CAN->MCR, CAN_MCR_TTCM);// Clear 0 /Set 1 the time triggered communication mode
	CLEAR_BIT(CAN->MCR, CAN_MCR_ABOM);// Clear 0 /Set 1 the automatic bus-off management
	CLEAR_BIT(CAN->MCR, CAN_MCR_AWUM);// Clear 0 /Set 1 the automatic wake-up mode
	CLEAR_BIT(CAN->MCR, CAN_MCR_NART);// Set 1/ Clear 0 the automatic retransmission
	CLEAR_BIT(CAN->MCR, CAN_MCR_RFLM);// Set the receive FIFO locked mode
	CLEAR_BIT(CAN->MCR, CAN_MCR_TXFP);// Set the transmit FIFO priority
	*/

	// Получение данных

	can_id = ((GPIOB->IDR)&0b0000001111111000)>>3;// b3-b9
	can_speed = ((GPIOA->IDR)&0x700)>>8;
	//can_speed =0;
	switch(can_speed){ // 48 Мгц.

		case 0x01: speed = 0x001c0002;break; //1000kb
		case 0x02: speed = 0x001b0003;break; //800kb
		case 0x03: speed = 0x001c0005;break; //500kb
		case 0x04: speed = 0x001c000b;break; //250kb
		case 0x05: speed = 0x001c0017;break; //125kb
		case 0x06: speed = 0x001c001d;break; //100kb
		case 0x07: speed = 0x001c003b;break; //50kb
		default: speed = 0x001c0005;break;//000 -default 500kb

	};

	WRITE_REG(CAN->BTR,speed);

/*	CAN filter master register (CAN_FMR)  Reset value: 0x2A1C 0E01
 * 		Bit 0 FINIT: Filter initialization mode
		Initialization mode for filter banks
			0: Active filters mode.
			1: Initialization mode for the filters.(reset default)

	CAN filter mode register (CAN_FM1R)  Reset value: 0x0000 0000
		     FBMx: Filter mode
			Mode of the registers of Filter x.
			0: Two 32-bit registers of filter bank x are in Identifier Mask mode.
			1: Two 32-bit registers of filter bank x are in Identifier List mode.

	CAN filter scale register (CAN_FS1R)  Reset value: 0x0000 0000
		This register can be written only when the filter initialization mode is set (FINIT=1) in the
			FSCx: Filter scale configuration
			These bits define the scale configuration of Filters 13-0.
			0: Dual 16-bit scale configuration
			1: Single 32-bit scale configuration

	CAN filter FIFO assignment register (CAN_FFA1R) Reset value: 0x0000 0000
		This register can be written only when the filter initialization mode is set (FINIT=1) in the
		FFAx: Filter FIFO assignment for filter x
			0: Filter assigned to FIFO 0
			1: Filter assigned to FIFO 1

	CAN filter activation register (CAN_FA1R) Reset value: 0x0000 0000
		FACTx: Filter active
		The software sets this bit to activate Filter x. To modify the Filter x registers (CAN_FxR[0:7]),
		the FACTx bit must be cleared or the FINIT bit of the CAN_FMR register must be set.
			0: Filter x is not active
			1: Filter x is active

	Filter bank i register x (CAN_FiRx) (i = 0..13, x = 1, 2) Reset value: 0xXXXX XXXX
		There are 14 filter banks, i= 0 to 13.
		Each filter bank i is composed of two 32-bit registers,
		CAN_FiR[2:1].
		This register can only be modified when the FACTx bit of the CAN_FAxR register is cleared
		or when the FINIT bit of the CAN_FMR register is set.

			Bits 31:0 FB[31:0]: Filter bits

			Each bit of the register specifies the level of the corresponding bit of the expected identifier.
				0: Dominant bit is expected
				1: Recessive bit is expected
			Mask
			Each bit of the register specifies whether the bit of the associated identifier register must
			match with the corresponding bit of the expected identifier or not.
				0: Do not care, the bit is not used for the comparison
				1: Must match, the bit of the incoming identifier mu

*/

	id_rxPDO1 = rxPDO1 + can_id;
	id_txPDO1 = txPDO1 + can_id;
	id_rxSDO =  rxSDO + can_id;
	heartbroken = 0x700 + can_id;


	CAN->FMR |=  CAN_FMR_FINIT; /*  init mode */
	CAN->FA1R |= (CAN_FA1R_FACT0|CAN_FA1R_FACT1);//Activate filter 0,1
	CAN->FM1R |= (CAN_FM1R_FBM0 |CAN_FM1R_FBM1); //ID_list
	CAN->FS1R  &=~(CAN_FS1R_FSC0|CAN_FS1R_FSC1);  //16bit
	CAN->FFA1R &=~CAN_FFA1R_FFA0; // filtr0 -> FIFO0
	CAN->FFA1R |= CAN_FFA1R_FFA1; // filtr1 -> FIFO1

	CAN->sFilterRegister[0].FR1 = id_rxPDO1<< 5;
	CAN->sFilterRegister[0].FR2 = 0;
	CAN->sFilterRegister[1].FR1 = (id_rxSDO<<16 | heartbroken)<< 5 | 0x02;//0x02 test rtr bit for heartbroken
	CAN->sFilterRegister[1].FR2 = (0x80 << 16 | 0x100 << 16)<< 5;

	CAN->FMR &=~ CAN_FMR_FINIT; /* Leave filter init */
	CAN->IER |= CAN_IER_FMPIE0; /* Set FIFO0 message pending IT enable */


	/*выйти в нормальный режим*/

	CAN->MCR &=~ CAN_MCR_INRQ;
	while((CAN->MSR & CAN_MSR_INAK)==CAN_MSR_INAK){}

	GPIOB->BRR |=led_error; //off_error
/*
  	Bit 4 FMPIE1: FIFO1 message pending interrupt enable
	Bit 1 FMPIE0: FIFO0 message pending interrupt enable
	    0: No interrupt generated when state of FMP[1:0] bits are not 00b.
	    1: Interrupt generated when state of FMP[1:0] bits are not 00b.
*/

    /* CAN interrupt Init */
	NVIC_SetPriority(CEC_CAN_IRQn,2);
	NVIC_EnableIRQ(CEC_CAN_IRQn);

}




/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
