/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "CANOpen.h"

CAN_HandleTypeDef hcan;

/* Private function prototypes -----------------------------------------------
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
*/

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define led_error  0x01
#define led_status 0x02
#define send_msg 0x01
#define n_Sync_Object 1

void init_controller_STM32F042(void);
uint8_t CAN_transmit (CAN_TxMailBox_TypeDef *tx);
void Processing_SDO_Object(CAN_FIFOMailBox_TypeDef*);

CAN_FIFOMailBox_TypeDef rx_mailbox;
CAN_TxMailBox_TypeDef 	tx_mailbox;
// gpio input
uint32_t gpio_time_pin[8]={0};
uint32_t gpio_time_ms[8]={10,10,10,10,10,10,10,10};
uint32_t gpio_input = 0;
uint32_t test_time = 0;
uint16_t blink1 = 0, blink2 = 0;


// CanOpen
uint32_t can_id,can_speed,id_rxPDO1,id_txPDO1,id_rxSDO,id_txSDO,heartbroken;
uint8_t  NMT_command = 0,NMT_status = NMT_status_Operational;

uint32_t *sync_data,*Sync_obj[n_Sync_Object]={NULL};
uint8_t  mask_gpio = 0xFF, send_txPDO1 = 0,send_txSDO = 0,
		 reply_rxPDO1 = 0,reply_rxPDO1_mask = 0;
uint32_t id  = 0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t  command,sub_index,toggle_bit = 0,tg=0;
	uint16_t index, cycle2=0;
	uint32_t id_message = 0, dlc_message =0;


	init_controller_STM32F042();// Init RCC 48Mhz, GPIOx, bxCAN
	SystemCoreClockUpdate();
	HAL_InitTick(TICK_INT_PRIORITY);

	//GPIOB->BSRR = led_status; //

   //Bootup Protocol

   tx_mailbox.TIR = ((0x700 + can_id)<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
   tx_mailbox.TDTR = 1;// n на_отправку
   tx_mailbox.TDLR = 0;// d0-d3
   tx_mailbox.TDHR = 0;// d4-d7

   CAN_transmit(&tx_mailbox);

  //GPIOB->BRR = led_status; // Block on
   gpio_input = GPIOA->IDR & 0xff;
   send_txPDO1 = 0;

  while (1)
  {
    /* USER CODE END WHILE */

/*

	  HAL_Delay(1);

	  if(!cycle2){

		  tg=tg?0:1;

		  if (tg){GPIOB->BSRR = 0x02;} else{GPIOB->BRR = 0x02;};

		  cycle2 = 200;
	  };
	  cycle2--;
*/



      if(test_time > 10){
    	  test_time = 0;
    	  TIM3->CCR3 = blink1;
    	  blink1++;
    	  blink1 = blink1>200?0:blink1;
      };



	  if(send_txPDO1){

		  	 send_txPDO1 --;
		  	 tx_mailbox.TIR = (id_txPDO1 <<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
		     tx_mailbox.TDTR = 1;// n на_отправку
		     tx_mailbox.TDLR = gpio_input;// d0-d3
		     tx_mailbox.TDHR = 0;// d4-d7

		     if(CAN_transmit(&tx_mailbox) != 0){
		    	 send_txPDO1 = 1;
		    	 //GPIOB->BRR = led_error;
		     }else{GPIOB->BSRR = led_error;}


	  };
	  if(reply_rxPDO1){

		  /*
	  		  	 reply_rxPDO1 --;
	  		  	 tx_mailbox.TIR = (id_txPDO1 <<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
	  		     tx_mailbox.TDTR = 1;// n на_отправку
	  		     tx_mailbox.TDLR = gpio_input & reply_rxPDO1_mask;// d0-d3
	  		     tx_mailbox.TDHR = 0;// d4-d7

	  		     if(CAN_transmit(&tx_mailbox) != 0){
	  		    	 reply_rxPDO1 = 1;
	  		    	 //GPIOB->BRR = led_error;
	  		     }else{GPIOB->BSRR = led_error;}
*/

		  TIM3->CCR4 = reply_rxPDO1_mask;


	  	  };


    /* USER CODE BEGIN 3 */
  }




  /* USER CODE END 3 */
}



/* USER CODE BEGIN 4 */



void TIM14_IRQHandler(){

	uint32_t const_bit = 0x01,
			 send_PDO = 0,
			 gpio_input_now = GPIOA->IDR&0xff;

     test_time++;

	for(uint32_t i=0; i < 8;i++){

		if((gpio_input_now&const_bit) == (gpio_input&const_bit)){
			if(gpio_time_pin[i])gpio_time_pin[i]=0;
	   }else{
		    gpio_time_pin[i]++;
		    if(gpio_time_pin[i] == gpio_time_ms[i]){
		    	gpio_input &= (~const_bit);
		    	gpio_input |= (gpio_input_now&const_bit);
		    	send_PDO = 1;}
		}
		const_bit <<= 1;
	};
	send_txPDO1 += send_PDO;
	TIM14->SR &= ~TIM_SR_UIF;

};


void CEC_CAN_IRQHandler(void){

	uint32_t id,dlc;
	 uint8_t data0;

			if((CAN->RF0R & 0b0011) == 0) goto exit; //no message
			// индефикатор = тип сообщения ? extd : std
			id = CAN->sFIFOMailBox[0].RIR&0b0100?CAN->sFIFOMailBox[0].RIR >> 3:CAN->sFIFOMailBox[0].RIR >> 21;
			dlc = CAN->sFIFOMailBox[0].RDTR &0b01111;// получение из регистра длины кадра
			if((CAN->sFIFOMailBox[0].RIR & 0b010) == 0 && dlc != 0){// тип сообщения 0 - data / 1- rtr

				//dlc = dlc>8? 8:dlc;
				data0 = CAN->sFIFOMailBox[0].RDLR&0xff;

				switch (id){

					 case 0:
					  if(dlc < 2) break;
					  if(data0 == 0 || data0 == can_id ){
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
					 if(dlc){reply_rxPDO1_mask = data0;
					 }else{reply_rxPDO1_mask = 0;}
					 reply_rxPDO1 = 1;
				     break;
				};
			};
	exit:	SET_BIT(CAN->RF0R, CAN_RF0R_RFOM0);//CAN->RF0R |= 0b0100000;  сообщение прочитано.
};
void TIM3_IRQHandler(void)
{

    TIM3->SR &= ~TIM_SR_UIF; // Сброс флага прерывания от обновления таймера TIM3
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



  /******************** GPIO ************************************
	 *
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

	//GPIOA->BRR = 0x00FF; // bit0-7 size 8bit Rele

	GPIOB->BRR = 0x0003;// bit 0-1 Led_Status_Pin/Led_Error_Pin

	//пины 0-7 порта  А7A6A5A4A3A2A1A0  на выход
    //GPIOA->MODER |=0x5555; //0b010101010101010101: General purpose output mode
    //GPIOA->OTYPER &=0b1111111100000000;//0 - push-pull, reset default

	GPIOA->OSPEEDR |= 0xFFFF;//0b0101010101010101  high speed

	/*
    // пины Led_Status_Pin bit 0| Led_Error_Pin bit 1

    GPIOB->MODER |= 0x05;//0b0101 01 -General purpose output mode
    GPIOB->OTYPER |=0x03; // 0b11 1 - open drain
    GPIOB->OSPEEDR |= 0x05; //0101  speed medium

	 */

    //id_can pull_down  b3-9

    GPIOB->PUPDR |= 0xAAA80;//0b10101010101010000000;

	//speed pull_down  pin A10,A9,A8

	GPIOA->PUPDR |= 0x2A0000;//0b1010100000000000000000;


/***************************** EXTIxx  *********************************
 *
 * SYSCFG_EXTICR1 exti0-3
 * SYSCFG_EXTICR2 exti4-7
 * SYSCFG_EXTICR3 exti8-11
 * SYSCFG_EXTICR4 ext12-15
  Bits 31:16 Reserved, must be kept at reset value.
  Bits 15:0 EXTIx[3:0]: EXTI x configuration bits (x = 0 to 3)
	These bits are written by software to select the source input for the EXTIx external interrupt.
		x000: PA[x] pin
		x001: PB[x] pin
		x010: PC[x] pin
		x011: PD[x] pin
		x100: PE[x] pin
		x101: PF[x] pin
			other configurations: reserved

   SYSCFG_ITLINE5
   	   Bit 1 EXTI1: EXTI line 1 interrupt request pending
   	   Bit 0 EXTI0: EXTI line 0 interrupt request pending
   SYSCFG_ITLINE6
   	   Bit 1 EXTI3: EXTI line 3 interrupt request pending
   	   Bit 0 EXTI2: EXTI line 2 interrupt request pending
   SYSCFG_ITLINE7
		Bit 11 EXTI15: EXTI line 15 interrupt request pending
   	   	Bit 10 EXTI14: EXTI line 14 interrupt request pending
		Bit 9 EXTI13: EXTI line 13 interrupt request pending
		Bit 8 EXTI12: EXTI line 12 interrupt request pending
		Bit 7 EXTI11: EXTI line 11 interrupt request pending
		Bit 6 EXTI10: EXTI line 10 interrupt request pending
		Bit 5 EXTI9: EXTI line 9 interrupt request pending
		Bit 4 EXTI8: EXTI line 8 interrupt request pending
		Bit 3 EXTI7: EXTI line 7 interrupt request pending
		Bit 2 EXTI6: EXTI line 6 interrupt request pending
		Bit 1 EXTI5: EXTI line 5 interrupt request pending
		Bit 0 EXTI4: EXTI line 4 interrupt request pending

	Interrupt mask register (EXTI_IMR)

	    bit 0 -15 exti0-15
		Address offset: 0x00
		Reset value: 0x0FF4 0000 (STM32F03x devices)
		0x7FF4 0000 (STM32F04x devices)
		0x0F94 0000 (STM32F05x devices)
		0x7F84 0000 (STM32F07x and STM32F09x devices)

		Bits 31:0 IMx: Interrupt Mask on line x (x = 31 to 0)
		0: Interrupt request from Line x is masked
		1: Interrupt request from Line x is not masked

		Event mask register (EXTI_EMR)
			Address offset: 0x04
			Reset value: 0x0000 0000

		Bits 31:0 EMx: Event mask on line x (x = 31 to 0)
			0: Event request from Line x is masked
			1: Event request from Line x is not masked

		Rising trigger selection register (EXTI_RTSR)  _|
		Falling trigger selection register (EXTI_FTSR) |_

			Address offset: 0x08
			Reset value: 0x0000 0000
			Bits 17:0 RTx: Rising trigger event configuration bit of line x (x = 17 to 0)
				0: Rising trigger disabled (for Event and Interrupt) for input line
				1: Rising trigger enabled (for Event and Interrupt) for input line.

		Software interrupt event register (EXTI_SWIER)
			Address offset: 0x10
			Reset value: 0x0000 0000
			Bits 17:0 SWIx: Software interrupt on line x (x = 17 to 0)
							If the interrupt is enabled on this line in the EXTI_IMR, writing a ‘1’ to this bit when it is at ‘0’
							sets the corresponding pending bit in EXTI_PR resulting in an interrupt request generation.
							This bit is cleared by clearing the corresponding bit of EXTI_PR (by writing a ‘1’ to the bit).
		Pending register (EXTI_PR)
			Address offset: 0x14
			Reset value: undefined
			PIFx: Pending bit on line x (x = 17 to 0)
				0: no trigger request occurred
				1: selected trigger request occurred
				This bit is set when the selected edge event arrives on the external interrupt line. This bit is
				cleared by writing a 1 to the bit.

 */
/******************************  TIM14 **************************/

	    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	    TIM14->PSC =  48000000/1000 - 1;
	    TIM14->DIER |= TIM_DIER_UIE; 	  // enable perepolnenia TIM14
	    TIM14->CR1 |= TIM_CR1_CEN;		  // ON TIM14
	    TIM14->ARR = 1; //1 ms

	    NVIC_EnableIRQ(TIM14_IRQn);



/*---------------  PWM  TIM3 ---------------
 Bits 6:4 OC1M: Output compare 1 mode
These bits define the behavior of the output reference signal OC1REF from which OC1 and
OC1N are derived. OC1REF is active high whereas OC1 and OC1N active level depends on
CC1P and CC1NP bits.
000: Frozen - The comparison between the output compare register TIMx_CCR1 and the
counter TIMx_CNT has no effect on the outputs.(this mode is used to generate a timing base).
001: Set channel 1 to active level on match. OC1REF signal is forced high when the counter
TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
010: Set channel 1 to inactive level on match. OC1REF signal is forced low when the
counter TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
011: Toggle - OC1REF toggles when TIMx_CNT=TIMx_CCR1.
100: Force inactive level - OC1REF is forced low.
101: Force active level - OC1REF is forced high.
110: PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
else inactive. In downcounting, channel 1 is inactive (OC1REF=‘0) as long as
TIMx_CNT>TIMx_CCR1 else active (OC1REF=1).
111: PWM mode 2 - In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1
else active. In downcounting, channel 1 is active as long as TIMx_CNT>TIMx_CCR1 else
inactive.

 *
 *
 *
 *
 * */

	    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	    TIM3->PSC = 120 - 1;
	    TIM3->ARR = 200-1;
	    GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;  // PB0 и PB1 в режим альтернативной функции
	    GPIOB->AFR[0] |= 0b00010001; // TIM_CH3, TIM_CH4
	    TIM3->CR1 |= 0x80;
	    TIM3->CCMR2 |= 0b110 << 4 | 0b110 << 12;//Bits 6:4 OC1M PWM 1
	    TIM3->CCER |= TIM_CCER_CC3E|TIM_CCER_CC3P| TIM_CCER_CC4E|TIM_CCER_CC4P; // output ~low _
	    TIM3->CCR3 = 10; //5%
	    TIM3->CCR4 = 50; //25%
	    // Включение таймера 3
	    TIM3->CR1 |= TIM_CR1_CEN;
	    //NVIC_EnableIRQ(TIM3_IRQn);




/***************************** CAN module ********************************
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

	//GPIOB->BRR = led_error; //on_error

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
	id_txSDO = txSDO + can_id;
	heartbroken = 0x700 + can_id;


	CAN->FMR |=  CAN_FMR_FINIT; /*  init mode */
	CAN->FA1R |= (CAN_FA1R_FACT0|CAN_FA1R_FACT1);//Activate filter 0,1
	CAN->FM1R |= (CAN_FM1R_FBM0 |CAN_FM1R_FBM1); //ID_list
	CAN->FS1R  &=~(CAN_FS1R_FSC0|CAN_FS1R_FSC1);  //16bit
	CAN->FFA1R &=~CAN_FFA1R_FFA0; // filtr0 -> FIFO0
	CAN->FFA1R |= CAN_FFA1R_FFA1; // filtr1 -> FIFO1

	CAN->sFilterRegister[0].FR1 = id_rxPDO1<< 5;
	CAN->sFilterRegister[0].FR2 = 0;
	CAN->sFilterRegister[1].FR1 = (id_rxSDO<<16 | heartbroken)<< 5;//0x02 test rtr bit for heartbroken
	CAN->sFilterRegister[1].FR2 = (0x80 << 16 | 0x100 << 16)<< 5;

	CAN->FMR &=~ CAN_FMR_FINIT; /* Leave filter init */
	CAN->IER |= CAN_IER_FMPIE0; /* Set FIFO0 message pending IT enable */


	/*выйти в нормальный режим*/

	CAN->MCR &=~ CAN_MCR_INRQ;
	while((CAN->MSR & CAN_MSR_INAK)==CAN_MSR_INAK){}

	//GPIOB->BSRR = led_error; //off_error
/*
  	Bit 4 FMPIE1: FIFO1 message pending interrupt enable
	Bit 1 FMPIE0: FIFO0 message pending interrupt enable
	    0: No interrupt generated when state of FMP[1:0] bits are not 00b.
	    1: Interrupt generated when state of FMP[1:0] bits are not 00b.
*/

    /* CAN interrupt Init */
	NVIC_SetPriority(CEC_CAN_IRQn,1);
	NVIC_EnableIRQ(CEC_CAN_IRQn);

}

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



/* USER CODE END 4 */

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
