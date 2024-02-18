
#include "main.h"
/* USER CODE BEGIN Includes */
#include "CANOpen.h"
#include "tft_panel_8bit.h"
#include "tft_screens.h"
#include "tft_widgets.h"



#define n_Sync_Object 1



/* USER CODE BEGIN PFP */
void init_controller_STM32F072(void);
uint8_t CAN_transmit (CAN_TxMailBox_TypeDef *tx);
void Processing_SDO_Object(CAN_FIFOMailBox_TypeDef*);


// Keys b0 - down b1 - OK  b2 - up
uint16_t tft_keys[3] = {0};
uint16_t keys_status = 0;
uint8_t  buffer_keys[8];
uint32_t gpio_time_pin[3]={0};
uint32_t gpio_time_ms[3]={5,5,5};
uint32_t gpio_input = 0;

uint8_t  test,rx_msg;
uint16_t test_color;
uint32_t rx_t1 = 0, rx_t2 = 0;

uint16_t ms_pause = 0;

// Input

#define PRODUCT_COUNTER 1
#define CYLINDR 2

uint16_t input_old = PRODUCT_COUNTER|CYLINDR;
uint16_t input_new = PRODUCT_COUNTER|CYLINDR;

// CanOpen block

uint32_t can_id,can_speed,
		 idRxPDO1,idRxPDO2,idRxPDO3,idRxPDO4,
		 idTxPDO1,idTxPDO2,idTxPDO3,idTxPDO4,
		 idRxSDO,idTxSDO,heartbroken;

uint8_t  NMT_command = 0,NMT_status = NMT_status_Operational,
		 sendTxPDO1 = 0 ;

struct data_in_can_message
matrix_can ={
	.data_in_message = &matrix_data.data,
	.id_object = txPDO2+0x03,
	.bit_offset = 0,
	.n_bits = 16,
	.status = 0,
},
punch_can={
	.data_in_message = &punch_data.data,
	.id_object = txPDO2+0x03,
	.bit_offset = 16,
	.n_bits = 16,
	.status = 0,
},
counter_can={
	.data_in_message = &counter_data.data,
	.id_object = txPDO1+0x03,
	.bit_offset = 0,
	.n_bits = 1,
	.status = 0,
},
cylindr_can={
	.data_in_message = &w_cylindr_animation.data,
	.id_object = txPDO1+0x03,
	.bit_offset = 0,
	.n_bits = 1,
	.status = 0,
};

/*
 				data_frame = CAN->sFIFOMailBox[0].RDHR;
				data_frame <<=32;
				data_frame |= CAN->sFIFOMailBox[0].RDLR;
				mask_frame = (1 << matrix_can.n_bits)-1;
				mask_frame <<=matrix_can.bit_offset;
				data_frame &= mask_frame;
				data_frame >>=matrix_can.bit_offset;
				*(matrix_can.data_in_message) =  data_frame;
 *
 *
 *
 *
uint32_t *sync_data, *Sync_obj[n_Sync_Object]={NULL};
uint8_t  mask_gpio = 0xFF, send_txPDO1 = 0,send_txSDO = 0,
		 reply_rxPDO1 = 0,reply_rxPDO1_mask = 0;
uint32_t id  = 0;

uint8_t tex_buf[20]={0};

*/

struct TFT_panel TFT_CAN_module={

	.init_tft = init_r61581,
	.screens = &Screen1,
    .window = &Panel_win,

};





int main(void)
{
  /* USER CODE BEGIN 1 */
	CAN_FIFOMailBox_TypeDef rx_mailbox;
	CAN_TxMailBox_TypeDef 	tx_mailbox;



  /* USER CODE END 1 */


  /* USER CODE BEGIN Init */


	init_controller_STM32F072();// Init RCC 48Mhz, GPIOx, bxCAN
	init_tft_display(TFT_CAN_module.init_tft);
	tft_fast_clear(TFT_CAN_module.window);
	tft_build_widgets(&TFT_CAN_module);


	 tx_mailbox.TIR = ((0x700 + can_id)<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
	 tx_mailbox.TDTR = 1;// n на_отправку
	 tx_mailbox.TDLR = can_id;// d0-d3
	 tx_mailbox.TDHR = 0;// d4-d7

	CAN_transmit(&tx_mailbox);




  /* USER CODE END Init */


  /* USER CODE BEGIN WHILE */
  test= 0;
  rx_msg = 0;

  tft_pause_ms = 5000;
  uint64_t temp,temp_mask;

  while (1)
  {

 	  if(sendTxPDO1){

	  	 	sendTxPDO1 = 0;
	  	 	keys_status = 0;
	  	 	tx_mailbox.TIR = (idTxPDO1 <<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
	  	 	tx_mailbox.TDTR = 1;// n на_отправку
	  	 	tx_mailbox.TDLR = keys_status;// d0-d3
	  	 	tx_mailbox.TDHR = 0;// d4-d7

	  	 	if(CAN_transmit(&tx_mailbox))sendTxPDO1 = 1;

	  	  };



	  if(matrix_can.status){

		temp = matrix_can.data_frame[1];
		temp <<= 32;
		temp |= matrix_can.data_frame[0];
		temp_mask = (1<< matrix_can.n_bits)-1;
		temp_mask <<= matrix_can.bit_offset;
		temp &=temp_mask;
		temp >>=matrix_can.bit_offset;
		*(matrix_can.data_in_message)= temp;
		matrix_can.status--; // can_frame _ok
		w_matrix_temp.status ++; // display visible

	  };

	  if(punch_can.status){

		temp = punch_can.data_frame[1];
		temp <<= 32;
		temp |= punch_can.data_frame[0];
		temp_mask = (1<< punch_can.n_bits)-1;
		temp_mask <<= punch_can.bit_offset;
		temp &=temp_mask;
		temp >>=punch_can.bit_offset;
		*(punch_can.data_in_message)= temp;
		punch_can.status--;
		w_punch_temp.status ++;

	  };

	  if(counter_can.status){

		temp = counter_can.data_frame[1];
		temp <<= 32;
		temp |= counter_can.data_frame[0];
		temp_mask = (1<< counter_can.n_bits)-1;
		temp_mask <<= counter_can.bit_offset;
		temp &=temp_mask;
		//temp >>=counter_can.bit_offset;
		input_new &=~PRODUCT_COUNTER;
		input_new |=temp?PRODUCT_COUNTER:0;
		// \_
		if((input_old & PRODUCT_COUNTER) == 1 &&
		   (input_new & PRODUCT_COUNTER) == 0){
			*(counter_can.data_in_message) += 1;
			if(*(counter_can.data_in_message) == 1000000000) *(counter_can.data_in_message) = 0;
			w_counter_data.status ++;
		};
			input_old &=~PRODUCT_COUNTER;
			input_old |=(input_new&PRODUCT_COUNTER);
			counter_can.status--;
	  };

	  if(cylindr_can.status){

		temp = cylindr_can.data_frame[1];
		temp <<= 32;
		temp |= cylindr_can.data_frame[0];
		temp_mask = (1<< cylindr_can.n_bits)-1;
		temp_mask <<= cylindr_can.bit_offset;
		temp &=temp_mask;
		input_new &=~CYLINDR;
		input_new |=temp?CYLINDR:0;
		// \_
		if((input_old&CYLINDR) != (input_new&CYLINDR)){
			*(cylindr_can.data_in_message) = temp?0:1;
			w_cylindr.status ++;
		};
			input_old &=~CYLINDR;
			input_old |=(input_new&CYLINDR);
			cylindr_can.status--;
	  };

	  dynamic_build_widgets(&TFT_CAN_module);



  }



}




/*------------------ TIMERS -------------------*/










void SysTick_Handler(void){
	 if(ms_pause)ms_pause --;
	 if(tft_pause_ms)tft_pause_ms--;
};

void TIM14_IRQHandler(){

	uint16_t  gpio = GPIOB->IDR&(tft_key_up|tft_key_down|tft_key_ok); //  key (b0..2)
	uint8_t keys_status = 0;
	tft_keys[pin_key_down] <<= 1;
    tft_keys[pin_key_down] |=(gpio&tft_key_down)>>pin_key_down;
    if(tft_keys[pin_key_down] == 0x00ff) keys_status |= 0x01;

    tft_keys[pin_key_ok] <<= 1;
    tft_keys[pin_key_ok] |=(gpio&tft_key_ok)>>pin_key_ok;
    if(tft_keys[pin_key_ok] == 0x00ff) keys_status |= 0x02;

    tft_keys[pin_key_up] <<= 1;
    tft_keys[pin_key_up] |=(gpio&tft_key_up)>>pin_key_up;
    if(tft_keys[pin_key_up] == 0x00ff) keys_status |= 0x04;;

    if(keys_status ){
    	sendTxPDO1++;
    };
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
			if(id == idRxPDO1){
				if(NMT_status != NMT_status_Operational) break;

					break;
			}
			if( id == matrix_can.id_object ){

				matrix_can.data_frame[0] =CAN->sFIFOMailBox[0].RDLR;
				matrix_can.data_frame[1] =CAN->sFIFOMailBox[0].RDHR;
				matrix_can.dlc_frame = dlc;
				matrix_can.status ++;
			}

			if(id == punch_can.id_object){

				punch_can.data_frame[0] =CAN->sFIFOMailBox[0].RDLR;
				punch_can.data_frame[1] =CAN->sFIFOMailBox[0].RDHR;
				punch_can.dlc_frame = dlc;
				punch_can.status ++;
			}


			if(id == counter_can.id_object){

				counter_can.data_frame[0] =CAN->sFIFOMailBox[0].RDLR;
				counter_can.data_frame[1] =CAN->sFIFOMailBox[0].RDHR;
				counter_can.dlc_frame = dlc;
				counter_can.status ++;

			}
			if(id == cylindr_can.id_object){

				cylindr_can.data_frame[0] =CAN->sFIFOMailBox[0].RDLR;
				cylindr_can.data_frame[1] =CAN->sFIFOMailBox[0].RDHR;
				cylindr_can.dlc_frame = dlc;
				cylindr_can.status ++;

			}
		break;
				};
			};
	exit:	SET_BIT(CAN->RF0R, CAN_RF0R_RFOM0);//CAN->RF0R |= 0b0100000;  сообщение прочитано.
};


#define data_165 0x01 << 8
#define pl_165   0x01 << 9
#define sck_165  0x01 << 10

uint32_t Read_addr_CAN(void){

	   uint8_t addr = 0;
	   uint8_t bit_addr =0;

	   GPIOA->BSRR = pl_165 | sck_165;
	   GPIOA->BRR = pl_165;//low
	   GPIOA->BSRR = pl_165; //high

	      // cycle 8 bit
	      for(uint8_t b=0; b<8 ; b++){

	    	  bit_addr = (GPIOA->IDR&data_165)?1:0;
	          bit_addr <<= b;
	          addr |=bit_addr;
	          GPIOA->BRR = sck_165;
	          GPIOA->BSRR = sck_165;

	      };
	   return addr;
};

void init_controller_STM32F072(void){

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

	//SystemCoreClockUpdate();
	//HAL_InitTick(TICK_INT_PRIORITY);

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
			Bits 31:16 BR[15:0]: Port x reset I/O pin y (y = 15 to 0)
			These bits are write-only. A read to these bits returns the value 0x0000.
				0: No action on the corresponding ODRx bit
				1: Resets the corresponding ODRx bit
					Note: If both BSx and BRx are set, BSx has priority.
			Bits 15:0 BS[15:0]: Port x set I/O pin y (y = 15 to 0)
			These bits are write-only. A read to these bits returns the value 0x0000.
				0: No action on the corresponding ODRx bit
				1: Sets the corresponding ODRx bit

	GPIO alternate function low register (GPIOx_AFRL) Reset value: 0x0000 0000
	GPIO port configuration lock register (GPIOx_LCKR) Reset value: 0x0000 0000

	 */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;// включить порт GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // включить порт GPIOВ

	// Выходы pin A0...A7  (D0-D7) output data
	// a8 - data (input), a9 - pl , a10 -sck (output)


    GPIOA->MODER |=0x145555;  //0b0101000101010101010101: General purpose output mode
    GPIOA->OSPEEDR |= 0x3FFFF;//0b1111111111111111111111: high speed
    //GPIOA->OTYPER &=0b1111111100000000;//0 - push-pull, reset default
    GPIOA->BRR = tft_data; // clear
    GPIOA->BSRR = pl_165 | sck_165; // set 1 pl and sck

    // Keys b0 - down, b1 - ok, b2 - up   - input
    // b7 - RST, b6 - CS, b5 - RS, b4 - WR. b3 - RD  - output
    					       // RSTCSRSWRRDb2b1b0
    GPIOB->MODER |= 0x5540;    //0b0101010101000000 : 01 -General purpose output mode
    GPIOB->OSPEEDR |= 0xFFD5;  //0b1111111111010101 : 7-3 high 2-0 medium speed
    GPIOB->BSRR = tft_RD|tft_WR|tft_RS|tft_CS|tft_RST;



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




    SYSCFG->EXTICR[0] = 0b0000000100010001;// EXT0..3 port B
    EXTI->IMR |= 0b111;
    EXTI->EMR |= 0b111;
    EXTI->RTSR |=0b111;
    EXTI->FTSR |= 0b111;

     */


	/******************************  SYStick **************************/

    SysTick->LOAD = 48000000/1000 - 1;  // (48 mHz / 1000) -1  // 1ms
    SysTick->VAL = 0;  // reset
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    //NVIC_SetPriority(SysTick_IRQn, 2);


    /******************************  TIM14 **************************/

    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	TIM14->PSC =  48000000/1000 - 1;
	TIM14->DIER |= TIM_DIER_UIE; 	  // enable perepolnenia TIM14
	TIM14->CR1 |= TIM_CR1_CEN;		  // ON TIM14
	TIM14->ARR = 2; //2 ms


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





	//   выставить пины B9-B8 на альтернативные функции 0x10

	GPIOB->MODER |= (GPIO_MODER_MODER8_1  | GPIO_MODER_MODER9_1 );

	// скорость порта на максимум

	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR8|GPIO_OSPEEDR_OSPEEDR9;

	//  пины PB8,PB9 включить модуль bxCAN

	GPIOB->AFR[1] |= ((0x04 << GPIO_AFRH_AFSEL8_Pos) | (0x04 << GPIO_AFRH_AFSEL9_Pos));

	// включить модуль Can

	RCC->APB1ENR |= RCC_APB1ENR_CANEN;

	// перевод в режим иницилизации

	SET_BIT(CAN->MCR, CAN_MCR_INRQ);
	while ((CAN->MSR & CAN_MSR_INAK) == 0){};
	// выход из сна.
	CLEAR_BIT(CAN->MCR, CAN_MCR_SLEEP);
	while (CAN->MSR & CAN_MSR_SLAK){};

	// get can_id
	uint32_t speed;
	can_id = Read_addr_CAN();
	can_speed = can_id&0x80?1:0;
	can_id &= 0x7F;

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
	// rxPDO
	idRxPDO1 =  rxPDO1 + can_id;
	idRxPDO2 =  rxPDO2 + can_id;
	idRxPDO3 =  rxPDO3 + can_id;
	idRxPDO4 =  rxPDO4 + can_id;

	// txPDO
	idTxPDO1 = txPDO1 + can_id;
	idTxPDO2 = txPDO2 + can_id;
	idTxPDO3 = txPDO3 + can_id;
	idTxPDO4 = txPDO4 + can_id;

	idRxSDO = rxSDO + can_id;
	idTxSDO = txSDO + can_id;
	heartbroken = 0x700 + can_id;


	CAN->FMR |=  CAN_FMR_FINIT; /*  init mode */
	CAN->FA1R |= (CAN_FA1R_FACT0|CAN_FA1R_FACT1|CAN_FA1R_FACT2);//Activate filter 0,1
	CAN->FM1R |= (CAN_FM1R_FBM0 |CAN_FM1R_FBM1|CAN_FM1R_FBM2); //ID_list
	CAN->FS1R  &=~(CAN_FS1R_FSC0|CAN_FS1R_FSC1|CAN_FS1R_FSC2);  //16bit

	CAN->FFA1R &=~CAN_FFA1R_FFA0; // filtr0 -> FIFO0
	CAN->FFA1R &=~CAN_FFA1R_FFA1; // filtr1 -> FIFO0
	CAN->FFA1R |= CAN_FFA1R_FFA2; // filtr1 -> FIFO2

	// CanOpen rxPDO1,2,3,4
	CAN->sFilterRegister[0].FR1 = (idRxPDO1<< 16 | idRxPDO2)<< 5;
	CAN->sFilterRegister[0].FR2 = (idRxPDO3<< 16 | idRxPDO4)<< 5;
	// NMT command
	CAN->sFilterRegister[1].FR1 = (matrix_can.id_object <<16 | punch_can.id_object) << 5;
	CAN->sFilterRegister[1].FR2	= (counter_can.id_object << 16 | 0) << 5; //counter_can.id_object

	// rxSDO, Heartbroken
	CAN->sFilterRegister[2].FR1 = (idRxSDO<<16 | heartbroken)<< 5;//0x02 test rtr bit for heartbroken
	CAN->sFilterRegister[2].FR2 = (0x80 << 16 | 0x100 )<< 5;



	CAN->FMR &=~ CAN_FMR_FINIT; /* Leave filter init */
	CAN->IER |= CAN_IER_FMPIE0; /* Set FIFO0 message pending IT enable */


	/*выйти в нормальный режим*/

	CAN->MCR &=~ CAN_MCR_INRQ;
	while((CAN->MSR & CAN_MSR_INAK)==CAN_MSR_INAK){}


/*
  	Bit 4 FMPIE1: FIFO1 message pending interrupt enable
	Bit 1 FMPIE0: FIFO0 message pending interrupt enable
	    0: No interrupt generated when state of FMP[1:0] bits are not 00b.
	    1: Interrupt generated when state of FMP[1:0] bits are not 00b.
*/
	 NVIC_EnableIRQ(SysTick_IRQn);
	 NVIC_EnableIRQ(TIM14_IRQn);
    /* CAN interrupt Init */
	NVIC_SetPriority(CEC_CAN_IRQn,1);
	NVIC_EnableIRQ(CEC_CAN_IRQn);
    // Systick

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



///////////////////////////////////////////// OLD //////////////////////////////////////////////////

/*

 	  tft_pause_ms = 2000;
	  while(tft_pause_ms){};

	  w_matrix_temp.func(w_matrix_temp.data);
	  w_punch_temp.func(w_punch_temp.data);
	  w_counter_data.func(w_counter_data.data);

	  matrix_data.data++;
	  punch_data.data++;
	  counter_data.data++;

	 if(matrix_data.data == 1000) matrix_data.data = 0;
	 if(punch_data.data == 1000) punch_data.data = 0;
	 if(counter_data.data == 1000000000) counter_data.data = 0;





 *
 *
 	  if(sendTxPDO1){

	  	 	sendTxPDO1 = 0;
	  	 	keys_status = 0;
	  	 	tx_mailbox.TIR = (idTxPDO1 <<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
	  	 	tx_mailbox.TDTR = 1;// n на_отправку
	  	 	tx_mailbox.TDLR = keys_status;// d0-d3
	  	 	tx_mailbox.TDHR = 0;// d4-d7

	  	 	if(CAN_transmit(&tx_mailbox))sendTxPDO1 = 1;

	  	  };

	  // test

 *
 *
 *
 *
 *
 *
 *
	  if(rx_msg){

		  rx_msg--;


		  switch(test){

	  	  	  case 0:
	  	  		  tft_command((rx_t1&0xff00)>>8);
	  	  	  break;
	  	  	  case 1:
		  		tft_command((rx_t1&0xff00)>>8);
		  		tft_data8((rx_t1&0x00ff0000)>>16);
		  	  break;
		  	  case 2:
		  		tft_command((rx_t1&0xff00)>>8);
		  		tft_data8((rx_t1&0x00ff0000)>>16);
		  		tft_data8((rx_t1&0xFF000000)>>24);
		  	  break;
		  	  case 3:
			  	tft_command((rx_t1&0xff00)>>8);
			  	tft_data8((rx_t1&0x00ff0000)>>16);
			  	tft_data8((rx_t1&0xFF000000)>>24);
			  	tft_data8(rx_t2&0xFF);
		  	  break;
		  	  case 4:
			  	tft_command((rx_t1&0xff00)>>8);
			  	tft_data8((rx_t1&0x00ff0000)>>16);
			  	tft_data8((rx_t1&0xFF000000)>>24);
			  	tft_data8(rx_t2&0xFF);
			  	tft_data8((rx_t2&0xFF00)>>8);

		  	  break;
		  	  case 5:
			  	tft_command((rx_t1&0xff00)>>8);
			  	tft_data8((rx_t1&0x00ff0000)>>16);
			  	tft_data8((rx_t1&0xFF000000)>>24);
			  	tft_data8(rx_t2&0xFF);
			  	tft_data8((rx_t2&0xFF00)>>8);
			  	tft_data8((rx_t2&0xFF0000)>>16);
		  	  break;
		  	  case 6:
			  	tft_command((rx_t1&0xff00)>>8);
			  	tft_data8((rx_t1&0x00ff0000)>>16);
			  	tft_data8((rx_t1&0xFF000000)>>24);
			  	tft_data8(rx_t2&0xFF);
			  	tft_data8((rx_t2&0xFF00)>>8);
			  	tft_data8((rx_t2&0xFF0000)>>16);
			  	tft_data8((rx_t2&0xFF000000)>>24);
		  	  break;
		  	  case 7:
		  		tft_data8((rx_t1&0xff00)>>8);
			  	tft_data8((rx_t1&0x00ff0000)>>16);
			  	tft_data8((rx_t1&0xFF000000)>>24);
			  	tft_data8(rx_t2&0xFF);
			  	tft_data8((rx_t2&0xFF00)>>8);
			  	tft_data8((rx_t2&0xFF0000)>>16);
			  	tft_data8((rx_t2&0xFF000000)>>24);
		  	  break;
		  	  case 8:
				  tft_command(0x36);
				  tft_data8((rx_t1&0xff00)>>8);
				  Panel_win.color_background = ((rx_t1&0xFF000000)>>24)|(rx_t1&0x00FF0000)>>8;
				  //tft_fast_clear(&Panel_win);
				  //tft_init_widgets(&Panel_win,block_Widgets);
		  	  break;
		  	  case 9:

		  		  if((rx_t1&0xff00)>>8 == 1){

		  			  init_tft_display(init_r61581);

		  		  }else if((rx_t1&0xff00)>>8 == 2){

		  			  init_tft_display(r61581_v2);

		  		  }else{

		  			  init_tft_display(init_ili9486);

		  		  };

		  	  break;




		  	  default:
		  	  break;

		  };



	  };


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
