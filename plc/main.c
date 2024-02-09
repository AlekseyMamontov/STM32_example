
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "CANOpen.h"


/*---------  define -------- */

//LED
#define LED_Status 0 //port B0
#define LED_Error  1 //port B1

// input
#define IN_220V    2 //port B2

// input portA
#define IN_1  8
#define IN_2  9
#define IN_3  10
#define IN_4  11
#define IN_5  12
#define IN_6  15

//

// output portA
#define OUT0	0
#define OUT1	1
#define OUT2	2
#define OUT3	3
#define OUT4	4
#define OUT5	5
#define OUT6	6
#define OUT7	7

// SPI - MAX6675

#define MISO 14 //PB14
#define SCK  13 //PB13
#define CS1  11 //PB11
#define CS2  12 //PB12
#define pin_CS1 1 << CS1
#define pin_CS2 1 << CS2

#define MAX_IN_PIN 6
#define SEND_temperature 	0x01
#define SPI_read_OK 		0x02
#define Сhange_temperature 	0x04

struct PLC_controller {

	// block Temperature

	uint16_t  temp_period; // ms cycle  ex. 500ms
	uint16_t  temp_pause; //
	uint16_t  temp1;
	uint16_t  temp2;
	uint16_t  temp_read;
	uint16_t* current_temp;
	uint32_t  current_cs;
	uint8_t   temp_status;// b0 -send_temperature, b1-spi_read_ok ,b2 -change temperature.
	CAN_TxMailBox_TypeDef can_send_temp;


	// In_block

	uint16_t pin_input; // 15,12,11,10,9,8 + b2(220v)
	uint16_t pin_reg_shift[MAX_IN_PIN];
	uint16_t pin_time_ms ;
	uint16_t input_mask;

	uint16_t status_pin;
	uint8_t  hold_time_in_pin;
	uint8_t  send_gpio_input;
	CAN_TxMailBox_TypeDef can_send_input;

	// Out_block_rele

	uint16_t rele_pin;
	uint16_t mask_rele_pin;

	// error

	uint16_t error;

	// Can module

	uint32_t can_id;
	uint32_t can_speed;
	CAN_FIFOMailBox_TypeDef rx_mailbox;
	CAN_TxMailBox_TypeDef 	tx_mailbox;

};


// CanOpen
uint32_t can_id,can_id_test = 0,can_speed,id_rxPDO1,id_txPDO1,id_rxSDO,id_txSDO,heartbroken;
uint8_t  NMT_command = 0,NMT_status = NMT_status_Operational;
#define n_Sync_Object 3
uint32_t *sync_data, *Sync_obj[n_Sync_Object]={NULL};
uint8_t  mask_gpio = 0xFF, send_txPDO1 = 0,send_txSDO = 0,
		 reply_rxPDO1 = 0,reply_rxPDO1_mask = 0;
uint32_t id  = 0,temp_can=0;


void read_temperature(struct PLC_controller* plc);
void init_controller_STM32F072(struct PLC_controller* plc);
uint8_t CAN_transmit (CAN_TxMailBox_TypeDef *tx);
void Processing_SDO_Object(CAN_FIFOMailBox_TypeDef*);






struct PLC_controller PLC ={

		// temperature max6675

		.temp_period = 0,
		.temp_pause = 500, // 500 ms
		.temp1 = 0,
		.temp2 = 0,
		.temp_status =SPI_read_OK,
		.current_cs =0,


		.current_cs = 0,

		// input
		.mask_rele_pin = 0xFF,
		.pin_reg_shift={0xFF},
		.hold_time_in_pin = 20,



		// output


};


uint16_t ms_pause = 0;

int main(void){

	 uint8_t led = 1;

	 init_controller_STM32F072(&PLC);// Init RCC 48Mhz, GPIOx, bxCAN

     // Transmit heartbroken

	 PLC.tx_mailbox.TIR = ((0x700 + PLC.can_id)<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
 	 PLC.tx_mailbox.TDTR = 1;// n на_отправку
 	 PLC.tx_mailbox.TDLR = PLC.can_id;// d0-d3
 	 PLC.tx_mailbox.TDHR = 0;// d4-d7

 	 CAN_transmit(&PLC.tx_mailbox);

 	 // Set txPDO1 for temperature

 	 id_txPDO1 = txPDO1 + PLC.can_id;
 	 id_txPDO1 = txPDO2 + PLC.can_id;

 	 PLC.can_send_temp.TIR = ((txPDO2 + PLC.can_id) <<21) | 0x01;// txPDO1
 	 PLC.can_send_temp.TDTR = 4;
 	 PLC.can_send_temp.TDLR = 0;
 	 PLC.can_send_temp.TDHR = 0;

 	 PLC.can_send_input.TIR = ((txPDO1 + PLC.can_id) <<21) | 0x01;
 	 PLC.can_send_input.TDTR = 1;
 	 PLC.can_send_input.TDLR = 0;
 	 PLC.can_send_input.TDHR = 0;



  while (1)
  {


	  if(ms_pause == 0){
	  GPIOB -> BSRR = led?(1<<LED_Status):(1 << (LED_Status+16));
	  ms_pause = 1000;
	  led = led?0:1;
	  };

	  if(send_txPDO1){

	  	 	send_txPDO1 = 0;

	  	 	PLC.tx_mailbox.TIR = (id_txPDO1 <<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
	  	 	PLC.tx_mailbox.TDTR = 2;// n на_отправку
	  	 	PLC.tx_mailbox.TDLR = GPIOA->IDR&0xFFFF;// d0-d3
	  	 	PLC.tx_mailbox.TDHR = 0;// d4-d7

	  	 	if(CAN_transmit(&PLC.tx_mailbox))send_txPDO1 = 1;

	  };

	  read_temperature(&PLC);






  }
}




void SysTick_Handler(void){
	 if(ms_pause)ms_pause --;
	 if(PLC.temp_period)PLC.temp_period --;
};



///////////////// Input Pins /////////////////////

void input_8pin(){

	uint16_t gpio_old = PLC.pin_input << 8,
	         gpio_current = (GPIOA->IDR >>8)&0b10011111;

	for(uint8_t i= 0; i< 8; i++ ){

		PLC.pin_reg_shift[i] <<=1;
		PLC.pin_reg_shift[i] |= gpio_current&1;

		if((PLC.pin_reg_shift[i]&PLC.input_mask) == PLC.input_mask){ // 1

			gpio_current |= 0x100;

		}else if ((PLC.pin_reg_shift[i]&PLC.input_mask) != 0){

			gpio_current |= (gpio_old&0x100);

		};

		gpio_old >>=1;
		gpio_current >>=1;
	};

	if (PLC.pin_input == gpio_current) return;

	PLC.pin_input = gpio_current;

	PLC.can_send_input.TDLR = PLC.pin_input;
	PLC.send_gpio_input = CAN_transmit(&PLC.can_send_input);

};


void TIM14_IRQHandler(){

	//if(PLC.hold_time_in_pin) return;

	input_8pin();

	//PLC.hold_time_in_pin --;

	//if(!PLC.hold_time_in_pin) NVIC_DisabledIRQ(TIM14_IRQn);

	TIM14->SR &= ~TIM_SR_UIF;

};

void EXTI4_15_IRQHandler(){

	PLC.status_pin = EXTI->PR & 0b1001111100000000;//8,9,10,11,12,15
	PLC.hold_time_in_pin = 50;
	EXTI->PR |= PLC.status_pin;
	//NVIC_EnableIRQ(TIM14_IRQn);
};






/////////////// Temperatura //////////////////




void SPI2_IRQHandler(void){

	uint16_t data;

	if (SPI2->SR & SPI_SR_RXNE){

		GPIOB->BSRR = PLC.current_cs;
		data = SPI2->DR;
		PLC.temp_read = ((data&0xFF)<<8)|((data&0xFF00)>>8);
		PLC.temp_status |= SPI_read_OK;
	};

	SPI2->SR &= ~SPI_SR_RXNE;
};
// b0 -send_temperature, b1-spi_read_ok ,b2 -change temperature.



void read_temperature(struct PLC_controller* plc){

	  if(plc->temp_period) return;
	  if(!(plc->temp_status&SPI_read_OK)) return; //read spi ok

		  	switch (plc->current_cs){

		  		  case pin_CS1:

		  			  plc->temp_read >>=5;

		  			  if(plc->temp_read != plc->temp1){
		  				  plc->temp_status |=Сhange_temperature;//change
		  			  	  plc->temp1 = plc->temp_read;}
		  			  plc->current_cs = pin_CS2;

		  			  break;

		  		  case pin_CS2:

		  			  plc->temp_read >>=5;

		  			  if(plc->temp_read != plc->temp2){
		  				  plc->temp_status |=Сhange_temperature;//change
		  			  	  plc->temp2 = plc->temp_read;}

		  			  plc->temp_period = plc->temp_pause; //....ms
		  			  plc->current_cs =  0; // start_pause

		  			  if(plc->temp_status&Сhange_temperature){
		  				  plc->can_send_temp.TDLR = (plc->temp2) << 16 | plc->temp1;// d0-d3
		  				  plc->temp_status |= SEND_temperature&CAN_transmit(&(plc->can_send_temp));
		  				  plc->temp_status &=~Сhange_temperature;
		  			  }

		  			  break;

		  		  default:
		  			  plc->current_cs = pin_CS1;
		  			  break;
		  		  };

		  	// b0 -send_temperature, b1-spi_read_ok ,b2 -change temperature.

		  	if(plc->current_cs){

		  			GPIOB->BRR = plc->current_cs;
		  			plc->temp_status &= ~SPI_read_OK;
				    while (!(SPI2->SR & SPI_SR_TXE)){}; // Ожидание готовности передатчика
				    SPI2->DR = 0x0000;

		    };

};



void CEC_CAN_IRQHandler(void){

	uint32_t id,dlc,gpio = 0;
	uint8_t data0;

		if((CAN->RF0R & 0b0011) == 0) goto exit;
		// ..td = тип сообщения ? extd : std
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
				 GPIOA->BSRR = gpio;
				 if(mask_gpio == 0)send_txPDO1 = 1;// 0-reply status pin
			     break;
			};
		};

exit:	SET_BIT(CAN->RF0R, CAN_RF0R_RFOM0);//CAN->RF0R |= 0b0100000;  сообщение прочитано.
};



// PB4 Data, PB3 SCK, PC13 - PL

#define data_165 0x01 << 4
#define pl_165   0x01 << 13
#define sck_165  0x01 << 3

uint32_t Read_addr_CAN(void){

	   uint8_t addr = 0;
	   uint8_t bit_addr =0;

	   GPIOB->BSRR = sck_165;
	   GPIOC->BSRR = pl_165;
	   GPIOC->BRR =  pl_165; //low
	   GPIOC->BSRR = pl_165; //high

	      // cycle 8 bit
	      for(uint8_t b=0; b<8 ; b++){

	    	  bit_addr = (GPIOB->IDR&data_165)?1:0;
	          bit_addr <<= b;
	          addr |=bit_addr;
	          GPIOB->BRR = sck_165;
	          GPIOB->BSRR = sck_165;

	      };
	   return addr;
};

void init_controller_STM32F072(struct PLC_controller* plc){

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

	// включаем буфер и частоту 48мгц

	FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

	NVIC_SetPriority(RCC_CRS_IRQn,0);
	NVIC_EnableIRQ(RCC_CRS_IRQn);

	RCC->CR |= RCC_CR_HSEON;  			// включаем генератор HSE
	while(!(RCC->CR & RCC_CR_HSERDY)){};// ожидание готовности HSE
    RCC->CR |= RCC_CR_CSSON; 			// следить за кварцем

	RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV1;  // Prediv предделитель для PLL
	RCC->CFGR |=(RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL6);//завести HSE на PLL и 8 Мгц x6 = 48 Мгц

	RCC->CR |= RCC_CR_PLLON;  			// включаем PLL
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
		Reset value: 0x2800 0000 for port A (swdio default enable)
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


	/*
	 OUTPUT GPIOA 0-7
	 INPUT  GPIOA 8-12,15
	 GPIOA 13-14 (swlck/swdio);
	 */

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;// включить порт GPIOA
	//---------------  	151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOA->MODER   |= 0b00000000000000000101010101010101; // General purpose output mode
	GPIOA->OSPEEDR |= 0b00000000000000001111111111111111; //0b11  high speed
	GPIOA->BRR 		= 0b0000000011111111;
    //GPIOA->OTYPER &=0b1111111100000000;//0 - push-pull, reset default

	/*
	 GPIOB out 0-1 Led, 3 - sck, cs1-2 11,12
	 GPIOB in  2 - 220v, 4 - data165
	 GPIOB alt spi miso 14, sck 13
	 GPIOB alt CAN 8,9
	 GPIOB alt uart1 tx1 6, rx1 7
	 5,10,15  none
	 */

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;// enable GPIOВ
	//---------------  	151413121110 9 8 7 6 5 4 3 2 1 0
    GPIOB->MODER |=   0b10101001010010100000000001000101; // General purpose in/out mode
    GPIOB->OSPEEDR |= 0b11111111110011111111000011000101;// 11 high, 01 medium speed
    GPIOB->PUPDR   |= 0b00000000000100000000010000000000;

    //data addr Can
    GPIOB->BSRR = sck_165 | 1 << CS1 | 1 << CS2; //high

    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	//---------------  	151413121110 9 8 7 6 5 4 3 2 1 0
    GPIOC->MODER |=   0b00000100000000000000000000000000; // General purpose in/out mode
    GPIOC->OSPEEDR |= 0b00001100000000000000000000000000;// 11 high, 01 medium speed
    GPIOC->BSRR = pl_165; //


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
    NVIC_EnableIRQ(SysTick_IRQn);

/******************************  TIM14 **************************/

    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	TIM14->PSC =  48000000/1000 - 1;
	TIM14->DIER |= TIM_DIER_UIE; 	  // enable perepolnenia TIM14
	TIM14->CR1 |= TIM_CR1_CEN;		  // ON TIM14
	TIM14->ARR = 2; //2 ms
	NVIC_EnableIRQ(TIM14_IRQn);


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
	//GPIOB->MODER |= (GPIO_MODER_MODER8_1  | GPIO_MODER_MODER9_1 );
	// скорость порта на максимум
	//GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR8|GPIO_OSPEEDR_OSPEEDR9;

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

	/*// reset default
	CLEAR_BIT(CAN->MCR, CAN_MCR_TTCM);// Clear 0 /Set 1 the time triggered communication mode
	CLEAR_BIT(CAN->MCR, CAN_MCR_ABOM);// Clear 0 /Set 1 the automatic bus-off management
	CLEAR_BIT(CAN->MCR, CAN_MCR_AWUM);// Clear 0 /Set 1 the automatic wake-up mode
	CLEAR_BIT(CAN->MCR, CAN_MCR_NART);// Set 1/ Clear 0 the automatic retransmission
	CLEAR_BIT(CAN->MCR, CAN_MCR_RFLM);// Set the receive FIFO locked mode
	CLEAR_BIT(CAN->MCR, CAN_MCR_TXFP);// Set the transmit FIFO priority
	*/

	// get can_id
	uint32_t speed;
	plc->can_id = Read_addr_CAN()&0x7F;
	plc->can_speed = 0x03;//can_id&0x80?1:0;


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


/*
  	Bit 4 FMPIE1: FIFO1 message pending interrupt enable
	Bit 1 FMPIE0: FIFO0 message pending interrupt enable
	    0: No interrupt generated when state of FMP[1:0] bits are not 00b.
	    1: Interrupt generated when state of FMP[1:0] bits are not 00b.
*/

    /* CAN interrupt Init */

	NVIC_SetPriority(CEC_CAN_IRQn,1);
	NVIC_EnableIRQ(CEC_CAN_IRQn);


    // Включение тактирования SPI2
	/*
	 AF0
	 SPI2
	  SPI control register 1 (SPIx_CR1)

	  	  Bit 15 BIDIMODE: Bidirectional data mode enable.
			This bit enables half-duplex communication using common single bidirectional data line.
			Keep RXONLY bit clear when bidirectional mode is active.
				0: 2-line unidirectional data mode selected
				1: 1-line bidirectional data mode selected

		Bit 14 BIDIOE: Output enable in bidirectional mode
		This bit combined with the BIDIMODE bit selects the direction of transfer in bidirectional
		mode.
			0: Output disabled (receive-only mode)
			1: Output enabled (transmit-only mode)
			Note: In master mode, the MOSI pin is used and in slave mode, the MISO pin is used.
		This bit is not used in I2S mode.
		Bit 13 CRCEN: Hardware CRC calculation enable
			0: CRC calculation disabled
			1: CRC calculation enabled
			Note: This bit should be written only when SPI is disabled (SPE = ‘0’) for correct operation.
		Bit 12 CRCNEXT: Transmit CRC next
				0: Next transmit value is from Tx buffer.
				1: Next transmit value is from Tx CRC register.
			Note: This bit has to be written as soon as the last data is written in the SPIx_DR register.

		Bit 11 CRCL: CRC length
			This bit is set and cleared by software to select the CRC length.
			0: 8-bit CRC length
			1: 16-bit CRC length
		Note: This bit should be written only when SPI is disabled (SPE = ‘0’) for correct operation.

		Bit 10 RXONLY: Receive only mode enabled.
		This bit enables simplex communication using a single unidirectional line to receive data
		exclusively. Keep BIDIMODE bit clear when receive only mode is active.This bit is also
			useful in a multislave system in which this particular slave is not accessed, the output from
				the accessed slave is not corrupted.
		0: Full-duplex (Transmit and receive)
		1: Output disabled (Receive-only mode)
			Note: This bit is not used in I2S mode.
		Bit 9 SSM: Software slave management
			When the SSM bit is set, the NSS pin input is replaced with the value from the SSI bit.
			0: Software slave management disabled
			1: Software slave management enabled

		Bit 8 SSI: Internal slave select
				This bit has an effect only when the SSM bit is set. The value of this bit is forced onto the
				NSS pin and the I/O value of the NSS pin is ignored.

		Bit 7 LSBFIRST: Frame format
			0: data is transmitted / received with the MSB first
			1: data is transmitted / received with the LSB first
			Note: 1. This bit should not be changed when communication is ongoing.

		Bit 6 SPE: SPI enable
				0: Peripheral disabled
				1: Peripheral enabled
			Note: When disabling the SPI, follow the procedure described in Procedure for disabling the
			SPI on page 779.

		Bits 5:3 BR[2:0]: Baud rate control
				000: fPCLK/2
				001: fPCLK/4
				010: fPCLK/8
				011: fPCLK/16
				100: fPCLK/32
				101: fPCLK/64
				110: fPCLK/128
				111: fPCLK/256
			Note: These bits should not be changed when communication is ongoing.

		Bit 2 MSTR: Master selection
			0: Slave configuration
			1: Master configuration
			Note: This bit should not be changed when communication is ongoing.

		Bit 1 CPOL: Clock polarity
				0: CK to 0 when idle
				1: CK to 1 when idle
		Note: This bit should not be changed when communication is ongoing.

		Bit 0 CPHA: Clock phase
			0: The first clock transition is the first data capture edge
			1: The second clock transition is the first data capture edge
			Note: This bit should not be changed when communication is ongoing.

	 Reset value: 0x0000
	 */
	//SPI2->CR1 &= ~SPI_CR1_SPE;// off SPI - reset default
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    // read only,CS - gpio,f/64,master
    SPI2->CR1 = SPI_CR1_SSM|SPI_CR1_SSI|(0b101 << SPI_CR1_BR_Pos)|SPI_CR1_MSTR;
    SPI2->CR2 = 0b1111 < SPI_CR2_DS_Pos;// set default
    SPI2->CR1 |= SPI_CR1_SPE;
    SPI2->CR2 |= SPI_CR2_RXNEIE;
    NVIC_EnableIRQ(SPI2_IRQn);


















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
