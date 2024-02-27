
#include "main.h"
#include "CANOpen.h"
//#include "data_rgb.h"
//#include "dimmer_color.h"
#define n_Sync_Object 1

#define Dimmer_1 TIM3->CCR4
#define Dimmer_2 TIM3->CCR3
#define Dimmer_3 TIM3->CCR2
#define Dimmer_4 TIM3->CCR1

#define Dimmer_5 TIM2->CCR4
#define Dimmer_6 TIM2->CCR3
#define Dimmer_7 TIM2->CCR2
#define Dimmer_8 TIM2->CCR1


#define find_Dimmer_1 0x01
#define find_Dimmer_2 0x02
#define find_Dimmer_3 0x04
#define find_Dimmer_4 0x08
#define find_Dimmer_5 0x10
#define find_Dimmer_6 0x20
#define find_Dimmer_7 0x40
#define find_Dimmer_8 0x80

#define Led_Status 4
#define Led_Error  2


void init_controller_STM32F042(void);
uint8_t CAN_transmit (CAN_TxMailBox_TypeDef *tx);
//void Processing_SDO_Object(CAN_FIFOMailBox_TypeDef*);
void SDO_object(void);
#define ADDR_SAVE_FLASH 0x08003C00 //042C4x4
void Load_default_dimmers(void);
uint16_t test_time = 0,blink1 =0;

// CanOpen
CAN_FIFOMailBox_TypeDef rx_mailbox;
CAN_TxMailBox_TypeDef 	tx_mailbox;

uint32_t
can_id,can_speed,
id_rxPDO1,id_rxPDO2,id_txPDO1,
id_rxSDO,id_txSDO,heartbroken;


uint8_t  NMT_command = 0,NMT_status = NMT_status_Operational;

uint32_t *sync_data,*Sync_obj[n_Sync_Object]={NULL};
uint8_t  mask_gpio = 0xFF, send_txPDO1 = 0,send_txSDO = 0,
		 reply_rxPDO1 = 0,reply_rxPDO1_mask = 0;
uint32_t id  = 0;



int main(void)
{

	init_controller_STM32F042();
	Load_default_dimmers();

	   tx_mailbox.TIR = (id_txPDO1<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
	   tx_mailbox.TDTR = 8;// n на_отправку
	   tx_mailbox.TDLR = 0;// d0-d3
	   tx_mailbox.TDHR = 0;// d4-d7

	   CAN_transmit(&tx_mailbox);


 uint32_t msg_dimmer;

  while (1){


	  if(send_txPDO1){

		   tx_mailbox.TIR = ((0x700 + can_id)<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
		   tx_mailbox.TDTR = 8;// n на_отправку

		   msg_dimmer  = Dimmer_4 & 0xff;
		   msg_dimmer <<= 8;
		   msg_dimmer |= Dimmer_3 & 0xff;
		   msg_dimmer <<= 8;
		   msg_dimmer |= Dimmer_2 & 0xff;
		   msg_dimmer <<= 8;
		   msg_dimmer |= Dimmer_1 & 0xff;
		   tx_mailbox.TDLR =  msg_dimmer;// d0-d3

		   msg_dimmer  = Dimmer_5 & 0xff;
		   msg_dimmer <<= 8;
		   msg_dimmer |= Dimmer_6 & 0xff;
		   msg_dimmer <<= 8;
		   msg_dimmer |= Dimmer_7 & 0xff;
		   msg_dimmer <<= 8;
		   msg_dimmer |= Dimmer_8 & 0xff;
		   tx_mailbox.TDHR = msg_dimmer;// d4-d7

		   CAN_transmit(&tx_mailbox);
		   send_txPDO1 --;

	  };

	  SDO_object();

  };


}

void TIM14_IRQHandler(){

	if(test_time) test_time--;
/*
	if(R_pwm.period_ms)R_pwm.period_ms --;
	if(G_pwm.period_ms)G_pwm.period_ms --;
	if(B_pwm.period_ms)B_pwm.period_ms --;
*/
	TIM14->SR &= ~TIM_SR_UIF;

};

void CEC_CAN_IRQHandler(void){

	uint32_t id,dlc;
	 uint8_t data0,data1;

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

						if(id == id_rxPDO1){;

							if(NMT_status != NMT_status_Operational) break;
							if(dlc < 1) break;
							if( data0 == 0 ){send_txPDO1++;break;}
							if(dlc < 2) break;

							data1 = (CAN->sFIFOMailBox[0].RDLR&0xff00) >>8;

							if(data0&find_Dimmer_1) Dimmer_1 = data1;
							if(data0&find_Dimmer_2) Dimmer_2 = data1;
							if(data0&find_Dimmer_3) Dimmer_3 = data1;
							if(data0&find_Dimmer_4) Dimmer_4 = data1;
							if(data0&find_Dimmer_5) Dimmer_5 = data1;
							if(data0&find_Dimmer_6) Dimmer_6 = data1;
							if(data0&find_Dimmer_7) Dimmer_7 = data1;
							if(data0&find_Dimmer_8) Dimmer_8 = data1;

							break;

						}else if(id == id_rxPDO2){

						if(NMT_status != NMT_status_Operational) break;
						if(dlc < 1) break;
						Dimmer_1 = CAN->sFIFOMailBox[0].RDLR&0x000000ff;
						if(dlc < 2) break;
						Dimmer_2 = CAN->sFIFOMailBox[0].RDLR&0x0000ff00 >> 8;
						if(dlc < 3) break;
						Dimmer_3 = CAN->sFIFOMailBox[0].RDLR&0x00ff0000 >>16;
						if(dlc < 4) break;
						Dimmer_4 = CAN->sFIFOMailBox[0].RDLR&0xff000000 >>24;
						if(dlc < 5) break;
						Dimmer_5 = CAN->sFIFOMailBox[0].RDHR&0x000000ff;
						if(dlc < 6) break;
						Dimmer_6 = CAN->sFIFOMailBox[0].RDHR&0x0000ff00 >> 8;
						if(dlc < 7) break;
						Dimmer_7 = CAN->sFIFOMailBox[0].RDHR&0x00ff0000 >>16;
						if(dlc < 8) break;
						Dimmer_8 = CAN->sFIFOMailBox[0].RDHR&0xff000000 >>24;

						};

				    break;
				};
			};
	exit:	SET_BIT(CAN->RF0R, CAN_RF0R_RFOM0);//CAN->RF0R |= 0b0100000;  сообщение прочитано.
};



void Load_default_dimmers(){

	volatile  uint8_t*  addr8  = (volatile uint8_t*) ADDR_SAVE_FLASH+2;
	volatile  uint16_t* addr16 = (volatile uint16_t*)ADDR_SAVE_FLASH;

				if(*addr16 == 0xFFFF){

					Dimmer_1 = 0;
					Dimmer_2 = 0;
					Dimmer_3 = 0;
					Dimmer_4 = 0;
					Dimmer_5 = 0;
					Dimmer_6 = 0;
					Dimmer_7 = 0;
					Dimmer_8 = 0;

				}else{

					Dimmer_1 = *addr8++;
					Dimmer_2 = *addr8++;
					Dimmer_3 = *addr8++;
					Dimmer_4 = *addr8++;
					Dimmer_5 = *addr8++;
					Dimmer_6 = *addr8++;
					Dimmer_7 = *addr8++;
					Dimmer_8 = *addr8;
				};
};

void SDO_object(){

	uint32_t id,dlc,cmd,index,subindex,data;
	volatile  uint8_t*  addr8;
	volatile  uint16_t* addr16;
	volatile  uint32_t* addr32;
	uint16_t counter;
    uint32_t error = 0;

	//no message?
	if((CAN->RF1R & 0b0011) == 0) return;

	// индефикатор = тип сообщения ? extd : std
	id = CAN->sFIFOMailBox[1].RIR&0b0100?CAN->sFIFOMailBox[1].RIR >> 3:CAN->sFIFOMailBox[1].RIR >> 21;
	dlc = CAN->sFIFOMailBox[1].RDTR &0b01111;// получение из регистра длины кадра
	if((CAN->sFIFOMailBox[1].RIR & 0b010) != 0 && dlc < 4)goto sdo_exit;// тип сообщения 0 - data / 1- rtr
	if(id !=id_rxSDO ) goto sdo_exit;


		cmd = 		CAN->sFIFOMailBox[1].RDLR&0xFF;
		index =    (CAN->sFIFOMailBox[1].RDLR&0x00FFFF00) >> 8;
		subindex = (CAN->sFIFOMailBox[1].RDLR&0xFF000000) >> 24;
		data = 		CAN->sFIFOMailBox[1].RDHR;

		error = 0x06010000;

		switch(index){

			case 0x1010: // save to flash default parametr

				if(dlc < 8)break;
				if(cmd !=0x23) break;
				if(subindex != 1)break;
				if(data != 0x65766173)break;
				addr16 = (volatile uint16_t*)ADDR_SAVE_FLASH;
				counter = *addr16;
				if(counter == 0xFFFF)counter = 0;
				counter++;
					// key
					FLASH->KEYR = 0x45670123;
					FLASH->KEYR = 0xCDEF89AB;
					//clear
					while (FLASH->SR & FLASH_SR_BSY){};
					if (FLASH->SR & FLASH_SR_EOP) FLASH->SR = FLASH_SR_EOP;
					FLASH->CR |= FLASH_CR_PER;
					FLASH->AR =  ADDR_SAVE_FLASH; //addr segment
					FLASH->CR |= FLASH_CR_STRT;
					while (!(FLASH->SR & FLASH_SR_EOP));//check operation end
					FLASH->SR = FLASH_SR_EOP;
					FLASH->CR &= ~FLASH_CR_PER;
					//save
					while (FLASH->SR & FLASH_SR_BSY){};
					if (FLASH->SR & FLASH_SR_EOP) FLASH->SR = FLASH_SR_EOP;
					FLASH->CR |= FLASH_CR_PG;

					*addr16++ = counter;
					while (!(FLASH->SR & FLASH_SR_EOP)){};
					FLASH->SR = FLASH_SR_EOP;

					counter = Dimmer_2&0xFF;
					counter <<=8;
					counter |=Dimmer_1&0xff;
					*addr16++ = counter;
					while (!(FLASH->SR & FLASH_SR_EOP)){};
					FLASH->SR = FLASH_SR_EOP;

					counter = Dimmer_4&0xFF;
					counter <<=8;
					counter |=Dimmer_3&0xff;
					*addr16++ = counter;
					while (!(FLASH->SR & FLASH_SR_EOP)){};
					FLASH->SR = FLASH_SR_EOP;

					counter = Dimmer_6&0xFF;
					counter <<=8;
					counter |=Dimmer_5&0xff;
					*addr16++ = counter;
					while (!(FLASH->SR & FLASH_SR_EOP)){};
					FLASH->SR = FLASH_SR_EOP;

					counter = Dimmer_8&0xFF;
					counter <<=8;
					counter |=Dimmer_7&0xff;
					*addr16 = counter;
					while (!(FLASH->SR & FLASH_SR_EOP)){};
					FLASH->SR = FLASH_SR_EOP;
					FLASH->CR &= ~(FLASH_CR_PG);

					FLASH->CR |= FLASH_CR_LOCK;

					cmd = 0x60;
					dlc = 4;
					error = 0;
					data = 0;
			break;

		case 0x1011: // load default

			if(dlc < 8)break;
			if(cmd !=0x23) break;
			if(subindex != 1)break;
			if(data != 0x64616f6c)break;

			Load_default_dimmers();

			cmd = 0x60;
			dlc = 4;
			error = 0;
			data = 0;
			break;
		default:
			break;
	};

	   if(error){cmd = 0x80;dlc = 8;data = error;}

	   tx_mailbox.TIR = (id_txSDO<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
	   tx_mailbox.TDTR = dlc;// n на_отправку
	   tx_mailbox.TDLR = (subindex<<24)|index<<8|cmd;// d0-d3
	   tx_mailbox.TDHR = data;// d4-d7
	   CAN_transmit(&tx_mailbox);

sdo_exit:SET_BIT(CAN->RF1R, CAN_RF1R_RFOM1);
};


void TIM3_IRQHandler(void)
{

    TIM3->SR &= ~TIM_SR_UIF; // Сброс флага прерывания от обновления таймера TIM3
}
void TIM2_IRQHandler(void)
{

    TIM2->SR &= ~TIM_SR_UIF; // Сброс флага прерывания от обновления таймера TIM3
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


	SystemCoreClockUpdate();
	SysTick_Config (SystemCoreClock / 1000);

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

     /* GPIOA
	 0,1,2,3 TIM2 ch1-ch4 PWM output
	 6,7 - TIM3 ch1,ch2
	 4 - LED status
     5 - none
	 13-14 (swlck/swdio);
	 8,9,10 - speed addr CAN
	 11,12 CAN
	 */

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;// включить порт GPIOA
	//---------------  151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOA->MODER  |= 0b00000010100000001010000110101010; // General purpose output mode
	GPIOA->OSPEEDR|= 0b00000000000000001111000111111111; //0b11  high speed
	GPIOA->PUPDR  |= 0b00000000001010100000100000000000; //8,9,10 addrCan,5 none
	GPIOA->OTYPER |= 0b0000000000000000;// open drain led_status

	/*
	 GPIOB
	 0,1 - TIM3 ch3,ch4
	 2 - led -error
	 3,4,5,6,7,8,9 - adrrCAN
	 --
	 12,13,14,15 - spi
	 10,11 - i2c
	 */

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;// enable GPIOВ
	//---------------  	151413121110 9 8 7 6 5 4 3 2 1 0
    GPIOB->MODER   |= 0b00000000000000000000000000011010;// General purpose in/out mode
    GPIOB->OSPEEDR |= 0b00000000000000000000000000011111;// 11 high, 01 medium speed
    GPIOB->PUPDR   |= 0b00000000000010101010101010000000;
    GPIOA->OTYPER  |= 0b0000000000000000;// open drain led_status


	// reset.

	GPIOA->BRR = 1 << Led_Status;
	GPIOB->BRR = 1 << Led_Error; // On

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

	/*	   RCC->AHBENR |= RCC_APB1ENR_TIM2EN;
		   TIM2->PSC = 120 - 1;
		   TIM2->ARR = 200-1;

		   GPIOA->MODER |=  GPIO_MODER_MODER2_1;//Alternate function mode
		   GPIOA->AFR[0] |= 0x200;  // 3 pin AF2 tim2_ch3

		   TIM2->CR1 |= 0x80;// Preload_en
		   TIM2->CCMR2 |= 0b110 << 4;//TIM2_ch3_PWM_1
		   TIM2->CCER |= TIM_CCER_CC3E|TIM_CCER_CC3P;
		   TIM2->CCR3 = 00; //50%
		   TIM2->CR1 |= TIM_CR1_CEN;// on timer2
	*/
			// TIM2
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

			TIM2->PSC = 120 - 1; // 2 kГц
			TIM2->ARR = 200-1;

			GPIOA->AFR[0] |= 0x2222;//AF#2 // TIM_CH1-TIM_CH4 pin A0-A3

			TIM2->CR1 |= 0x80; // Preload_en
			TIM2->CCMR1 |= 0b110 << 4 | 0b110 << 12;//
			TIM2->CCMR2 |= 0b110 << 4 | 0b110 << 12;//Bits 6:4 OC1M PWM 1
			TIM2->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E;
		  //TIM2->CCER |= TIM_CCER_CC1P|TIM_CCER_CC2P|TIM_CCER_CC3P|TIM_CCER_CC4P;// output ~low _
			TIM2->CCR1 = 00;
			TIM2->CCR2 = 00;
			TIM2->CCR3 = 00;
			TIM2->CCR4 = 00;
		  // Включение таймера 3
			TIM2->CR1 |= TIM_CR1_CEN;// on timer2

			//NVIC_EnableIRQ(TIM2_IRQn);

			// TIM3

		    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

		    TIM3->PSC = 120 - 1;// 2 kГц
		    TIM3->ARR = 200-1;

		    GPIOA->AFR[0] |= 0x11000000; // TIM3_CH1, TIM3_CH2 PA6,PA7
		    GPIOB->AFR[0] |= 0b00010001; // TIM3_CH3, TIM3_CH4 PB0,PB1


		    TIM3->CR1 |= 0x80;
		    TIM3->CCMR1 |= 0b110 << 4 | 0b110 << 12;//
		    TIM3->CCMR2 |= 0b110 << 4 | 0b110 << 12;//Bits 6:4 OC1M PWM 1
		    TIM3->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E;
		    // TIM3->CCER |= TIM_CCER_CC3P|TIM_CCER_CC4P; output ~low _TIM_CCER_CC1P|TIM_CCER_CC2P|TIM_CCER_CC3P|TIM_CCER_CC4P;
		    TIM3->CCR1 = 00; //50%
		    TIM3->CCR2 = 00; //75%
		    TIM3->CCR3 = 00; //50%
		    TIM3->CCR4 = 00; //75%
		    // Включение таймера 3
		    TIM3->CR1 |= TIM_CR1_CEN;// on timer2

		    //NVIC_EnableIRQ(TIM3_IRQn);



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
	can_speed =0;
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
	id_rxPDO2 = rxPDO2 + can_id;
	id_txPDO1 = txPDO1 + can_id;
	id_rxSDO =  rxSDO + can_id;
	id_txSDO =  txSDO + can_id;
	heartbroken = 0x700 + can_id;


	CAN->FMR |=  CAN_FMR_FINIT; /*  init mode */
	CAN->FA1R |= (CAN_FA1R_FACT0|CAN_FA1R_FACT1);//Activate filter 0,1
	CAN->FM1R |= (CAN_FM1R_FBM0 |CAN_FM1R_FBM1); //ID_list
	CAN->FS1R  &=~(CAN_FS1R_FSC0|CAN_FS1R_FSC1);  //16bit
	CAN->FFA1R &=~CAN_FFA1R_FFA0; // filtr0 -> FIFO0
	CAN->FFA1R |= CAN_FFA1R_FFA1; // filtr1 -> FIFO1

	CAN->sFilterRegister[0].FR1 = (id_rxPDO1<< 16 | id_rxPDO2 ) << 5;
	CAN->sFilterRegister[0].FR2 = 0;
	CAN->sFilterRegister[1].FR1 = (id_rxSDO << 16 | heartbroken)<< 5;//0x02 test rtr bit for heartbroken
	CAN->sFilterRegister[1].FR2 = (0x80 << 16 | 0x100)<< 5;

	CAN->FMR &=~ CAN_FMR_FINIT; /* Leave filter init */
	CAN->IER |= CAN_IER_FMPIE0; /* Set FIFO0 message pending IT enable */


	/*выйти в нормальный режим*/

	CAN->MCR &=~ CAN_MCR_INRQ;
	while((CAN->MSR & CAN_MSR_INAK)==CAN_MSR_INAK){}

	GPIOB->BSRR = 1 << Led_Error; //off_error
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




void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  GPIOB->BRR = 1 << Led_Error;

  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}




/*

	 for(uint8_t i = 0; i< 201;i++){
		 test_time = 20;
		 while(test_time){};
		 Color_R = i;
		 Color_G1 = i;
		// canal2 = i;
	 };
	 for(uint8_t i = 0; i< 201;i++){
	 		 test_time = 20;
	 		 while(test_time){};
	 		 Color_G = i;
	 		Color_R1 = i;
	 		Color_White = i;
	 };

	 for(uint8_t i = 0; i< 201;i++){
	 		 test_time = 20;
	 		 while(test_time){};
	 		 Color_B = i;
	 		Color_B1 = i;

	 };


	 r = 200;

	 for(uint8_t i = 0; i< 201;i++){
	 		 test_time = 20;
	 		 while(test_time){};
	 		 Color_R = r;
	 		//canal2 = r;
	 		Color_G1 = r--;
	 	 };

	 g = 200;

		 for(uint8_t i = 0; i< 201;i++){
		 		 test_time = 20;
		 		 while(test_time){};
		 		 Color_G = g;
		 		Color_B1 = g;
		 		Color_White = g--;
		 	 };
	  b = 200;

	  	  for(uint8_t i = 0; i< 201;i++){
	  		 		 test_time = 20;
	  		 		 while(test_time){};
	  		 		 Color_B = b;
	  		 		Color_R1 = b--;

	  	  };


	  	 test_time = 3000;
	  	 while(test_time){};


  }





 */
