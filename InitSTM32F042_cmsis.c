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
        RCC->CR |= RCC_CR_CSSON; 	//следить за кварцем

	RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV1;  // Prediv предделитель для PLL
	//завести HSE на PLL и 8 Мгц x6 = 48 Мгц
	RCC->CFGR |=(RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL6);

	RCC->CR |= RCC_CR_PLLON;  //включаем PLL
	while(!(RCC->CR & RCC_CR_PLLRDY));  // ожидание готовности PLL
	RCC->CFGR |= RCC_CFGR_SW_PLL;  		// переводим тактирование от PLL

	//MODIFY_REG(RCC->CFGR,RCC_CFGR_HPRE,RCC_CFGR_HPRE_DIV1);//AHB Prescaler /1
	//MODIFY_REG(RCC->CFGR,RCC_CFGR_PPRE,RCC_CFGR_PPRE_DIV1);//APB1 Prescaler /1,

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // включить блок syscfg
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;// управление питанием



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

/******************************  bxCAN **************************/
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
		default: speed = 0x001c0005;break;

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
	CAN->sFilterRegister[1].FR1 = (id_rxSDO<<16 | heartbroken)<< 5;
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

