/*
 * GPIO.h
 *
 *  Created on: Feb 1, 2025
 *      Author: oleksii
 */

#ifndef INC_INIT_STM32G473_GPIO_H_
#define INC_INIT_STM32G473_GPIO_H_


void SystemClock_Config(void) {
	/*
    // Включение HSI
    RCC->CR |= RCC_CR_HSION; // Включение HSI
    while (!(RCC->CR & RCC_CR_HSIRDY)); // Ожидание готовности HSI

    // Настройка PLL для генерации тактовой частоты 150 МГц
    RCC->PLLCFGR = 0; // Сброс конфигурации PLL
    RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_HSI | // Использование HSI как источника
                     (8 << RCC_PLLCFGR_PLLM_Pos) | // PLLM = 8 (16 МГц / 8 = 2 МГц)
                     (150 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 150 (2 МГц * 150 = 300 МГц)
                     (2 << RCC_PLLCFGR_PLLR_Pos) | // PLLR = 2 (300 МГц / 2 = 150 МГц)
                     RCC_PLLCFGR_PLLREN); // Включение PLLR
    */


    // Включение HSE 8mhz
    RCC->CR |= RCC_CR_HSEON; // Включение HSE*
    while (!(RCC->CR & RCC_CR_HSERDY)); // Ожидание готовности HSE

    // Настройка флеш-памяти для работы на высокой частоте
    FLASH->ACR |= FLASH_ACR_LATENCY_4WS; // Установка латентности Flash (4 такта для 150 МГц)



    // Настройка PLL для генерации тактовой частоты 150 МГц
    RCC->PLLCFGR = 0; // Сброс конфигурации PLL
    RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_HSE | 	  // Использование HSE как источника
                    (1 << RCC_PLLCFGR_PLLM_Pos) | // PLLM = 2 (8 МГц / 2 = 4 МГц)
                    (80 << RCC_PLLCFGR_PLLN_Pos)| // PLLN = 80 * 4 МГц *  320 МГц)
                    (0 << RCC_PLLCFGR_PLLR_Pos) | // PLLR = 2 (320 МГц / 2 = 160 МГц)
					(1 << RCC_PLLCFGR_PLLQ_Pos) | // PLLQ = 4 (320 МГц / 4 = 80 МГц) for FDCAN
                    RCC_PLLCFGR_PLLREN |
					RCC_PLLCFGR_PLLQEN); 		  // Включение PLLR


    // Включение PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Ожидание готовности PLL

    // Настройка AHB, APB1 и APB2
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // AHB = SYSCLK
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV1; // APB1 = HCLK
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 = HCLK

    // Установка SYSCLK на PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Ожидание установки PLL как SYSCLK


    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_CRSEN);
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN);







    // Обновление SystemCoreClock
    SystemCoreClockUpdate();

    SysTick->LOAD = 160000000/1000 - 1;  // (48 mHz / 1000) -1  // 1ms
    SysTick->VAL = 0;  // reset
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    NVIC_EnableIRQ(SysTick_IRQn);

}

void GPIO_INIT(void){


	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN|RCC_AHB2ENR_GPIOCEN;



	/*
		GPIOA

		  USART2 TX PA2
		  USART2 RX PA3

		  LED_STAT  PA5

		  USB DP PA12
		  USB DM PA11

		  LED_CAN3  PA15


	  GPIOB

	      FDCAN3 PB3 RX
	      FDCAN3 PB4 TX
	      FDCAN2 PB5 RX
	      FDCAN2 PB6 TX

	      LED CAN2  PB7

	      FDCAN1 PB8 RX
	      FDCAN1 PB9 TX

	      USART3 TX PB10
	      USART3 RX PB11


      GPIOC

		  PC13 LED CAN1


	 */

	/*
	GPIO port mode register (GPIOx_MODER)
		(x =A to G)
			Address offset:0x00
				Reset value: 0xABFF FFFF (for port A) 1010 1011 1111 1111 1111 1111 1111 1111
				Reset value: 0xFFFF FEBF (for port B) 1111 1111 1111 1111 1111 1110 1011 1111
				Reset value: 0xFFFF FFFF (for ports C..G)

				Bits 31:0 MODE[15:0][1:0]: Port x configuration I/O pin y (y = 15 to 0)
				These bits are written by software to configure the I/O mode.
					00: Input mode
					01: General purpose output mode
					10: Alternate function mode
					11: Analog mode (reset state)

    */



	//   ------------  151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOA->MODER = 	 0b01101011111111111111011110101111; // General purpose output moder
	//-------------    151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOB->MODER =   0b11111111101010100110101010111111; // General purpose output mode
	GPIOC->MODER =   0b11110111111111111111111111111111;

	/*

	GPIO port output speed register (GPIOx_OSPEEDR)
		(x = A to G)
		Address offset: 0x08
			Reset value: 0x0C00 0000 (for port A)
			Reset value: 0x0000 0000 (for the other ports)

		Bits 31:0 OSPEED[15:0][1:0]: Port x configuration I/O pin y (y = 15 to 0)
		These bits are written by software to configure the I/O output speed.
			00: Low speed
			01: Medium speed
			10: High speed
			11: Very high speed
    */
	//-------------    151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOA->OSPEEDR = 0b10111100000000000000100011110000;

	//-------------    151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOB->OSPEEDR = 0b00000000111111111011111111000000;



	/*GPIO port output type register (GPIOx_OTYPER)
		(x = A to G)
				Address offset: 0x04
				Reset value: 0x0000 0000
			Bits 15:0 OT[15:0]: Port x configuration I/O pin y (y = 15 to 0)
			These bits are written by software to configure the I/O output type.
				0: Output push-pull (reset state)
				1: Output open-drain
   */

	//-------------   151413121110 9 8 7 6 5 4 3 2 1 0
	//GPIOA->OTYPER |= 0b00000000000000000000000000000000;
	//GPIOB->OTYPER |= 0b00000000000000000000000000000000;

	/*
	GPIO port pull-up/pull-down register (GPIOx_PUPDR)
			(x = A to G)
			Address offset: 0x0C
			Reset value: 0x6400 0000 (for port A)
			Reset value: 0x0000 0100 (for port B)
			Reset value: 0x0000 0000 (for other ports)

				These bits are written by software to configure the I/O pull-up or pull-down
				00: No pull-up, pull-down
				01: Pull-up
				10: Pull-down
				11: Reserved
  */
	     //-------------  151413121110 9 8 7 6 5 4 3 2 1 0
	//	 GPIOA->PUPDR = 0b00100100000000000000000000000000; // General purpose output mode
	     //-------------  151413121110 9 8 7 6 5 4 3 2 1 0
	//	 GPIOB->PUPDR = 0b00000000000000000000000000000000; // General purpose output mode

	/*

	GPIO alternate function low register (GPIOx_AFRL) (x = A to G) Address offset: 0x20 Reset value: 0x0000 0000
	GPIO alternate function high register (GPIOx_AFRH)(x = A to G) Address offset: 0x24 Reset value: 0x0000 0000

				0000: AF0
				0001: AF1
				0010: AF2
				0011: AF3
				0100: AF4
				0101: AF5
				0110: AF6
				0111: AF7
				1000: AF8
				1001: AF9
				1010: AF10
				1011: AF11
				..
				1111: AF15
	*/


			/*
				GPIOA

				  USART2 TX PA2  AF7
				  USART2 RX PA3  AF7

				  LED_STAT  PA5  -> AF1->TIM2_CH1

				  USB DP PA12
				  USB DM PA11

				  LED CAN3  PA15 -> AF2 -> TIM8_CH1

			*/

			//-------	     7   6   5   4   3   2   1   0
		GPIOA->AFR[0] = 0b00000000000000000111011100000000;
		   //-------	    15  14  13  12  11  10   9   8
	    GPIOA->AFR[1] = 0b00000000000000000000000000000000;

/*
		  GPIOB

		      FDCAN3 PB3 RX AF11
		      FDCAN3 PB4 TX AF11
		      FDCAN2 PB5 RX AF9
		      FDCAN2 PB6 TX AF9

		      LED CAN2  PB7    ->AF1 ->TIM17_CH1N

		      FDCAN1 PB8 RX  AF9
		      FDCAN1 PB9 TX  AF9

		      USART3 TX PB10 AF7
		      USART3 RX PB11 AF7


	      GPIOC

			  PC13 LED CAN1


		 */

	       //-------	     7   6   5   4   3   2   1   0
	    GPIOB->AFR[0] = 0b00001001100110111011000000000000;
	    	//-------	    15  14  13  12  11  10   9   8
	    GPIOB->AFR[1] = 0b00000000000000000111011110011001;





	/*
	GPIO port input data register (GPIOx_IDR)  Address offset: 0x10 Reset value: 0x0000 XXXX
	GPIO port output data register (GPIOx_ODR) Address offset: 0x14 Reset value: 0x0000 0000

	GPIO port bit set/reset register (GPIOx_BSRR) (x = A to G) Address offset: 0x18

		Bits 31:16 BR[15:0]: Port x reset I/O pin y (y = 15 to 0)
			These bits are write-only. A read to these bits returns the value 0x0000.
				0: No action on the corresponding ODx bit
				1: Resets the corresponding ODx bit
				Note: If both BSx and BRx are set, BSx has priority.
		Bits 15:0 BS[15:0]: Port x set I/O pin y (y = 15 to 0)
			These bits are write-only. A read to these bits returns the value 0x0000.
				0: No action on the corresponding ODx bit
				1: Sets the corresponding ODx bit
	  */





  /* EXTI

 IN0 PA2
 IN1 PA3
 IN2 PA4

  SYSCFG->EXTICR[0] |= 0b0000000000000000;
  SYSCFG->EXTICR[3] |= 0b0000000000000000;
  SYSCFG->EXTICR[2] |= 0b0000000000000000;

  EXTI->IMR1  |= 0b0000000000000000;// On interput EXTI1
  EXTI->EMR1  |= 0b0000000000000000;// Pending register 1 (EXTI_PR1)
  EXTI->FTSR1 |= 0b0000000000000000;// \_
  EXTI->RTSR1 |= 0b0000000000000000;// _/

  //NVIC_EnableIRQ(EXTI15_10_IRQn);
  //NVIC_EnableIRQ(EXTI1_IRQn);
  //NVIC_SetPriority(EXTI15_10_IRQHandler, 0);

  */
}

uint16_t systick_pause = 0;

void SysTick_Handler(void) {

	if (systick_pause) systick_pause--;

}





/*
void delay_ms(uint32_t ms){
    // Настраиваем SysTick для генерации задержки
    SysTick->LOAD = (SystemCoreClock / 1000) - 1; // Загрузка значения таймера для 1 мс
    SysTick->VAL = 0; // Обнуляем текущее значение
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;




    for (uint32_t i = 0; i < ms; i++) {
        // Ждем, пока счетчик не дойдет до нуля
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    }

    // Отключаем SysTick
    SysTick->CTRL = 0;
}
*/
#endif /* INC_INIT_STM32G473_GPIO_H_ */
