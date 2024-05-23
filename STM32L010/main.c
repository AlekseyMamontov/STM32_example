/* STM32L010f4 */

#include "main.h"
#include "core_cm0plus.h"
uint32_t  ms_pause = 500;


int main(void){


	cmsis_init_stm32L010();

	uint32_t led_stat = 0b00000000010000000000000000100000;

  while(1){



	  if(!ms_pause){

		led_stat = led_stat == 0b00000000010000000000000000100000?
							   0b00000000001000000000000001000000:
							   0b00000000010000000000000000100000;
	  	GPIOA->BSRR = led_stat;
		ms_pause =  500;

	  }








  };

}
void pause_mss(){for(uint32_t i=0;i<300000;i++){};};

void cmsis_init_stm32L010(void){


	FLASH->ACR |= FLASH_ACR_LATENCY;
	while ((FLASH->ACR & FLASH_ACR_LATENCY) == 0){};

		RCC->CR |= RCC_CR_HSION;
		while (!(RCC->CR & RCC_CR_HSIRDY)){}; // Ждем, пока HSI16 станет готов

		/* Выбираем HSI16 в качестве источника системного тактового сигнала */
		RCC->CFGR |= RCC_CFGR_SW_HSI;
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

		/* Настраиваем делители  HPRE[3:0]: AHB prescaler/2 = 8 МГц для HCLK, PCLK1 и PCLK2 */
		RCC->CFGR |= RCC_CFGR_HPRE_DIV2 ;

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // включить блок syscfg
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;	  // управление питанием


    // Установка тактовой частоты SysTick на HSI16/2 = 8 МГц. 1ms

    SysTick->LOAD  = (uint32_t)(8000U - 1UL);                         /* set reload register */
     /* set Priority for Systick Interrupt */
    SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
    NVIC_EnableIRQ(SysTick_IRQn);



	// Led PA6 - status / Led PA5 - error
	// i2c1 PA10 - sda / PA9 - scl
	// PA7 interrput - dry
	// PA4 mosfet - key output

	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;// включить порт GPIOA
	//---------------  	151413121110 9 8 7 6 5 4 3 2 1 0

	/*
	GPIO port mode register (GPIOx_MODER)
	(x =A to E and H)
	Address offset:0x00
	Reset value: 0xEBFF FCFF for port A // SWD -enabled
	Reset value: 0xFFFF FFFF for the other ports
	 * */


	GPIOA->MODER   &= 0b11111111110000110000000011111111;
	GPIOA->MODER   |= 0b00000000001010000001010100000000; // General purpose output mode

	GPIOA->OSPEEDR |= 0b00000000001111000011110000000000; //0b11  high speed
	GPIOA->BRR 		= 0b0000000001110000;
	GPIOA->AFR[1] |= 0b0000000100010000; //AF1 i2c1

	//GPIOA->BSRR = 0b000100000;



	/* Включение тактирования I2C1 от SySclk
		Clock configuration register (RCC_CCIPR)
		Address: 0x4C
		Reset value: 0x0000 0000

	Bits 13:12 I2C1SEL: I2C1 clock source selection bits
	This bit is set and cleared by software.
		00: APB clock selected as I2C1 clock
		01: System clock selected as I2C1 clock
		10: HSI16 clock selected as I2C1 clock
1		1: not used

	*/

	//RCC->CCIPR |= RCC_CCIPR_I2C1SEL_0; //System clock


	/*
		I2C timing register (I2C_TIMINGR)
		Address offset: 0x10
		Reset value: 0x0000 0000

		Bits 31:28 PRESC[3:0]: Timing prescaler
			This field is used to prescale I2CCLK in order to generate the clock period tPRESC used for
			data setup and hold counters (refer to I2C timings on page 511) and for SCL high and low
			level counters (refer to I2C master initialization on page 526).
			tPRESC = (PRESC+1) x tI2CCLK
		Bits 27:24 Reserved, must be kept at reset value.
		Bits 23:20 SCLDEL[3:0]: Data setup time
			This field is used to generate a delay tSCLDEL between SDA edge and SCL rising edge. In
			master mode and in slave mode with NOSTRETCH = 0, the SCL line is stretched low during tSCLDEL.
			tSCLDEL = (SCLDEL+1) x tPRESC
				Note: tSCLDEL is used to generate tSU:DAT timing.
		Bits 19:16 SDADEL[3:0]: Data hold time
			This field is used to generate the delay tSDADEL between SCL falling edge and SDA edge. In
			master mode and in slave mode with NOSTRETCH = 0, the SCL line is stretched low during tSDADEL.
			tSDADEL= SDADEL x tPRESC
			Note: SDADEL is used to generate tHD:DAT timing.
		Bits 15:8 SCLH[7:0]: SCL high period (master mode)
			This field is used to generate the SCL high period in master mode.tSCLH = (SCLH+1) x tPRESC
				Note: SCLH is also used to generate tSU:STO and tHD:STA timing.
		Bits 7:0 SCLL[7:0]: SCL low period (master mode)
			This field is used to generate the SCL low period in master mode.tSCLL = (SCLL+1) x tPRESC
				Note: SCLL is also used to generate tBUF and tSU:STA timings.
				Note:This register must be configured when the I2C is disabled (PE = 0).
				Note:The STM32CubeMX tool calculates and provides the I2C_TIMINGR content in the I2C
				Configuration window.

		  <<28     <<20    <<16   <<8   <<0
		PRESC | SCLDEL | SDADEL| SCLH| SCLL|
		 1       0x04     0x02   0x0F  0x13   // 8 мгц 100кгц
		 0		 0x03     0x01   0x03  0x09   // 8 мгц 400кгц
		 3		 0x04	  0x02   0x0F  0x13   // 16mhz 100khz
		 1       0x03     0x02   0x03  0x09   // 16mhz 400khz


	*/

	I2C1->TIMINGR = 0x10420F13; //
	// Сброс и включение I2C1
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;; // Включение I2C1
	I2C1->CR2 = I2C_CR2_AUTOEND | (1<<16) | I2C_CR2_RD_WRN;



};


//I2C master transmitter code example
void send_i2c (uint8_t data){
/* Check Tx empty */
		if((I2C1->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE)){

			I2C1->TXDR = data; /* Byte to send */
			I2C1->CR2 |= I2C_CR2_START; /* Go */

		}
};


//I2C master receiver code example
uint8_t read_i2c (uint8_t data){
		if((I2C1->ISR & I2C_ISR_RXNE) == I2C_ISR_RXNE){
				/* Read receive register, will clear RXNE flag */
			if(I2C1->RXDR == data){/* Process */};
		}
return 	data;
};








void SysTick_Handler(void){
	 if(ms_pause)ms_pause --;
};


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
