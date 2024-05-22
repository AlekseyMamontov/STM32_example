/* STM32L010f4 */

#include "main.h"
uint32_t  ms_pause = 500;
void cmsis_init_stm32L010(void);

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

	  /*

	  led_stat = led_stat == 0b00000000010000000000000000100000?0b00000000001000000000000001000000:0b00000000010000000000000000100000;
	  GPIOA->BSRR = led_stat;
	  pause_mss();

	   */






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


    // Установка тактовой частоты SysTick на HSI16/2 = 8 МГц

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
	GPIOA->MODER   &= 0b11111111110000110000000011111111;
	GPIOA->MODER   |= 0b00000000001010000001010100000000; // General purpose output mode

	GPIOA->OSPEEDR |= 0b00000000001111000011110000000000; //0b11  high speed
	GPIOA->BRR 		= 0b0000000001110000;
	GPIOA->AFR[1] |= 0b0000000100010000; //AF1 i2c1

	//GPIOA->BSRR = 0b000100000;

	// Включение тактирования I2C1 от HSI16/2
	RCC->CCIPR &= ~(RCC_CCIPR_I2C1SEL);
	RCC->CCIPR |= RCC_CCIPR_I2C1SEL_1;
	// Настройка тактирования I2C1 от HSI16/2
	//I2C1->TIMINGR = 0x10420309; // 400kHz при HSI16/2 = 8MHz
	I2C1->TIMINGR = 0x30421E1E; // 100kHz при HSI16/2 = 8MHz
	// Сброс и включение I2C1
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	I2C1->CR1 = 0;
	I2C1->CR1 |= I2C_CR1_PE; // Включение I2C1




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
