#include "stm32g4xx.h"
#include "main.h"
#include "CanFD_stm32G4.h"



int main(void) {
    // Настройка системного тактирования

    SystemClock_Config();
    GPIO_INIT();
    CAN_Config();


    while (1) {
        // Включаем светодиод
        GPIOA->BSRR = GPIO_BSRR_BS12; // Включаем светодиод
        delay_ms(500); // Задержка 500 мс

        // Выключаем светодиод
        GPIOA->BSRR = GPIO_BSRR_BR12; // Выключаем светодиод
        delay_ms(500); // Задержка 500 мс
    }

};


////////////////////////////////////////////////////////////////////////////////////////////////

void GPIO_INIT(void){

		// Включаем тактирование GPIOA

		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

	    //LED////////////

	    // Настраиваем PA12 как выход
	    GPIOA->MODER &= ~(GPIO_MODER_MODE12); // Очищаем биты
	    GPIOA->MODER |= GPIO_MODER_MODE12_0;  // Устанавливаем как выход
	    // Настраиваем PA12 на Push-Pull выход
	    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT12);
	    // Настраиваем PA12 на высокий скоростной режим
	    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED12;
	    // Отключаем Pull-up и Pull-down резисторы
	    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD12);

	    //CAN ///////////

	    // Настройка PA11 (CAN_RX) и PB9 (CAN_TX) в режим альтернативной функции
	    GPIOA->MODER &= ~(GPIO_MODER_MODE11); // Очистка битов режима для PA11
	    GPIOA->MODER |= GPIO_MODER_MODE11_1;  // Альтернативная функция для PA11
	    GPIOA->AFR[1] |= (9 << GPIO_AFRH_AFSEL11_Pos); // AF9 для CAN_RX (PA11)

	    GPIOB->MODER &= ~(GPIO_MODER_MODE9);  // Очистка битов режима для PB9
	    GPIOB->MODER |= GPIO_MODER_MODE9_1;   // Альтернативная функция для PB9
	    GPIOB->AFR[1] |= (9 << GPIO_AFRH_AFSEL9_Pos);  // AF9 для CAN_TX (PB9)

	    // Настройка PA11 и PB9 на высокую скорость
	    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED11;
	    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED9;

}





// Настройка CAN

void CAN_Config(void) {
    // Включение тактирования для CAN
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;

    // Настройка CAN
    FDCAN_GlobalTypeDef *CAN = FDCAN1;

    // Выключаем CAN перед настройкой
    CAN->CCCR |= FDCAN_CCCR_INIT;
    while (!(CAN->CCCR & FDCAN_CCCR_INIT));

    // Настройка битрейта для скорости 500 кбит/с
    CAN->CCCR |= FDCAN_CCCR_CCE; // Разрешение доступа к конфигурационным регистрам

/*
    Baudrate	NSJW	NBRP	NTSEG1	NTSEG2	FDCAN_NBTP (uint32)
    125000		1		640		543		95		0x21F5F
    250000		1		320		271		47		0x10F2F
    500000		1		160		135		23		0x08617
    800000		1		100		84		14		0x0540E
    1000000		1		80		67		11		0x0430B
*/

    // Set the nominal bit timing register
    FDCAN1->NBTP = (1 << FDCAN_NBTP_NSJW_Pos) |
    		       (1 << FDCAN_NBTP_NBRP_Pos) |
				   (66 << FDCAN_NBTP_NTSEG1_Pos)|
				   (11 << FDCAN_NBTP_NTSEG2_Pos);

   //CAN->NBTP = 0x08617; //500

    // Настройка режима работы (нормальный режим)
    CAN->CCCR &= ~FDCAN_CCCR_INIT; // Выход из режима инициализации
    while (CAN->CCCR & FDCAN_CCCR_INIT);
}



void SystemClock_Config(void) {
	/*
    // Включение HSI
    RCC->CR |= RCC_CR_HSION; // Включение HSI
    while (!(RCC->CR & RCC_CR_HSIRDY)); // Ожидание готовности HSI

    // Настройка флеш-памяти для работы на высокой частоте
    FLASH->ACR |= FLASH_ACR_LATENCY_4WS; // Установка латентности Flash (4 такта для 150 МГц)

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

    // Обновление SystemCoreClock
    SystemCoreClockUpdate();
}



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



void Error_Handler(void)
{
  // Обработка ошибок
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  // Обработка ошибок assert
}
#endif /* USE_FULL_ASSERT */


