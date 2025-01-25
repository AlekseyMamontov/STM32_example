#include "stm32g4xx.h"
#include "main.h"
#include "CanFD_stm32G4.h"


uint8_t test = 1, trigger = 0;
uint32_t pause = 500;

int main(void) {
    // Настройка системного тактирования

    SystemClock_Config();
    GPIO_INIT();
    CAN_Config();
    __enable_irq();
    // Сообщение для отправки
    uint8_t   data[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    					  0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00};
    uint32_t  data32[18]={0};
    uint32_t  id = 0x222;  // Стандартный идентификатор CAN
    uint32_t* RAM = (uint32_t*)data32;///RAMBaseFDCAN1;//RAMBaseFDCAN1;
    uint32_t  counterRAM = 0;
    uint32_t getIndex;

    while (1) {

        // Включаем светодиод
       // GPIOA->BSRR = GPIO_BSRR_BS12; // Включаем светодиод
       // delay_ms(500); // Задержка 500 мс

        // Выключаем светодиод
        if(!pause){

          GPIOA->BSRR = trigger? GPIO_BSRR_BS12:GPIO_BSRR_BR12;
          trigger = trigger?0:1;
          pause = 500;

          };
/*

        if( FDCAN1->RXF0S & FDCAN_RXF0S_F0FL_Msk){

            getIndex = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI_Msk) >> 8;
        	test = 1;

        	FDCAN1->RXF0A = getIndex;
        }
*/



        if(test){


        	data32[0] = FDCAN1->IE;
        	data32[1] = FDCAN1->ILE;
        	CAN_SendMessage(id,data32, 8);//(uint8_t*)RAM + counterRAM*4


         //counterRAM++;
        //if(counterRAM >= 28) counterRAM = 0;
        test = 0;
        };








    }

};
///////


// Функция для отправки классического сообщения CAN








////////////////////////////////////////////////////////////////////////////////////////////////

void GPIO_INIT(void){

		// Включаем тактирование GPIOA

		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;



		//---------------  	151413121110 9 8 7 6 5 4 3 2 1 0
	//	GPIOA->MODER   |= 0b00000000000000000101010101010101; // General purpose output mode
	//	GPIOA->OSPEEDR |= 0b00000000000000001111111111111111; //0b11  high speed



		//---------------  	151413121110 9 8 7 6 5 4 3 2 1 0
	//	GPIOB->MODER   |= 0b00000000000000000101010101010101; // General purpose output mode
    //	GPIOB->OSPEEDR |= 0b00000000000000001111111111111111; //0b11  high speed




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


	    // SPI2







}





// Настройка CAN

void CAN_Config(void) {

	// Настройка CAN
	FDCAN_GlobalTypeDef *CAN = FDCAN1;
	uint32_t *filterRAM = (uint32_t *)RAMBaseFDCAN1;

    // Включение тактирования для CAN
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;

    RCC->CCIPR &= ~RCC_CCIPR_FDCANSEL;
    // Установить PLL "Q" как источник тактирования для FDCAN
    RCC->CCIPR |= (0x1 << RCC_CCIPR_FDCANSEL_Pos); // Установить PLL "Q"



    // Выключаем CAN перед настройкой
    CAN->CCCR |= FDCAN_CCCR_INIT;
    while (!(CAN->CCCR & FDCAN_CCCR_INIT));

    // Настройка битрейта для скорости 500 кбит/с
    CAN->CCCR |= FDCAN_CCCR_CCE; // Разрешение доступа к конфигурационным регистрам

/*
    Baudrate	NSJW	NBRP	NTSEG1	NTSEG2	FDCAN_NBTP (uint32)
    125000		1		640		543		95		0x21F5F
    250000		1		320		271		47		0x10F2F
    500000		2		160		67		12		0x08617
    800000		1		100		84		14		0x0540E
    1000000		1		80		67		11		0x0430B
*/

    // Set the nominal bit timing register
    CAN->NBTP = (1 << FDCAN_NBTP_NSJW_Pos) |
    		       (1 << FDCAN_NBTP_NBRP_Pos) |
				   (66 << FDCAN_NBTP_NTSEG1_Pos)|
				   (11 << FDCAN_NBTP_NTSEG2_Pos);

    // Clear message RAM

    for(uint8_t i=0;i< 212;i++){filterRAM[i] = 0;};


      /*  FDCAN global filter configuration register (FDCAN_RXGFC)
      	  Address offset: 0x0080
     	  Reset value: 0x0000 0000
     */

       CAN->RXGFC = STDfilter_n(2)|EXTfilter_n(0)|ANFS_Reject_rx|ANFE_Reject_rx;

       // ID filters 100 and 80

       filterRAM[0] = STDfilterID_DUAL | STDfilterRxFIFO0 | STDfilterID1(0x100) | STDfilterID2(0x80); // ID = 100, Standard ID, Store in FIFO0


       // Включить прерывания в FDCAN FIFO

         CAN->IE |=  3; // FDCAN_IE_RF0NE_| RF0FE
         CAN->ILS |= 1;  // RXFIFO0: RX FIFO bit grouping the following interruption
         CAN->ILE |= 2;  // Enable IT0

         //FDCAN1->IE |= FDCAN_IE_RF0NE;

       // Настройка режима работы (нормальный режим)
       CAN->CCCR &= ~FDCAN_CCCR_INIT; // Выход из режима инициализации
       while (CAN->CCCR & FDCAN_CCCR_INIT);


        // включить блок syscfg

       //NVIC_SetPriority(FDCAN1_IT0_IRQn, 0); // Установить приоритет прерывания
      //NVIC_EnableIRQ(FDCAN1_IT0_IRQn);;
       NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
       // Включить прерывание
 //   NVIC_EnableIRQ(FDCAN1_IT1_IRQn);

}

//void FDCAN1_IT0_IRQHandler(void){test = 1;};

void FDCAN1_IT1_IRQHandler(void){

	uint32_t index_rxfifo = 0, rxHeader0,rxHeader1,id,dlc;
	uint32_t* RxBuffer;

	test = 1;

    // Проверить, было ли прерывание из FIFO 0
	if (FDCAN1->IR & 1 || FDCAN1->IR & 2) {

		index_rxfifo = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
		RxBuffer = (uint32_t *)(RAMBaseFDCAN1 + RamFIFO0RX + (index_rxfifo * 18 *4));
		rxHeader0 = *RxBuffer ++;
		rxHeader1 = *RxBuffer ++;
		id = (rxHeader0&XTDbit)? rxHeader0 & 0x1FFFFFFF : (rxHeader0 & 0x1FFFFFFF)>>18;
		dlc = (rxHeader1 >> 16) & 0xF;


		switch(id){

		case 100:

			test = 1;

		   break;

		default:
		break;

		};



		FDCAN1->RXF0A = index_rxfifo;

		FDCAN1->IR |= 1; // clearFifo
    }


}




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
void SysTick_Handler(void){


	if(pause)pause --;


};


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


