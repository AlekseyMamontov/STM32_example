#include "stm32g4xx.h"
#include "main.h"

#define RAMBaseFDCAN1  		 0x4000A400
#define STDfilterRxFIFO0     (1 << 27)
#define STDfilterRxFIFO1     (2 << 27)
#define STDfilterID1(a)      (a << 16)
#define STDfilterID2(a)      (a << 0)
#define STDfilterID_DUAL  (1 << 30)
#define ESIbit			   0x80000000
#define XTDbit  		   0x40000000
#define RTRbit  		   0x20000000
#define Ram11bitfilter 0x0000
#define Ram29bitfilter 0x0070
#define RamFIFO0RX	   0x00B0
#define RamFIFO1RX	   0x0188
#define RamTxeventFIFO 0x0260
#define RamBaseTX      0x0278
#define LED1 (1 << 6)
#define LED2 (1 << 5)
#define STDfilter_n(a)       (a << FDCAN_RXGFC_LSS_Pos) //max 28
#define EXTfilter_n(a)       (a << FDCAN_RXGFC_LSE_Pos) //max 8
#define ANFE_FIFO0_rx         0
#define ANFE_FIFO1_rx         (1 << 2)
#define ANFE_Reject_rx        (2 << 2) //0b1100

//Accept Non-matching frames standard
#define ANFS_FIFO0_rx         0
#define ANFS_FIFO1_rx         (1 << 4)
#define ANFS_Reject_rx        (2 << 4) //0b1100


void SystemClock_Config(void) {

    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
       PWR->CR5 &= ~PWR_CR5_R1MODE;
       while (PWR->SR2 & PWR_SR2_VOSF);

    // HSE 8Mhz on
       RCC->CR |= RCC_CR_HSEON;
       while (!(RCC->CR & RCC_CR_HSERDY));

     //Flash
        FLASH->ACR =  FLASH_ACR_DBG_SWEN | FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;


      RCC->PLLCFGR = 0; // reset PLL
      RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_HSE |   // HSE ON
                    (1 << RCC_PLLCFGR_PLLM_Pos) | // PLLM = 2 (8  / 2 = 4 )
                    (80 << RCC_PLLCFGR_PLLN_Pos)| // PLLN = 80 * 4  *  320 )
                    (0 << RCC_PLLCFGR_PLLR_Pos) | // PLLR = 2 (320 МГц / 2 = 160 )
					 (1 << RCC_PLLCFGR_PLLQ_Pos) | // PLLQ = 4 (320 МГц / 4 = 80 ) for FDCAN
                    RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLQEN);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); 


    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // AHB = SYSCLK
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV1; // APB1 = HCLK
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 = HCLK

    
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);



    RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

   SystemCoreClockUpdate();

   SysTick->LOAD = 160000000/1000 - 1;  // (48 mHz / 1000) -1  // 1ms
   SysTick->VAL = 0;  // reset
   SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

   NVIC_EnableIRQ(SysTick_IRQn);

}

void GPIO_INIT(void){

/* LED_1  PA5, FDCAN1 PB8 RX
   LED_2  PA6, FDCAN1 PB9 TX */

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
	//   ------------  151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOA->MODER &=  0b11111111111111111100001111111111;
	GPIOA->MODER |=  0b00000000000000000001010000000000;
	GPIOB->MODER &=  0b11111111111100001111111111111111;
	GPIOB->MODER |=  0b00000000000010100000000000000000;
	//-------------     151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOA->OSPEEDR|= 0b00000000000000000010100000000000;
	GPIOB->OSPEEDR|= 0b00000000000011110000000000000000;
	       //-------	  15  14  13  12  11  10   9   8
	GPIOB->AFR[1]  |= 0b00000000000000000000000010011001;

}

uint16_t systick_pause = 0;

void SysTick_Handler(void) {

	if (systick_pause) systick_pause--;

}


void FDCANs_Config(void){


	uint32_t *RAM_CANFD1 = (uint32_t*)RAMBaseFDCAN1;

	RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
	RCC->CCIPR &= ~RCC_CCIPR_FDCANSEL;
	RCC->CCIPR |= (0x1 << RCC_CCIPR_FDCANSEL_Pos); // Установить PLL "Q"

	FDCAN1->CCCR |= FDCAN_CCCR_INIT;
	while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT));
	FDCAN1->CCCR |= FDCAN_CCCR_CCE;

	// Set the nominal bit timing register -1 (500kb)

	FDCAN1->NBTP = (1 << FDCAN_NBTP_NSJW_Pos) | (1 << FDCAN_NBTP_NBRP_Pos)
			| (66 << FDCAN_NBTP_NTSEG1_Pos) | (11 << FDCAN_NBTP_NTSEG2_Pos);

	for (uint8_t i = 0; i < 212; i++) {RAM_CANFD1[i] = 0;};

	FDCAN1->RXGFC = STDfilter_n(2)|EXTfilter_n(0)|ANFS_Reject_rx|ANFE_Reject_rx;
	*RAM_CANFD1++ = STDfilterID_DUAL | STDfilterRxFIFO0| STDfilterID1(0x20) | STDfilterID2(0x25);
	*RAM_CANFD1   = STDfilterID_DUAL | STDfilterRxFIFO1| STDfilterID1(0x30) | STDfilterID2(0x35);


	FDCAN1->IE  |= 0x09; // FDCAN_IE_RF0N_| RF1N fifoO/fifo1
	FDCAN1->ILS |= 0;    // IT0
	FDCAN1->ILE |= 1;    // Enable bit1 it0?



	FDCAN1->CCCR &= ~FDCAN_CCCR_INIT; 
	while (FDCAN1->CCCR & FDCAN_CCCR_INIT);

    NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    //NVIC_EnableIRQ(FDCAN1_IT1_IRQn);

}

void FDCAN1_IT0_IRQHandler(void) {

	uint32_t index_rxfifo = 0, rxHeader0,id;// rxHeader1,dlc;
	uint32_t *RxBuffer;

	// msg FIFO 0
	if (FDCAN1->IR & 1) {

		index_rxfifo = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
		RxBuffer = (uint32_t*) (RAMBaseFDCAN1 + RamFIFO0RX + (index_rxfifo * 18 * 4));
		rxHeader0 = *RxBuffer++;
		//rxHeader1 = *RxBuffer++;
		id = (rxHeader0 & XTDbit) ? rxHeader0 & 0x1FFFFFFF : (rxHeader0 & 0x1FFFFFFF) >> 18;
		//dlc = (rxHeader1 >> 16) & 0xF;

		if(id == 0x20){ GPIOA->BSRR = (GPIOA->ODR & LED1)? LED1 <<16: LED1;};

		FDCAN1->RXF0A = index_rxfifo;

		FDCAN1->IR |= 1; // clearFifo
	}

	// msg FIFO 1

	if (FDCAN1->IR & 8) {

			index_rxfifo = (FDCAN1->RXF1S & FDCAN_RXF1S_F1GI) >> FDCAN_RXF1S_F1GI_Pos;
			RxBuffer = (uint32_t*) (RAMBaseFDCAN1 + RamFIFO1RX + (index_rxfifo * 18 * 4));
			rxHeader0 = *RxBuffer++;
			//rxHeader1 = *RxBuffer++;
			id = (rxHeader0 & XTDbit) ? rxHeader0 & 0x1FFFFFFF : (rxHeader0 & 0x1FFFFFFF) >> 18;
			//dlc = (rxHeader1 >> 16) & 0xF;

			if(id==0x30){ GPIOA->BSRR = (GPIOA->ODR & LED1)? LED1 <<16: LED1;};


			FDCAN1->RXF1A = index_rxfifo;

			FDCAN1->IR |= 8; // clearFifo
		}

}


void SendCANframe() {

	uint32_t PutIndex;
	uint32_t *TxBuffer;

	// Проверяем, что Tx FIFO не заполнен
	if ((FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) == 0) {

		PutIndex = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
		TxBuffer = (uint32_t*) (RAMBaseFDCAN1 + RamBaseTX + (PutIndex * 18 * 4));
		*TxBuffer++ = (0x300 << 18);
		*TxBuffer++ = (8 << 16); //header1
		*TxBuffer++ = 0x11223344; //data 1-4
		*TxBuffer++ = 0x55667788; //data 5-8

		FDCAN1->TXBAR = (1 << PutIndex);

	};
}



int main(void)
{

	SystemClock_Config();
	GPIO_INIT();
	FDCANs_Config();

	GPIOA->BSRR = LED1|LED2;

   while(1){

	   if(!systick_pause){

		   GPIOA->BSRR = (GPIOA->ODR & LED2)? LED2 <<16: LED2;
	       systick_pause = 1000;
	       SendCANframe();

	       }

	 //  FDCAN1_IT0_IRQHandler();

   };


};



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
