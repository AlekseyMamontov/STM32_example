


#include "stm32g4xx.h"
#include "INIT_STM32G473_GPIO.h"
#include <CANFD_STM32G473.h>

#include "main.h"
uint8_t led_test = 0;



int main(void)
{
	SystemClock_Config();
	GPIO_INIT();
	FDCANs_Config();

  while (1)
  {

	  CAN1_to_CAN2();
	  CAN2_to_CAN1();


	if(!systick_pause){

    GPIOB->BSRR = led_test? 1 << 11 : 1 <<27;
    led_test = led_test?0:1;
    systick_pause = 500;

    }


  }


}

//////////////// CAN init ////////////////////

void FDCANs_Config(void){


	// Настройка CAN
	//FDCAN_GlobalTypeDef *CAN = FDCAN1;
	uint32_t *RAM_CANFD1 = (uint32_t*)RAMBaseFDCAN1;
	uint32_t *RAM_CANFD2 = (uint32_t*)RAMBaseFDCAN2;
	uint32_t *RAM_CANFD3 = (uint32_t*)RAMBaseFDCAN3;

	// Включение тактирования для CAN
	RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
	RCC->CCIPR &= ~RCC_CCIPR_FDCANSEL;
	// Установить PLL "Q" как источник тактирования для FDCAN
	RCC->CCIPR |= (0x1 << RCC_CCIPR_FDCANSEL_Pos); // Установить PLL "Q"

	// Выключаем CAN перед настройкой
	FDCAN1->CCCR |= FDCAN_CCCR_INIT;
	while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT));
	FDCAN2->CCCR |= FDCAN_CCCR_INIT;
	while (!(FDCAN2->CCCR & FDCAN_CCCR_INIT));
	FDCAN3->CCCR |= FDCAN_CCCR_INIT;
	while (!(FDCAN3->CCCR & FDCAN_CCCR_INIT));

	// Разрешение доступа к конфигурационным  регистрам

	FDCAN1->CCCR |= FDCAN_CCCR_CCE;
	FDCAN2->CCCR |= FDCAN_CCCR_CCE;
	FDCAN3->CCCR |= FDCAN_CCCR_CCE;

	// Настройка битрейта для скорости 500 кбит/с
	/*
	 Baudrate	NSJW	NBRP	NTSEG1	NTSEG2	FDCAN_NBTP (uint32)
	 125000		1		640		543	void C	95		0x21F5F
	 250000		1		320		271		47		0x10F2F
	 500000		2		160		67		12		0x08617
	 800000		1		100		84		14		0x0540E
	 1000000		1		80		67		11		0x0430B
	 */

	// Set the nominal bit timing register -1 (500кб)

	FDCAN1->NBTP = (1 << FDCAN_NBTP_NSJW_Pos) | (1 << FDCAN_NBTP_NBRP_Pos)
			| (66 << FDCAN_NBTP_NTSEG1_Pos) | (11 << FDCAN_NBTP_NTSEG2_Pos);
	FDCAN2->NBTP = (1 << FDCAN_NBTP_NSJW_Pos) | (1 << FDCAN_NBTP_NBRP_Pos)
			| (66 << FDCAN_NBTP_NTSEG1_Pos) | (11 << FDCAN_NBTP_NTSEG2_Pos);
	FDCAN3->NBTP = (1 << FDCAN_NBTP_NSJW_Pos) | (1 << FDCAN_NBTP_NBRP_Pos)
			| (66 << FDCAN_NBTP_NTSEG1_Pos) | (11 << FDCAN_NBTP_NTSEG2_Pos);

	// Clear message RAM

	for (uint8_t i = 0; i < 212; i++) {
		RAM_CANFD1[i] = 0;
		RAM_CANFD2[i] = 0;
		RAM_CANFD3[i] = 0;
	};

	/*  FDCAN global filter configuration register (FDCAN_RXGFC)
	 Address offset: 0x0080
	 Reset value: 0x0000 0000
	 */

	//CAN->RXGFC = STDfilter_n(2)|EXTfilter_n(0)|ANFS_Reject_rx|ANFE_Reject_rx;

	// ID filters 100 and 80

	/*filterRAM[0] = STDfilterID_DUAL | STDfilterRxFIFO0| STDfilterID1(0x100) | STDfilterID2(0x80);
	ID = 100, Standard ID, Store in FIFO0

			// Включить прерывания в FDCAN FIFO

	CAN->IE |= 3; // FDCAN_IE_RF0NE_| RF0FE
	CAN->ILS |= 1;  // RXFIFO0: RX FIFO bit grouping the following interruption
	CAN->ILE |= 2;  // Enable IT0

	//FDCAN1->IE |= FDCAN_IE_RF0NE;
 */


	// Настройка режима работы (нормальный режим)
	FDCAN1->CCCR &= ~FDCAN_CCCR_INIT; // Выход из режима инициализации
	while (FDCAN1->CCCR & FDCAN_CCCR_INIT);
	FDCAN2->CCCR &= ~FDCAN_CCCR_INIT; // Выход из режима инициализации
	while (FDCAN2->CCCR & FDCAN_CCCR_INIT);
	FDCAN3->CCCR &= ~FDCAN_CCCR_INIT; // Выход из режима инициализации
	while (FDCAN3->CCCR & FDCAN_CCCR_INIT);
	// включить блок syscfg

	//NVIC_SetPriority(FDCAN1_IT0_IRQn, 0); // Установить приоритет прерывания
	//NVIC_EnableIRQ(FDCAN1_IT0_IRQn); FIFO1
	//NVIC_EnableIRQ(FDCAN1_IT1_IRQn); FIFO0
	// Включить прерывание
	//   NVIC_EnableIRQ(FDCAN1_IT1_IRQn);

}

void CAN1_to_CAN2(void){

	uint32_t index_rxfifo = 0,index_txfifo = 0, rxHeader0, rxHeader1;
	uint32_t *RxBuffer,*TxBuffer;
	uint8_t  status;
	// Проверить cообщение в FIFO 0
	if (FDCAN1->RXF0S & FDCAN_RXF0S_F0FL){

		index_rxfifo = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
		RxBuffer = (uint32_t*) (RAMBaseFDCAN1 + RamFIFO0RX + (index_rxfifo * 18 * 4));
		rxHeader0 = *RxBuffer++;
		rxHeader1 = *RxBuffer++;
		//id = (rxHeader0 & XTDbit) ? rxHeader0 & 0x1FFFFFFF : (rxHeader0 & 0x1FFFFFFF) >> 18;
		//dlc = (rxHeader1 >> 16) & 0xF;

		//FDCAN1->RXF0A = index_rxfifo;
		//FDCAN1->IR |= 1; // clearFifo

			if ((FDCAN2->TXFQS & FDCAN_TXFQS_TFQF) == 0) {
				    // Получаем индекс вставки в Tx FIFO
				    index_txfifo = (FDCAN2->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
					// Указатель на буфер передачи в Message RAM/
			        TxBuffer =(uint32_t*) (RAMBaseFDCAN2 + RamBaseTX + (index_txfifo * 18 * 4));

			        *TxBuffer++ = rxHeader0&0x7FFFFFFF;
			        status = rxHeader1 & FDFbit?1:0;
			        *TxBuffer++ = rxHeader1&0x3F0000; //header1

			        *TxBuffer++ = *RxBuffer++; //data 1-4
			        *TxBuffer++ = *RxBuffer++; //data 5-8

			        if (status){for(uint8_t i = 0; i < 14; i++){*TxBuffer++ = *RxBuffer++;}};

			FDCAN2->TXBAR = (1 << index_txfifo); // Активация соответствующего запроса на передачу
			FDCAN1->RXF0A = index_rxfifo;

		};
	}
};

void CAN2_to_CAN1(void){

	uint32_t index_rxfifo = 0,index_txfifo = 0, rxHeader0, rxHeader1;
	uint32_t *RxBuffer,*TxBuffer;
	uint8_t  status;
	// Проверить cообщение в FIFO 0
	if (FDCAN2->RXF0S & FDCAN_RXF0S_F0FL){

		index_rxfifo = (FDCAN2->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
		RxBuffer = (uint32_t*) (RAMBaseFDCAN2 + RamFIFO0RX + (index_rxfifo * 18 * 4));
		rxHeader0 = *RxBuffer++;
		rxHeader1 = *RxBuffer++;
		//id = (rxHeader0 & XTDbit) ? rxHeader0 & 0x1FFFFFFF : (rxHeader0 & 0x1FFFFFFF) >> 18;
		//dlc = (rxHeader1 >> 16) & 0xF;

		//FDCAN1->RXF0A = index_rxfifo;
		//FDCAN1->IR |= 1; // clearFifo

			if ((FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) == 0) {
				    // Получаем индекс вставки в Tx FIFO
				    index_txfifo = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
					// Указатель на буфер передачи в Message RAM/
			        TxBuffer =(uint32_t*) (RAMBaseFDCAN1 + RamBaseTX + (index_txfifo * 18 * 4));

			        *TxBuffer++ = rxHeader0&0x7FFFFFFF;
			        status = rxHeader1 & FDFbit?1:0;
			        *TxBuffer++ = rxHeader1&0x3F0000; //header1

			        *TxBuffer++ = *RxBuffer++; //data 1-4
			        *TxBuffer++ = *RxBuffer++; //data 5-8

			        if (status){for(uint8_t i = 0; i < 14; i++){*TxBuffer++ = *RxBuffer++;}};

			FDCAN1->TXBAR = (1 << index_txfifo); // Активация соответствующего запроса на передачу
			FDCAN2->RXF0A = index_rxfifo;

		};
	}
};












void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  //disable_irq();
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
