
#include "stm32g4xx.h"
#include "INIT_STM32G431_GPIO.h"
#include <CANFD_STM32G431.h>
#include "I2C.h"
#include "SPI.h"
#include "main.h"

//#include "Ixm42xxxTransport.h"
//#include "Ixm42xxxDefs.h"
//#include "Ixm42xxxDriver_HL.h"

uint8_t test = 1, trigger = 0, command = 0xEF, data = 0;
uint32_t pause = 500;

int main(void) {
	// Настройка системного тактирования

	SystemClock_Config();
	GPIO_INIT();
	CAN_Config();
	ConfigSPI2();
	I2C2_Init();
	BMP280_Init(&BMP280_sensor1);

	__enable_irq();

	// Сообщение для отправки

	uint32_t data32[18] = { 0 };
	uint32_t id = 0x222;  // Стандартный идентификатор CAN

	while (1) {

		// Выключаем светодиод
		if (!pause) {

			GPIOA->BSRR = trigger ? GPIO_BSRR_BS12 : GPIO_BSRR_BR12;
			trigger = trigger ? 0 : 1;
			pause = 500;

		};

		if (test) {

			SPI2_data(command, data);
			data = 0;
			data32[0] = SPI2_data(0x80 | 0x1E, data);
			data32[0] |= (SPI2_data(0x80 | 0x1D, data)) << 8;
			data32[0] |= (SPI2_data(0x80 | 0x20, data)) << 16;
			data32[0] |= (SPI2_data(0x80 | 0x1f, data)) << 24;
			data32[1] = SPI2_data(0x80 | 0x22, data);
			data32[1] |= (SPI2_data(0x80 | 0x21, data)) << 8;
			data32[1] |= (SPI2_data(0x80 | 0x24, data)) << 16;
			data32[1] |= (SPI2_data(0x80 | 0x23, data)) << 24;

			CAN_SendMessage(id, (uint8_t*) data32, 8); //(uint8_t*)RAM + counterRAM*4

			BMP280_Read_Raw_Data(&BMP280_sensor1);
			data32[0] = BMP280_Compensate_Temperature(&BMP280_sensor1);
			data32[1] = BMP280_Compensate_Pressure(&BMP280_sensor1);
			CAN_SendMessage(id, (uint8_t*) data32, 8);

			test = 0;
		};

	}

}
;
///////

// Функция для отправки классического сообщения CAN

// Message FIFO 0

void FDCAN1_IT1_IRQHandler(void) {

	uint32_t index_rxfifo = 0, rxHeader0, rxHeader1, id, dlc;
	uint32_t *RxBuffer;

	// Проверить, было ли прерывание из FIFO 0
	if (FDCAN1->IR & 1 || FDCAN1->IR & 2) {

		index_rxfifo = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI)
				>> FDCAN_RXF0S_F0GI_Pos;
		RxBuffer = (uint32_t*) (RAMBaseFDCAN1 + RamFIFO0RX
				+ (index_rxfifo * 18 * 4));
		rxHeader0 = *RxBuffer++;
		rxHeader1 = *RxBuffer++;
		id = (rxHeader0 & XTDbit) ?
				rxHeader0 & 0x1FFFFFFF : (rxHeader0 & 0x1FFFFFFF) >> 18;
		dlc = (rxHeader1 >> 16) & 0xF;

		switch (id) {

		case 0x100:

			if (dlc < 2)
				break;

			test = 1;
			data = (*RxBuffer) & 0xFF;
			command = (*RxBuffer >> 8) & 0xFF;

			break;

		default:
			break;

		};

		FDCAN1->RXF0A = index_rxfifo;

		FDCAN1->IR |= 1; // clearFifo
	}

}

////////////////////////////////////////////////////////////////////////////////////////////////

// Настройка CAN

//void FDCAN1_IT0_IRQHandler(void){test = 1;};

///////////////////// SPI2/////////////////////

void SysTick_Handler(void) {

	if (pause)
		pause--;

}
;

void Error_Handler(void) {
	// Обработка ошибок
	__disable_irq();
	while (1) {
	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  // Обработка ошибок assert
}
#endif /* USE_FULL_ASSERT */

