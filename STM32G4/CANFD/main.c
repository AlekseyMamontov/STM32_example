
#include "stm32g4xx.h"
#include "INIT_STM32G431_GPIO.h"
#include <CANFD_STM32G431.h>
#include "I2C.h"
#include "SPI.h"
#include "DMA.h"
#include "main.h"
#include "IIM_42652.h"
#include "LIS3MD.h"

//#include "Ixm42xxxTransport.h"
//#include "Ixm42xxxDefs.h"
//#include "Ixm42xxxDriver_HL.h"

uint8_t test = 1, trigger = 0, command = 0xEF, data = 0,readyINT1 =1,sendACC=0,lis3m=0;
uint32_t pause = 500;

int main(void) {
	// Настройка системного тактирования

	SystemClock_Config();
	GPIO_INIT();
	CAN_Config();
	ConfigSPI2();
	I2C2_Init();
	DMA_Init();
	BMP280_Init(&BMP280_sensor1);
	init_iim42652(&imu_iim42652);
	init_lis3md(&mag_lis3md);
	__enable_irq();


	uint32_t data32[18] = { 0 };
	uint32_t id = 0x222;  // Стандартный идентификатор CAN

	while (1) {

		if(readyINT1){
			readyINT1 = 0;sendACC = 1;
			SPI2_data((INT_STATUS|READ_REG_II42xxx)<<8 | 0x00);
			load_gyro_aceel_temp(&imu_iim42652);}
//test

		if (!systick_pause) {

			GPIOA->BSRR = trigger ? GPIO_BSRR_BS12 : GPIO_BSRR_BR12;

			if(sendACC){
				sendACC =0;
				//*(imu_iim42652.raw_fifo_buf +15) = lis3m;
				CAN_SendMessage(id+1,imu_iim42652.raw_fifo_buf, 8); //(uint8_t*)RAM + counterRAM*4
				CAN_SendMessage(id+2,(imu_iim42652.raw_fifo_buf)+8, 8);
			};
			BMP280_Read_Raw_Data(&BMP280_sensor1);
			data32[0] = BMP280_Compensate_Temperature(&BMP280_sensor1);
			data32[1] = BMP280_Compensate_Pressure(&BMP280_sensor1);
			CAN_SendMessage(id, (uint8_t*) data32, 8);

			load_mag_lis3mdtr(&mag_lis3md);
			CAN_SendMessage(id+3,mag_lis3md.raw_fifo_buffer, 8);

			trigger = trigger ? 0 : 1;

			systick_pause = 500;
		};

		if (test){test = 0;}

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

/*

PB12 IMU_int1  EXTI 12 configuration bits
PB11 IMU int2
*/

void EXTI15_10_IRQHandler(void) {

    if (EXTI->PR1 & (1 << 12)) { // PB12 INT1

    	if(!readyINT1){readyINT1=1;};

    	EXTI->PR1 |= (1 << 12); // Сброс флага
    }

    if (EXTI->PR1 & (1 << 11)) { //PB11  INT2



        EXTI->PR1 |= (1 << 11); // Сброс флага
   }
}
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

