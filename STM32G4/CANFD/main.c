
#include "stm32g4xx.h"

#include <math.h>
#include "Fusion.h"
#include "INIT_STM32G431_GPIO.h"
#include <CANFD_STM32G431.h>
#include "BMP280.h"
#include "DMA.h"
#include "SPI.h"
#include "IIM_42652.h"
#include "LIS3MD.h"
#include "main.h"




uint8_t test = 1,
		trigger = 0,
		command = 0xEF,
		data = 0,
		sendACC=0,sendMAG=0,
		sendPs=0, test_ps=0;


uint32_t pause = 500;



FusionAhrs ahrs;
FusionQuaternion quaternion;
FusionVector acc,gyro,mag;
float heading;
#define SAMPLE_RATE 1000.0f
#define DELTA_TIME (1.0f / SAMPLE_RATE)
#define ACCEL_SCALE 2048.0f  // ±16g
#define GYRO_SCALE 16.384f   // ±2000 dps
#define MAG_SCALE 0.15f      // Пример (настройте под ваш магнитометр)

uint16_t spi2_rx_buf[30]={0},spi2rxsize = 12,
		 spi2_tx_buf[30]={0},spi2txsize = 12;

uint8_t  spi2_rx_data[30]={0},*fdata, dmaComplete=0, readyINT1 =0;




int main(void) {

///////  Настройка системного тактирования

	SystemClock_Config();
	GPIO_INIT();
	CAN_Config();
	Init_SPI_STM32();

///////  Sensors


	init_iim42652(&imu_iim42652);
	init_lis3md	 (&mag_lis3md);
	init_bmp280  (&bmp280_sensor1);

	float data32[2]= {0};
	uint32_t id = 1003;  // Стандартный идентификатор CAN


	while (1) {




///////////////   iim42652

		if(*(imu_iim42652.status) & DMA_OK_IIM42xxx ){

				*(imu_iim42652.status) &= ~DMA_OK_IIM42xxx;//????

				if((*imu_iim42652.DMA_RX_fifo_buf) &0x0C){

				sendACC = 1;

				fdata = (uint8_t*) imu_iim42652.DMA_RX_fifo_buf;
				spi2_rx_data[0] = *fdata ;
				spi2_rx_data[1] = *(fdata+3);
				spi2_rx_data[2] = *(fdata+2);
				spi2_rx_data[3] = *(fdata+5);

				spi2_rx_data[4] = *(fdata+4);
				spi2_rx_data[5] = *(fdata+7);

				spi2_rx_data[6] = *(fdata+6);
				spi2_rx_data[7] = *(fdata+9);

				spi2_rx_data[8] = *(fdata+8);
				spi2_rx_data[9] = *(fdata+11);

				spi2_rx_data[10] = *(fdata+10);
				spi2_rx_data[11] = *(fdata+13);

				spi2_rx_data[12] = *(fdata+12);
				spi2_rx_data[13] = *(fdata+15);

				spi2_rx_data[14] = *(fdata+14);
				spi2_rx_data[15] = *(fdata+17);

				spi2_rx_data[16] = *(fdata+16);


			}
		}

/////////////////  lis3md

		if(*(mag_lis3md.status)&DMA_OK_LIS3MXX){

			*(mag_lis3md.status) &= ~DMA_OK_LIS3MXX;

			sendMAG = 1;

		};

/////////////////  bmp280



	//	BMP280_Read_Raw_Data(&BMP280_sensor1);

		if(*(bmp280_sensor1.status)&DMA_OK_BMP280 ){

			*(bmp280_sensor1.status) &= ~DMA_OK_BMP280;

			sendPs= 1;

		 }else{
				 if (!systick_pause){
					 test_ps++;
					 if(test_ps >= 25){
						 test_ps = 0;
						 BMP280_Read_Data(&bmp280_sensor1);
						 sendPs= 1;
					 };
				 };
			  }





//////////////////   CAN_SendMessage


 		if (!systick_pause) {



			if(sendACC){sendACC =0;
						CAN_SendMessage(id+1,(uint8_t*) spi2_rx_data, 8);
					    CAN_SendMessage(id+2,(uint8_t*) spi2_rx_data+8, 8);

			};

			if(sendMAG){CAN_SendMessage(id+3,(uint8_t*)mag_lis3md.DMA_RX_fifo_buf, 8);sendMAG=0;}

			if(sendPs){

				data32[0] = BMP280_Compensate_Temperature(&bmp280_sensor1);
				data32[1] = BMP280_Compensate_Pressure(&bmp280_sensor1);
				CAN_SendMessage(id+4, (uint8_t*)bmp280_sensor1.DMArx_buf, 8);
				sendPs = 0;

			}

			///////// LED

			GPIOA->BSRR = trigger ? GPIO_BSRR_BS12 : GPIO_BSRR_BR12;
			trigger = trigger ? 0 : 1;
			systick_pause = 20;
		};

		if (test){test = 0;}

	}

}
;
///////////// IRQ block CANFD


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

///////////// IRQ IIM42652

void DMA1_Channel1_IRQHandler(void) {

    if (DMA1->ISR & DMA_ISR_TCIF1) {

        DMA1->IFCR |= DMA_IFCR_CTCIF1;

        //DMA1_Channel1->CCR &= ~DMA_CCR_EN;

        DMAtx_IIM42XXX->CCR &= ~DMA_CCR_EN;

         IIM42XXX_CS_off

         iim_42652_status &= ~INT_FIFO_IIM42xxx;
         iim_42652_status |= DMA_OK_IIM42xxx;

    }

  DMA1->IFCR |= DMA_IFCR_CGIF1;
}
/*

PB12 IMU_int1  EXTI 12 configuration bits
PB11 IMU int2
*/

void EXTI15_10_IRQHandler(void) {

    if (EXTI->PR1 & (1 << 12)) { // PB12 INT1

    	if((iim_42652_status & INT_FIFO_IIM42xxx) == 0){

    	  iim_42652_status |= INT_FIFO_IIM42xxx;

    	  IIM42XXX_CS_on

          DMA_TX_buf_iim42652[0] = 0xAD00; // READ INT_STATUS0,FL,FH,FIFO (packet)


    	  DMAtx_IIM42XXX->CNDTR = 12; //24 byte
    	  DMAtx_IIM42XXX->CMAR = (uint32_t)DMA_TX_buf_iim42652;
    	  DMAtx_IIM42XXX->CCR |= DMA_CCR_EN;// enable tx dma
    	}

    	EXTI->PR1 |= (1 << 12); // Сброс флага

    }


    if (EXTI->PR1 & (1 << 11)) { //PB11  INT2

    	EXTI->PR1 |= (1 << 11); // Сброс флага
   }
}


///////////// IRQ LISM3D

void DMA1_Channel3_IRQHandler(void) {

    if (DMA1->ISR & DMA_ISR_TCIF3) {

        DMA1->IFCR |= DMA_IFCR_CTCIF3;

        DMAtx_LIS3M->CCR &= ~DMA_CCR_EN;

        LIS3M_CS_off

        *(mag_lis3md.status) &= ~INT_FIFO_LIS3MXX;
        *(mag_lis3md.status) |= DMA_OK_LIS3MXX;

    }

  DMA1->IFCR |= DMA_IFCR_CGIF3;
}



void EXTI1_IRQHandler(void){


	if((*(mag_lis3md.status) & INT_FIFO_LIS3MXX) == 0){

		*(mag_lis3md.status) |= INT_FIFO_LIS3MXX;

		LIS3M_CS_on;

		mag_lis3md.DMA_TX_fifo_buf[0]= ((LIS3M_STATUS_REG|INC_REG_LIS3M |READ_REG_LIS3M)<<8|0);

		 DMAtx_LIS3M->CNDTR = *(mag_lis3md.n_16bit_packet_fifo); //8 byte
		 DMAtx_LIS3M->CMAR = (uint32_t)mag_lis3md.DMA_TX_fifo_buf;
		 DMAtx_LIS3M->CCR |= DMA_CCR_EN;// enable tx dma
	}

	//if(!lis3m)lis3m = 1;

	EXTI->PR1 |= (1 << 1);//PB1

};



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
/*

    FusionAhrsInitialise(&ahrs);
    FusionAhrsSetSettings(&ahrs, &(FusionAhrsSettings){
        .gain = 0.5f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f
        // Убрано rejectionTimeout, так как его нет в вашей версии
    });



			 //FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, acc, DELTA_TIME);
			 //heading = FusionCompassCalculateHeading(FusionConventionNwu, acc, mag);
			 //FusionAhrsSetHeading(&ahrs, heading);
			 //quaternion = FusionAhrsGetQuaternion(&ahrs);







			ft32[0] = quaternion.element.w;
			ft32[1] = quaternion.element.x;
			CAN_SendMessage(id+1, (uint8_t*)ft32,8);
			ft32[2] = quaternion.element.y;
			ft32[3] = quaternion.element.z;
			CAN_SendMessage(id+2, (uint8_t*)ft32+2,8);









  */
