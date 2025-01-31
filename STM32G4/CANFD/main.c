#include "stm32g4xx.h"
#include "main.h"
#include "CanFD_stm32G4.h"
#include "I2C.h"


uint8_t test = 1, trigger = 0, command = 0xEF, data = 0;
uint32_t pause = 500;


int main(void) {
    // Настройка системного тактирования

    SystemClock_Config();
    GPIO_INIT();
    CAN_Config();
    ConfigSPI2();
    I2C2_Init();
    BMP280_Init();


    __enable_irq();

    // Сообщение для отправки
    uint8_t   data2[16] = {0};
    uint32_t  data32[18]={0};
    uint32_t  id = 0x222,per32;  // Стандартный идентификатор CAN
    uint32_t* RAM = (uint32_t*)data32;
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

        	  GPIOA->BRR = 1<<10;


        	  while(!(SPI2->SR & SPI_SR_TXE));

        	  per32 = data;
        	  SPI2->DR =((per32 << 8) | command);
        	 // SPI2->DR =0;
        	  while(!(SPI2->SR & SPI_SR_RXNE));

        	  GPIOA->BSRR = 1<<10;

        	  data32[0] = SPI2->DR;
        	 // data32[1] = SPI2->DR;


        	  //if(stat){ data32[0] = SPI2->SR;data32[1]=SPI2->SR;}



        	CAN_SendMessage(id,(uint8_t*)data32, 8);//(uint8_t*)RAM + counterRAM*4


        	data32[0]=BMP280_ReadTemperature();


        	data32[1]= (I2C2_ReadByte(0x76, 0xD0)<<24) |
        			   (I2C2_ReadByte(0x76, 0xf7)<<16) |
					   (I2C2_ReadByte(0x76, 0xf8)<< 8) |
					   (I2C2_ReadByte(0x76, 0xf9))
					   ;
        	CAN_SendMessage(id,(uint8_t*)data32,8);


         test = 0;
        };








    }

};
///////


// Функция для отправки классического сообщения CAN








////////////////////////////////////////////////////////////////////////////////////////////////

void GPIO_INIT(void){

	/*

	  PA12 LED HEART

      FDCAN1 PB9 TX
      FDCAN1 PA11 RX

      USART1 RX PB7
      USART1 TX PB6

      BMP280
      PA9  I2C2 SCL
      PA8  I2C2_SDA

      IIM-42652
      PB15 Spi2 MOSI
      PB14 SPI2 MiSO
      PB13 SPI2 SCK
      PB12 IMU_int1
      PB11 IMU int2
      PA10 SPI2 CS

      LS3MDl
      PA5  SPI1 _SCK
      PA6  SPI1 _MISO
      PA7  SPI1 _MOSI

	 */

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

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

    // PA12 LED|PA11 FDCAN1_RX|PA10 SPI2_CS|PA9 I2C2 SCL|PA8 I2C2_SDA|PA7 SPI1_MOSI|PA6 SPI1_MISO|PA5 SPI1 _SCK

	//   -------------  151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOA->MODER = 	  0b10101001100110101010101111111111; // General purpose output moder



	//-------------    151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOA->OSPEEDR = 0b11111101111111111111110000000000;

    // PB15 Spi2 MOSI | PB14 SPI2 MiSO|PB13 SPI2 SCK|PB12 IMU_int1|PB11 IMU int2|PB9 FDCAN1_TX|
    // PB7 USART1 RX  | PB6 USART1 TX |PB2 input War|PB1  ReadyOk |PBO SPI1_CS

	//-------------    151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOB->MODER =   0b10101000001110111010111111000001; // General purpose output mode
	//-------------    151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOB->OSPEEDR = 0b11111111110011001111000000111111;

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
	GPIOA->OTYPER = 0b00000000000000000000000000000000;
	GPIOB->OTYPER = 0b00000000000000000000000000000000;

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
		 GPIOA->PUPDR = 0b01100100000000000000000000000000; // General purpose output mode
	     //-------------  151413121110 9 8 7 6 5 4 3 2 1 0
		 GPIOB->PUPDR = 0b00000000000000010000000000000000; // General purpose output mode

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

		 // PA7 SPI1_MOSI AF5| PA6 SPI1_MISO AF5 | PA5 SPI1_SCK AF5
			//-------	     7   6   5   4   3   2   1   0
		GPIOA->AFR[0] = 0b01010101010100000000000000000000;

		// PA12 LED    |PA11 FDCAN1_RX AF9|PA10 SPI2_CS |PA9 I2C2 SCL AF4|PA8 I2C2_SDA AF4
		   //-------	    15  14  13  12  11  10   9   8
	    GPIOA->AFR[1] = 0b00000000000000001001000001000100;



	       // PB7 USART1 RX AF7 | PB6 USART1 TX AF7 |PB2 input War|PB1  ReadyOk |PBO s pi!_CS
	       //-------	     7   6   5   4   3   2   1   0
	    GPIOB->AFR[0] = 0b01110111000000000000000000000000;

	        // PB15 Spi2 MOSI AF5| PB14 SPI2 MiSO AF5|PB13 SPI2 SCK AF5|PB12 IMU_int1|PB11 IMU int2|PB9 FDCAN1_TX AF9|
	    	//-------	    15  14  13  12  11  10   9   8
	    GPIOB->AFR[1] = 0b01010101010100000000000010010000;





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

  // PA10 SPI2_CS / PBO spi2_CS

  GPIOA->BSRR = 1 << 10; //spi2cs
  GPIOB->BSRR = 1;  // spi1cs

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

	//test = 1;

    // Проверить, было ли прерывание из FIFO 0
	if (FDCAN1->IR & 1 || FDCAN1->IR & 2) {

		index_rxfifo = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
		RxBuffer = (uint32_t *)(RAMBaseFDCAN1 + RamFIFO0RX + (index_rxfifo * 18 *4));
		rxHeader0 = *RxBuffer ++;
		rxHeader1 = *RxBuffer ++;
		id = (rxHeader0&XTDbit)? rxHeader0 & 0x1FFFFFFF : (rxHeader0 & 0x1FFFFFFF)>>18;
		dlc = (rxHeader1 >> 16) & 0xF;


		switch(id){

		case 0x100:




			if (dlc<2) break;

			test = 1;
			command = (*RxBuffer) & 0xFF;
			data = (*RxBuffer >> 8) & 0xFF;

		   break;

		default:
		break;

		};



		FDCAN1->RXF0A = index_rxfifo;

		FDCAN1->IR |= 1; // clearFifo
    }


}

///////////////////// SPI2/////////////////////

void ConfigSPI2(void){

	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;


	/*
	 SPI control register 1 (SPIx_CR1) Address offset: 0x00 Reset value: 0x0000

	 Bit 15 BIDIMODE: Bidirectional data mode enable.
			This bit enables half-duplex communication using common single bidirectional data line.
			Keep RXONLY bit clear when bidirectional mode is active.
			0: 2-line unidirectional data mode selected
			1: 1-line bidirectional data mode selected
N
	Bit 14 BIDIOE: Output enable in bidirectional mode
			This bit combined with the BIDIMODE bit selects the direction of transfer in bidirectional mode.
			0: Output disabled (receive-only mode)
			1: Output enabled (transmit-only mode)
		Note: In master mode, the MOSI pin is used and in slave mode, the MISO pin is used.

		Bit 13 CRCEN: Hardware CRC calculation enable
			0: CRC calculation disabled
			1: CRC calculation enabled
		Note: This bit should be written only when SPI is disabled (SPE = ‘0’) for correct operation.

		Bit 12 CRCNEXT: Transmit CRC next
			0: Next transmit value is from Tx buffer.
			1: Next transmit value is from Tx CRC register.
			Note: This bit has to be written as soon as the last data is written in the SPIx_DR register.

		Bit 11 CRCL: CRC length
			This bit is set and cleared by software to select the CRC length.
			0: 8-bit CRC length
			1: 16-bit CRC length
		Note: This bit should be written only when SPI is disabled (SPE = ‘0’) for correct operation.

		Bit 10 RXONLY: Receive only mode enabled.
		This bit enables simplex communication using a single unidirectional line to receive data
			exclusively. Keep BIDIMODE bit clear when receive only mode is active.This bit is also
			useful in a multislave system in which this particular slave is not accessed, the output from
				the accessed slave is not corrupted.
			0: Full-duplex (Transmit and receive)
			1: Output disabled (Receive-only mode)

			Bit 9 SSM: Software slave management
			When the SSM bit is set, the NSS pin input is replaced with the value from the SSI bit.
			0: Software slave management disabled
			1: Software slave management enabled

			Bit 8 SSI: Internal slave select
				This bit has an effect only when the SSM bit is set. The value of this bit is forced onto the
					NSS pin and the I/O value of the NSS pin is ignored.
			Note: This bit is not used in I2S mode and SPI TI mode.
			Bit 7 LSBFIRST: Frame format
					0: data is transmitted / received with the MSB first
					1: data is transmitted / received with the LSB first
			Note: 1. This bit should not be changed when communication is ongoing.
			2. This bit is not used in I2S mode and SPI TI mode.

			Bit 6 SPE: SPI enable
				0: Peripheral disabled
				1: Peripheral enabled
				Note: When disabling the SPI, follow the procedure described in Procedure for disabling the
				SPI on page 1761.
				Bits 5:3 BR[2:0]: Baud rate control
					000: fPCLK/2
					001: fPCLK/4
					010: fPCLK/8
					011: fPCLK/16
					100: fPCLK/32
					101: fPCLK/64
					110: fPCLK/128
					111: fPCLK/256

				Bit 2 MSTR: Master selection
					0: Slave configuration
					1: Master configuration

	 */

	SPI2->CR1 =  (1 << 2) | (3 << 3) | (3 << 8)|0; // master | clk/8 |  SSM=1 SSI =1 CPOL0 _/


/*
 SPI control register 2 (SPIx_CR2) Address offset: 0x04 Reset value: 0x0700 (8 bit)

		Bit 14 LDMA_TX: Last DMA transfer for transmission
			This bit is used in data packing mode, to define if the total number of data to transmit by DMA
			is odd or even. It has significance only if the TXDMAEN bit in the SPIx_CR2 register is set
			and if packing mode is used (data length =< 8-bit and write access to SPIx_DR is 16-bit
			wide). It has to be written when the SPI is disabled (SPE = 0 in the SPIx_CR1 register).
					0: Number of data to transfer is even
					1: Number of data to transfer is odd
				Note: Refer to Procedure for disabling the SPI on page 1761 if the CRCEN bit is set.

		Bit 13 LDMA_RX: Last DMA transfer for reception
			This bit is used in data packing mode, to define if the total number of data to receive by DMA
			is odd or even. It has significance only if the RXDMAEN bit in the SPIx_CR2 register is set
			and if packing mode is used (data length =< 8-bit and write access to SPIx_DR is 16-bit
			wide). It has to be written when the SPI is disabled (SPE = 0 in the SPIx_CR1 register).
					0: Number of data to transfer is even
					1: Number of data to transfer is odd
			Note: Refer to Procedure for disabling the SPI on page 1761 if the CRCEN bit is set.

		Bit 12 FRXTH: FIFO reception threshold
			This bit is used to set the threshold of the RXFIFO that triggers an RXNE event
				0: RXNE event is generated if the FIFO level is greater than or equal to 1/2 (16-bit)
				1: RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)


		Bits 11:8 DS[3:0]: Data size
			These bits configure the data length for SPI transfers.
				0111: 8-bit
				1111: 16-bit

				If software attempts to write one of the “Not used” values, they are forced to the value “0111”
				(8-bit)

		Bit 7 TXEIE: Tx buffer empty interrupt enable
				0: TXE interrupt masked
				1: TXE interrupt not masked. Used to generate an interrupt request when the TXE flag is set.
		Bit 6 RXNEIE: RX buffer not empty interrupt enable
				0: RXNE interrupt masked
				1: RXNE interrupt not masked. Used to generate an interrupt request when the RXNE flag isset.
		Bit 5 ERRIE: Error interrupt enable
			This bit controls the generation of an interrupt when an error condition occurs (CRCERR,
			OVR, MODF in SPI mode, FRE at TI mode and UDR, OVR, and FRE in I2S mode).
				0: Error interrupt is masked
				1: Error interrupt is enabled
		Bit 4 FRF: Frame format
				0: SPI Motorola mode
				1 SPI TI mode
		Note: This bit must be written only when the SPI is disabled (SPE=0).

		Bit 3 NSSP: NSS pulse management
			This bit is used in master mode only. it allows the SPI to generate an NSS pulse between two
			consecutive data when doing continuous transfers. In the case of a single data transfer, it
			forces the NSS pin high level after the transfer.
			It has no meaning if CPHA = ’1’, or FRF = ’1’.
				0: No NSS pulse
				1: NSS pulse generated
		Note: 1. This bit must be written only when the SPI is disabled (SPE=0).
		2. This bit is not used in I2S mode and SPI TI mode.

		Bit 2 SSOE: SS output enable
				0: SS output is disabled in master mode and the SPI interface can work in multimasterconfiguration
				1: SS output is enabled in master mode and when the SPI interface is enabled. The SPI
				interface cannot work in a multimaster environment.
Note: This bit is not used in I2S mode and SPI TI mode.
		Bit 1 TXDMAEN: Tx buffer DMA enable
				When this bit is set, a DMA request is generated whenever the TXE flag is set.
				0: Tx buffer DMA disabled
				1: Tx buffer DMA enabled
				Bit 0 RXDMAEN: Rx buffer DMA enable
		When this bit is set, a DMA request is generated whenever the RXNE flag is set.
				0: Rx buffer DMA disabled
				1: Rx buffer DMA enabled
 */


	SPI2->CR2 |=(0b1111 << 8);

/*
	SPI status register (SPIx_SR)
	Address offset: 0x08
	Reset value: 0x0002

	Біти 12:11 FTLVL[1:0]: Рівень передачі FIFO Ці біти встановлюються та скидаються апаратно.
		00: FIFO порожній 01: 1/4 FIFO  10: 1/2 FIFO 11: FIFO повний (вважається повним, коли поріг FIFO більше 1/2)

	Біти 10:9 FRLVL[1:0]: Рівень прийому FIFO Ці біти встановлюються та скидаються апаратно.
		00: FIFO порожній 01: 1/4 FIFO 	10: 1/2 FIFO 	11: FIFO повний
	.
	Біт 7 BSY: Флаг зайнятості
		0: SPI (або I2S) не зайнятий
		1: SPI (або I2S) зайнятий у процесі передачі або буфер Tx не порожній
		Цей флаг встановлюється та скидається апаратно.
		Примітка: Флаг BSY слід використовувати з обережністю: дивіться розділ 39.5.10:
		Флаги стану SPI та процедуру відключення SPI на сторінці 1761.

	Біт 6 OVR: Флаг переповнення
		0: Переповнення не сталося
		1: Виникло переповнення
		Цей флаг встановлюється апаратно і скидається програмною послідовністю.
	Біт 5 MODF: Помилка режиму
		0: Помилки режиму не сталося 1: Виникла помилка режиму
		Цей флаг встановлюється апаратно і скидається програмною послідовністю.
	Біт 4 CRCERR: Флаг помилки CRC
		0: Отримане значення CRC збігається зі значенням SPIx_RXCRCR
		1: Отримане значення CRC не збігає зі значенням SPIx_RXCRCR
	Біт 1 TXE: Буфер передачі порожній
		0: Буфер Tx не порожній
		1: Буфер Tx порожній
	Біт 0 RXNE: Буфер прийому не порожній
		0: Буфер Rx порожній
		1: Буфер Rx не порожній
*/


	SPI2->CR1 |= (1 << 6);

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


