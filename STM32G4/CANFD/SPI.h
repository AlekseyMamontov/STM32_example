/*
 * SPI.h
 *
 *  Created on: Feb 1, 2025
 *      Author: oleksii
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_


#define SPI1_CS_on   GPIOB->BRR =  1;
#define SPI1_CS_off  GPIOB->BSRR = 1;

#define SPI2_disable SPI2->CR1 &=0xFFBF;
#define SPI2_enable  SPI2->CR1 |=0x0040;
#define SPI2_set_DMA SPI2->CR2 |=0x03;
#define SPI2_clr_DMA SPI2->CR2 &=0xFFFC;


void Init_SPI_STM32(void) {

	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
	RCC->APB2ENR  |= RCC_APB2ENR_SPI1EN;
	/*
	 SPI control register 1 (SPIx_CR1) Address offset: 0x00 Reset value: 0x0000

	 [15] BIDIMODE: Bidirectional data mode enable.
	 This bit enables half-duplex communication using common single bidirectional data line.
	 Keep RXONLY bit clear when bidirectional mode is active.
	 *	0: 2-line unidirectional data mode selected
	 *	1: 1-line bidirectional data mode selected
	 N
	 [14] BIDIOE: Output enable in bidirectional mode
	 This bit combined with the BIDIMODE bit selects the direction of transfer in bidirectional mode.
	 * 	0: Output disabled (receive-only mode)
	 *	1: Output enabled (transmit-only mode)
	 Note: In master mode, the MOSI pin is used and in slave mode, the MISO pin is used.

	 [13] CRCEN: Hardware CRC calculation enable
	 *	0: CRC calculation disabled
	 *	1: CRC calculation enabled
	 Note: This bit should be written only when SPI is disabled (SPE = ‘0’) for correct operation.

	 [12] CRCNEXT: Transmit CRC next
	 *	0: Next transmit value is from Tx buffer.
	 *	1: Next transmit value is from Tx CRC register.
	 Note: This bit has to be written as soon as the last data is written in the SPIx_DR register.

	 [11] CRCL: CRC length
	 This bit is set and cleared by software to select the CRC length.
	 *	0: 8-bit CRC length
	 *	1: 16-bit CRC length
	 Note: This bit should be written only when SPI is disabled (SPE = ‘0’) for correct operation.

	 [10] RXONLY: Receive only mode enabled.
	 This bit enables simplex communication using a single unidirectional line to receive data
	 exclusively. Keep BIDIMODE bit clear when receive only mode is active.This bit is also
	 useful in a multislave system in which this particular slave is not accessed, the output from
	 the accessed slave is not corrupted.
	 *	0: Full-duplex (Transmit and receive)
	 *	1: Output disabled (Receive-only mode)

	 [9] SSM: Software slave management
	 When the SSM bit is set, the NSS pin input is replaced with the value from the SSI bit.
	 *	0: Software slave management disabled
	 *	1: Software slave management enabled

	 [8] SSI: Internal slave select
	 This bit has an effect only when the SSM bit is set. The value of this bit is forced onto the
	 NSS pin and the I/O value of the NSS pin is ignored.
	 Note: This bit is not used in I2S mode and SPI TI mode.

	 [7] LSBFIRST: Frame format
	 *	0: data is transmitted / received with the MSB first
	 *	1: data is transmitted / received with the LSB first
	 *
	 Note: 1. This bit should not be changed when communication is ongoing.
	 2. This bit is not used in I2S mode and SPI TI mode.

	 [6] SPE: SPI enable
	 *	0: Peripheral disabled
	 *	1: Peripheral enabled
	 Note: When disabling the SPI, follow the procedure described in Procedure for disabling the
	 SPI on page 1761.
	 [5:3] BR[2:0]: Baud rate control
	 *	000: fPCLK/2
	 *	001: fPCLK/4
	 *	010: fPCLK/8
	 *	011: fPCLK/16
	 *	100: fPCLK/32
	 *	101: fPCLK/64
	 *	110: fPCLK/128
	 *	111: fPCLK/256

	 [2] MSTR: Master selection
	 *	0: Slave configuration
	 *	1: Master configuration
	 [1] CPOL: Clock polarity
		0: CK to 0 when idle
		1: CK to 1 when idle

	[0] CPHA: Clock phase
		0: The first clock transition is the first data capture edge
		1: The second clock transition is the first data capture edge
	 */

	SPI2->CR1 = (1 << 2) | (2 << 3) | (3 << 8) | 0; // master | clk/8 |  SSM=1 SSI =1 CPOL0 _/
	SPI1->CR1 = (1 << 2) | (6 << 3) | (3 << 8) | 0; // master | clk/64 |  SSM=1 SSI =1 CPOL0 _/
	/*
	 SPI control register 2 (SPIx_CR2) Address offset: 0x04 Reset value: 0x0700 (8 bit)
	 Регістровий опис SPI_CR2
	 Адреса відступу: 0x04
	 Початкове значення: 0x0700 (8 біт)

	 [14] LDMA_TX: Остання передача DMA для передачі
	 Цей біт використовується в режимі пакування даних, щоб визначити, чи загальна кількість даних для передачі через DMA
	 є парною 		чи непарною. Має значення лише якщо біт TXDMAEN в регістрі SPI_CR2 встановлений, і якщо використовується
	 режим пакування (довжина даних ≤ 8 біт і запис у SPI_DR шириною 16 біт). Його потрібно записати, коли SPI вимкнено
	 (SPE = 0 в регістрі SPI_CR1).
	 *	0: Кількість даних для передачі парна
	 *	1: Кількість даних для передачі непарна
	 Примітка: Дивіться процедуру вимкнення SPI на сторінці 1761, якщо біт CRCEN встановлений.

	 [13]LDMA_RX: Остання передача DMA для прийому
	 Цей біт використовується в режимі пакування даних, щоб визначити, чи загальна кількість даних для прийому ч
	 ерез DMA є парною чи непарною. Має значення лише якщо біт RXDMAEN в регістрі SPI_CR2 встановлений, і якщо
	 використовується режим пакування (довжина даних ≤ 8 біт і запис у SPI_DR шириною 16 біт).
	 Його потрібно записати, коли SPI вимкнено (SPE = 0 в регістрі SPI_CR1).
	 *	0: Кількість даних для прийому парна
	 *	1: Кількість даних для прийому непарна
	 Примітка: Дивіться процедуру вимкнення SPI на сторінці 1761, якщо біт CRCEN встановлений.

	 [12] FRXTH: Поріг приймання FIFO
	 Цей біт використовується для встановлення порогу RXFIFO, який викликає подію RXNE.
	 0: Подія RXNE генерується, якщо рівень FIFO більший або рівний 1/2 (16 біт)
	 1: Подія RXNE генерується, якщо рівень FIFO більший або рівний 1/4 (8 біт)

	 [11:8] DS[3:0]: Розмір даних
	 Ці біти конфігурують довжину даних для передач через SPI.
	 *	0111: 8 біт
	 *	1111: 16 біт
	 Якщо програмне забезпечення намагається записати одне з "Не використовується" значень,
	 вони фіксуються на значенні "0111" (8 біт).

	 [7] TXEIE: Увімкнення переривання для порожнього буфера TX
	 *	0: Переривання TXE заблоковано
	 *	1: Переривання TXE не заблоковано. Використовується для генерації запиту на переривання, коли прапорець TXE встановлений.
	 [6] RXNEIE: Увімкнення переривання для непорожнього буфера RX
	 *	0: Переривання RXNE заблоковано
	 *	1: Переривання RXNE не заблоковано. Використовується для генерації запиту на переривання, коли прапорець RXNE встановлений.
	 [5] ERRIE: Увімкнення переривання для помилок
	 Цей біт контролює генерацію переривання, коли виникає умова помилки (CRCERR, OVR, MODF в режимі SPI, FRE в режимі TI
	 та UDR, OVR і FRE в режимі I2S).
	 *	0: Переривання помилки заблоковано
	 *	1: Переривання помилки увімкнено
	 [4] FRF: Формат кадру
	 0: Режим Motorola SPI
	 1: Режим TI SPI
	 Примітка: Цей біт потрібно записувати лише коли SPI вимкнено (SPE=0).
	 [3] NSSP: Керування імпульсом NSS
	 Цей біт використовується лише в майстер-режимі. Дозволяє SPI генерувати імпульс NSS між двома послідовними даними
	 під час безперервних передач. У випадку одиничної передачі даних, він примусово підвищує рівень NSS після передачі.
	 Не має значення, якщо CPHA = '1', або FRF = '1'.
	 *	0: Імпульс NSS не генерується
	 *	1: Імпульс NSS генерується
	 Примітка: 1. Цей біт потрібно записувати лише коли SPI вимкнено (SPE=0).
	 Цей біт не використовується в режимі I2S і TI SPI.

	 [2] SSOE: Увімкнення виходу SS
	 *	0: Вихід SS вимкнено в майстер-режимі, і SPI інтерфейс може працювати в мультимайстерній конфігурації
	 *	1: Вихід SS увімкнено в майстер-режимі, і коли SPI інтерфейс увімкнено.
	 *	   SPI інтерфейс не може працювати в мультимайстерному середовищі.
	 Примітка: Цей біт не використовується в режимі I2S і TI SPI.

	 [1] TXDMAEN: Увімкнення DMA для буфера TX
	 Коли цей біт встановлений, генерується запит DMA щоразу, коли встановлюється прапорець TXE.
	 *	0: DMA для буфера TX вимкнено
	 *	1: DMA для буфера TX увімкнено
	 *
	 [0] RXDMAEN: Увімкнення DMA для буфера RX
	 Коли цей біт встановлений, генерується запит DMA щоразу, коли встановлюється прапорець RXNE.
	 *	0: DMA для буфера RX вимкнено
	 *	1: DMA для буфера RX увімкнено
	 */

	SPI2->CR2 |= (0b1111 << 8) | 0; // 16 BIT /
	SPI1->CR2 |= (0b1111 << 8) | 0; // 16 BIT /

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

	//SPI2->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;


	SPI2->CR1 |= (1 << 6);// EN SP2
	SPI1->CR1 |= (1 << 6);// EN SP1
}


#define SPI_TIMEOUT 50000

///////////// spi 8bit

uint8_t SPI_data8(SPI_TypeDef * spi, uint8_t data){

	uint32_t timeout = 0;

	while (!(spi->SR & SPI_SR_TXE)){
		if (++timeout > SPI_TIMEOUT)return 0;};

	spi->DR = data;

	timeout = 0;

	while (!(spi->SR & SPI_SR_RXNE)){
		if (++timeout > SPI_TIMEOUT)return 0;};

	return spi->DR;
};

///////////// spi 16bit

uint16_t SPI_data16(SPI_TypeDef * spi, uint16_t data){

	uint32_t timeout = 0;

	while (!(spi->SR & SPI_SR_TXE)){
		if (++timeout > SPI_TIMEOUT)return 0;};

	spi->DR = data;

	timeout = 0;

	while (!(spi->SR & SPI_SR_RXNE)){
		if (++timeout > SPI_TIMEOUT)return 0;};

	return spi->DR;
};



///////////  SPI 16bit -> data 8bit , for sensor data (reg (MSB) | data (LSB))

uint8_t SPI_sensor_reg_data(SPI_TypeDef * spi,uint16_t data){

	uint32_t timeout = 0;

	while (!(spi->SR & SPI_SR_TXE)){
		if (++timeout > SPI_TIMEOUT)return 0;};

	spi->DR = data;

	timeout = 0;
	while (!(spi->SR & SPI_SR_RXNE)){
		if (++timeout > SPI_TIMEOUT)return 0;};

	return spi->DR;
};

///////////// SPI1 reg + data


uint8_t SPI_reg_data(SPI_TypeDef * spi,uint8_t reg, uint8_t data){
	return SPI_sensor_reg_data(spi,(reg<<8)|data);
}


///////////////  SPI 16 bit to RAM 8bit

uint8_t SPI_sensor_reg_data_check(SPI_TypeDef * spi, uint16_t reg, uint8_t *data) {

	uint32_t timeout = 0;

		while (!(spi->SR & SPI_SR_TXE)){
			if (++timeout > SPI_TIMEOUT)return 1;};


		spi->DR = reg;

		timeout = 0;
		while (!(spi->SR & SPI_SR_RXNE)){
			if (++timeout > SPI_TIMEOUT)return 2;};


		*data = spi->DR;

	return 0;
};


///////////////////

uint8_t SPI_reg_data_check(SPI_TypeDef * spi,uint8_t reg, uint8_t *data){
	return SPI_sensor_reg_data_check (spi,(reg<<8)|(*data),data);
};


////////////////// 16 bit  (Reg|Data) -> data (8bit)

uint8_t SPI_sensor_regdata16_to_data8_check(SPI_TypeDef * spi,uint16_t* reg, uint8_t *data, uint16_t len) {


	uint32_t timeout = 0;

	while (len--){

		timeout = 0;
		while (!(spi->SR & SPI_SR_TXE)){
			if (++timeout > SPI_TIMEOUT)return 1;}

		spi->DR = *reg++;

		timeout = 0;
		while (!(spi->SR & SPI_SR_RXNE)){
			if (++timeout > SPI_TIMEOUT)return 2;}

		*data++ = (uint8_t)(spi->DR & 0xFF);
	};

	return 0;
};

///////////////// 16 bit  RAM 16bit > 16bit

uint8_t SPI_array16to16_check(SPI_TypeDef * spi,uint16_t* reg, uint16_t *data, uint16_t len) {

	uint32_t timeout;

	while (len--){

		timeout = 0;
		while (!(spi->SR & SPI_SR_TXE)){
			if (++timeout > SPI_TIMEOUT)return 1;}

		spi->DR = *reg++;

		timeout = 0;
		while (!(spi->SR & SPI_SR_RXNE)){
			if (++timeout > SPI_TIMEOUT)return 2;};

		*data++ = spi->DR;
	};


	return 0;
};


/////////////////////////// specific sensor  LIS3M,IMU 42xxx /////////////////////////////////////
// LSB MSB
uint8_t SPI_read_reg_to_array8_check(SPI_TypeDef * spi,uint16_t reg, uint8_t *data, uint16_t len) {

	if(!len) return 0;

	uint32_t timeout = 0;

////////  registr

	while (!(spi->SR & SPI_SR_TXE)){if (++timeout > SPI_TIMEOUT)return 1;}
	spi->DR = reg;
	timeout = 0;

//////// data

	while (!(spi->SR & SPI_SR_RXNE)){if (++timeout > SPI_TIMEOUT)return 2;}
	*data++ = spi->DR & 0xFF;
	len--;

	while (len--){

		timeout = 0;

		while (!(spi->SR & SPI_SR_TXE)){if (++timeout > SPI_TIMEOUT)return 1;}

		spi->DR = 0;
		timeout = 0;

		while (!(spi->SR & SPI_SR_RXNE)){if (++timeout > SPI_TIMEOUT)return 2;}

		timeout = spi->DR;
		*data++ = timeout >> 8 & 0xFF;
		*data++ = timeout & 0xFF;

	};

	return 0;
};

/*  uint16_t block[] ={
 *  ((reg<<8) | dir | data),
 *  ((reg<<8) | dir | data),
 *  ((banksel)| dir | N),
 *  ((reg<<8) | dir | data),
 *  ,,, }

*/

uint8_t SPI_WR_reg16_check(SPI_TypeDef * spi,uint16_t* reg,uint16_t dir) {

	uint16_t timeout = 0;

		while (!(spi->SR & SPI_SR_TXE)){if (++timeout > SPI_TIMEOUT)return 1;}

		spi->DR = (*reg) | dir;

		timeout = 0;
		while (!(spi->SR & SPI_SR_RXNE)){if (++timeout > SPI_TIMEOUT)return 2;}

		timeout = spi->DR;
		if(dir) *reg = ((*reg)&0xff00)|(timeout&0x00ff);// read

	return 0;
};







// Обробник переривання для SPI
void SPI2_IRQHandler(void){};

/*old code
 *
 /////////////////////////////  DMA ////////////////////////////

#define SPI2_DMA_enable   SPI2->CR2 |=  3; // 16 BIT
#define SPI2_DMA_disabled SPI2->CR2 |= ~3;

uint8_t SPI2dmaComplete;

void ReadSPI2_DMA(uint8_t startReadCommand,uint16_t* ram,uint16_t* ram2,uint16_t size) {

	SPI2dmaComplete=0;

    ram[0] = (uint16_t)startReadCommand << 8; // 0xAD00

    SPI2_CS_on

    //SPI2_DMA_enable

    DMA1_Channel2->CNDTR = size;
    DMA1_Channel2->CPAR = (uint32_t)&(SPI2->DR);
    DMA1_Channel2->CMAR = (uint32_t)ram;
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    DMA1_Channel2->CNDTR = size;
    DMA1_Channel2->CPAR = (uint32_t)&(SPI2->DR);
    DMA1_Channel2->CMAR = (uint32_t)ram2;
    DMA1_Channel2->CCR |= DMA_CCR_EN;

   while (!SPI2dmaComplete){};

    SPI2_DMA_disabled
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    SPI2_CS_off



}
void DMA1_Channelxx_IRQHandler(void) {

    if (DMA1->ISR & DMA_ISR_TCIF1) {

        DMA1->IFCR = DMA_IFCR_CTCIF1;

        //DMA1_Channel1->CCR &= ~DMA_CCR_EN;

         DMA1_Channel2->CCR &= ~DMA_CCR_EN;
         SPI2_CS_off
         SPI2dmaComplete =1;

  }

  DMA1->IFCR |= DMA_IFCR_CGIF1;
}
 *
 *
 *
 * */


#endif /* INC_SPI_H_ */
