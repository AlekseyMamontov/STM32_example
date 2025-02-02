/*
 * SPI.h
 *
 *  Created on: Feb 1, 2025
 *      Author: oleksii
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#define SPI2_CS_on  GPIOA->BRR = 1<<10;
#define SPI2_CS_off GPIOA->BSRR = 1<<10;

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

///////////////////////////////////////////////

uint8_t SPI2_data(uint8_t reg,uint8_t data ){

	SPI2_CS_on
	while(!(SPI2->SR & SPI_SR_TXE));
	SPI2->DR =(reg << 8) | data ;
	while(!(SPI2->SR & SPI_SR_RXNE));
	SPI2_CS_off

	return SPI2->DR;
};

///////////////////////////////////////////////

#define SPI_TIMEOUT 1000

uint8_t SPI2_Send_Data(uint8_t reg,uint8_t *data, uint16_t length) {

	uint8_t error =1;
	uint32_t timeout = 0;

	SPI2_CS_on

    while (!(SPI2->SR & SPI_SR_TXE)) {
        if (++timeout > SPI_TIMEOUT) goto spi_exit;}

        SPI2->DR = reg;


    for (size_t i = 0; i < length; i++) {

       timeout = 0;
       while (!(SPI2->SR & SPI_SR_TXE)){
            if (++timeout > SPI_TIMEOUT) goto spi_exit;}

        SPI2->DR = data[i];

        timeout = 0;
        while (!(SPI2->SR & SPI_SR_RXNE)){
            if (++timeout > SPI_TIMEOUT) goto spi_exit;}

        (void)SPI2->DR;  // Чтение данных из регистра для сброса флага RXNE
    }

    error = 0;
spi_exit: SPI2_CS_off;  // Деактивировать устройство
    return error;  // Успешная передача
}

////////////////////////////////////////////////////////////////////

uint8_t SPI2_Read_Data(uint8_t reg,uint8_t *data, uint16_t length) {

	uint8_t error =1;
	uint32_t timeout = 0;

	SPI2_CS_on

    while (!(SPI2->SR & SPI_SR_TXE)) {
        if (++timeout > SPI_TIMEOUT) goto spi_exit;}

       SPI2->DR = reg;  // Отправка адреса регистра


    for (size_t i = 0; i < length; i++) {

       timeout = 0;
       while (!(SPI2->SR & SPI_SR_TXE)){
            if (++timeout > SPI_TIMEOUT) goto spi_exit;}

        SPI2->DR = 0;  // Отправка байта

        timeout = 0;
        while (!(SPI2->SR & SPI_SR_RXNE)){
            if (++timeout > SPI_TIMEOUT) goto spi_exit;}

        data[i] = SPI2->DR;  // Чтение данных из регистра для сброса флага RXNE
    }

error = 0;
spi_exit: SPI2_CS_off;  // Деактивировать устройство
return error;  // Успешная передача
}






#endif /* INC_SPI_H_ */
