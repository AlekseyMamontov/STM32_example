/*
 * LIS3MD.h
 *
Список регістрів LIS3MDL

1. OFFSET_X_REG_L_M (05h) Нижній байт жорсткого зсуву по осі X (двійковий код, 16-бітний).
2. OFFSET_X_REG_H_M (06h) Старший байт жорсткого зсуву по осі X.
3. OFFSET_Y_REG_L_M (07h) Нижній байт жорсткого зсуву по осі Y.
4. OFFSET_Y_REG_H_M (08h) Старший байт жорсткого зсуву по осі Y.
5. OFFSET_Z_REG_L_M (09h) Нижній байт жорсткого зсуву по осі Z.
6. OFFSET_Z_REG_H_M (0Ah) Старший байт жорсткого зсуву по осі Z.

7. WHO_AM_I (0Fh) 		  Ідентифікаційний код пристрою (значення за замовчуванням: 0x3D).

8. CTRL_REG1 (20h)

	[7] TEMP_EN (0: вимкнено, 1: вмикається датчик температури).
	[6:5] OM[1:0] (режими роботи для осей X та Y).
			00 - Low-power mode
			01 - Medium-performance mode
			10 - High-performance mode
			11 - Ultrahigh-performance mode
	[4:2] DO[2:0] (частота оновлення даних).
				   [ODR hz]
				000 0.625
				001 1.25
				010	2.5
				011	5
				100	10
				101	20
				110	40
				111	80

	[1]   FAST_ODR (0: вимкнено, 1: увімкнено).
					[OM]
			 1  1000 LP
			 1  560  MP
			 1	300  HP
			 1	155	 UHP

	[0]   ST (0: вимкнено, 1: увімкнено самоперевірку).

9. CTRL_REG2 (21h)  [0xx0xx00]

	[6:5] FS[1:0]  (вибір діапазону вимірювань: 00: ±4 гаусс, 01: ±8 гаусс, 10: ±12 гаусс, 11: ±16 гаусс).
	[3]   REBOOT   (0: нормальний режим, 1: перезавантаження).
	[2]   SOFT_RST (0: нормальна робота, 1: скидання конфігурацій).


10. CTRL_REG3 (22h) [00x00xxx]

	[5]   LP (0: нормальний режим, 1: режим низького споживання).
	[2]   SIM (0: 4-провідний SPI, 1: 3-провідний SPI).
	[0:1] MD[1:0] режим роботи:
		00: безперервний,
		01: одноразовий,
		10/11: режим вимкнення.

11. CTRL_REG4 (23h) [0000xxx0]

	[3:2] OMZ[1:0] (режими роботи для осі Z).
			00 	Low-power mode
			01	Medium-performance mode
			10	High-performance mode
			11	Ultrahigh-performance mode
	[1]   BLE (0: LSB, 1: MSB).

12. CTRL_REG5 (24h) [xx000000]

	[7] FAST_READ (0: вимкнено, 1: увімкнено).
		FAST READ дозволяє читати лише верхню частину DATA OUT для підвищення ефективності читання.
		Значення за замовчуванням для FAST_READ: 0

	[6] BDU (0: постійне оновлення,
	         1: блокувати оновлення до зчитування).

13. STATUS_REG (27h)

	[7] ZYXOR (переповнення даних по всіх осях).
	[6] ZOR (переповнення даних по осі Z).
	[5] YOR (переповнення даних по осі Y).
	[4] XOR (переповнення даних по осі X).
	[3] ZYXDA (нові дані по всіх осях доступні).
	[2] ZDA (нові дані по осі Z доступні).
	[1] YDA (нові дані по осі Y доступні).
	[0] XDA (нові дані по осі X доступні).

14. OUT_X_L (28h) Нижній байт даних по осі X (двійковий код).
15. OUT_X_H (29h) Старший байт даних по осі X.
16. OUT_Y_L (2Ah) Нижній байт даних по осі Y.
17. OUT_Y_H (2Bh) Старший байт даних по осі Y.
18. OUT_Z_L (2Ch) Нижній байт даних по осі Z.
19. OUT_Z_H (2Dh) Старший байт даних по осі Z.

20. TEMP_OUT_L (2Eh)  Нижній байт даних з температурного датчика.
21. TEMP_OUT_H (2Fh) Старший байт даних з температурного датчика.

22. INT_CFG (30h) [xxx01xxx]

	[7] XIEN (увімкнення переривання по осі X).
	[6] YIEN (увімкнення переривання по осі Y).
	[5] ZIEN (увімкнення переривання по осі Z).
	[2] IEA (активна конфігурація переривання).
	[1] LIR (запам'ятовування запиту на переривання).
	[0] IEN (увімкнення переривання).

23. INT_SRC (31h)

	[7] PTH_X (перевищення порога по позитивній стороні по осі X).
	[6] PTH_Y (перевищення порога по позитивній стороні по осі Y).
	[5] PTH_Z (перевищення порога по позитивній стороні по осі Z).
	[4] NTH_X (перевищення порога по негативній стороні по осі X).
	[3] NTH_Y (перевищення порога по негативній стороні по осі Y).
	[2] NTH_Z (перевищення порога по негативній стороні по осі Z).
	[1] MROI (переповнення внутрішнього діапазону вимірювання).
	[0] INT (сигналізація події переривання).

24. INT_THS_L (32h) [xxxxxxxx] Нижній байт порога переривання (значення в 16-бітному беззнаковому форматі).
25. INT_THS_H (33h) [0xxxxxxx] Старший байт порога переривання.

 */

#ifndef INC_LIS3MD_H_
#define INC_LIS3MD_H_

#define LIS3M_CS_on   GPIOB->BRR =  1;
#define LIS3M_CS_off  GPIOB->BSRR = 1;

#define LIS3M_SPI     SPI1
#define DMA_SPIenable_LIS3M  LIS3M_SPI->CR2 |=0x03
#define DMA_SPIdisable_LIS3M LIS3M_SPI->CR2 &=0xFFFC

#define DMArx_LIS3M     DMA1_Channel3
#define DMAtx_LIS3M	    DMA1_Channel4
#define DMAMUXrx_LIS3M  DMAMUX1_Channel2
#define DMAMUXtx_LIS3M  DMAMUX1_Channel3

#define DMAMUXrx_id_device_LIS3M 10    // STM32G4  SPI1 10(tab 91)
#define DMAMUXtx_id_device_LIS3M 11    // ---

#define InitRCC_DMA_LIS3M  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
#define IRQ_DMArx_LIS3M    DMA1_Channel3_IRQn
#define IRQ_pin_LIS3M      EXTI1_IRQn  //INT_fifo_ready EN



#define LIS3M_WHO_AM_I  0x0F
#define LIS3MDL 		0x3D

#define LIS3M_CTRL_REG1 0x20
#define LIS3M_CTRL_REG2 0x21
#define LIS3M_CTRL_REG3 0x22
#define LIS3M_CTRL_REG4 0x23
#define LIS3M_CTRL_REG5 0x24



#define LIS3M_OFFSET_X_REG_L_M 0x05
#define LIS3M_OFFSET_X_REG_H_M 0x06
#define LIS3M_OFFSET_Y_REG_L_M 0x07
#define LIS3M_OFFSET_Y_REG_H_M 0x08
#define LIS3M_OFFSET_Z_REG_L_M 0x09
#define LIS3M_OFFSET_Z_REG_H_M 0x0A

#define LIS3M_STATUS_REG 0x27

#define LIS3M_OUT_X_L 0x28
#define LIS3M_OUT_X_H 0x29
#define LIS3M_OUT_Y_L 0x2A
#define LIS3M_OUT_Y_H 0x2B
#define LIS3M_OUT_Z_L 0x2C
#define LIS3M_OUT_Z_H 0x2D
#define LIS3M_TEMP_OUT_LR	0x2E
#define LIS3M_TEMP_OUT_HR   0x2F

#define LIS3M_INT_CFG 	0x30
#define LIS3M_INT_SRC	0x31

#define LIS3M_INT_THS_L  0x32
#define LIS3M_INT_THS_H  0x32

#define READ_REG_LIS3M   0x80
#define INC_REG_LIS3M    0x40
#define WRITE_REG_LIS3M  0x00

#define OPERATION_MODE_LIS3MXX 0x01
#define CONFIG_MODE_LIS3MXX    0x02
#define DISABLED_LIS3MXX       0x80
#define INT_FIFO_LIS3MXX       0x40
#define DMA_OK_LIS3MXX         0x20


///////// Data

int 	mag_data_lis3md [3];
uint8_t lis3mdl_buffer[40],
	    lis3md_status =0,
	    lis3md_dma_packet = 4;

uint16_t lis3mdl_bufferRX[5]={0},
		 lis3mdl_bufferTX[5]={0};

struct data_magnit{

		int* 	  mag;
		uint8_t*  status;

		uint16_t* reg_config;
		uint16_t* reg_status;
		uint16_t* reg_raw_mag;

		uint8_t *  raw_fifo_buffer;
		uint16_t*  DMA_RX_fifo_buf;
		uint16_t*  DMA_TX_fifo_buf;
		uint8_t *  n_16bit_packet_fifo;


		uint16_t  n_raw_fifo_buffer;
		uint16_t  n_reg_config;
		uint16_t  n_reg_status;
		uint16_t  n_reg_raw_mag;

};


uint16_t magnit_config_registr[]={

//		(LIS3M_OFFSET_X_REG_L_M << 8)| 0,
//		(LIS3M_OFFSET_X_REG_H_M << 8)| 0,
//		(LIS3M_OFFSET_Y_REG_L_M << 8)| 0,
//		(LIS3M_OFFSET_Y_REG_H_M << 8)| 0,
//		(LIS3M_OFFSET_Z_REG_L_M << 8)| 0,
//		(LIS3M_OFFSET_Z_REG_H_M << 8)| 0,

		(LIS3M_CTRL_REG1 << 8)| 0b11011100,  //def 00010000
		(LIS3M_CTRL_REG2 << 8)| 0b00100000,  //def 00000000
		(LIS3M_CTRL_REG4 << 8)| 0b00001000,  //def 00000000
//		(LIS3M_CTRL_REG5 << 8)| 0,  //def 00000000

//		(LIS3M_INT_CFG   << 8)| 0b11101010, // def 11101000
//		(LIS3M_INT_THS_L << 8)| 0, // def 00000000
//		(LIS3M_INT_THS_H << 8)| 0, // def 00000000

		//enable
		(LIS3M_CTRL_REG3 << 8)| 0b00000000,  //def 00000011
};
uint16_t read_raw_data_magnit[]={


		(LIS3M_OUT_X_L | READ_REG_LIS3M)<< 8 | 0,
		(LIS3M_OUT_X_H | READ_REG_LIS3M)<< 8 | 0,
		(LIS3M_OUT_Y_L | READ_REG_LIS3M)<< 8 | 0,
		(LIS3M_OUT_Y_H | READ_REG_LIS3M)<< 8 | 0,
		(LIS3M_OUT_Z_L | READ_REG_LIS3M)<< 8 | 0,
		(LIS3M_OUT_Z_H | READ_REG_LIS3M)<< 8 | 0,

		(LIS3M_TEMP_OUT_LR| READ_REG_LIS3M)<< 8 | 0,
		(LIS3M_TEMP_OUT_HR| READ_REG_LIS3M)<< 8 | 0,

		(LIS3M_STATUS_REG | READ_REG_LIS3M)<< 8 | 0,
		(LIS3M_INT_SRC | READ_REG_LIS3M)<< 8    | 0,

};

struct data_magnit mag_lis3md ={

		.mag = 	mag_data_lis3md,
		.status = &lis3md_status,

		.raw_fifo_buffer = lis3mdl_buffer,
		.DMA_RX_fifo_buf = lis3mdl_bufferRX,
		.DMA_TX_fifo_buf = lis3mdl_bufferTX,
		.n_16bit_packet_fifo = &lis3md_dma_packet,
		.reg_config = magnit_config_registr,
		.reg_status = NULL,
		.reg_raw_mag = read_raw_data_magnit,

		.n_raw_fifo_buffer =sizeof(lis3mdl_buffer),
		.n_reg_config =  sizeof(magnit_config_registr)/2,
		.n_reg_status =0,
		.n_reg_raw_mag = sizeof(read_raw_data_magnit)/2,

};


uint8_t RW_REG_LIS3M(uint16_t* reg,uint16_t dir, uint16_t len) {

	uint8_t data = 0, error = 0;

	while (len--){

		LIS3M_CS_on
		error = SPI_sensor_reg_data_check(LIS3M_SPI,(*reg)|(dir),&data);
		LIS3M_CS_off
		if(error) break;
		if(dir) *reg = ((*reg)&0xff00)|data;// read 1, write 0
		reg++;
	};

	return error;
};
void DMA_init_LIS3M(struct data_magnit* mag) {

	InitRCC_DMA_LIS3M // RCC

    // SPI_RX  DIR = 0 (Peripheral-to-Memory)

    DMArx_LIS3M->CCR = DMA_CCR_MINC   |    // RAM++
                       DMA_CCR_TCIE    |   // IRQ_RXNE
                       DMA_CCR_PSIZE_0 |   // 16 bit
                       DMA_CCR_MSIZE_0 |   // 16 bit
                       DMA_CCR_CIRC;

	DMArx_LIS3M->CNDTR = *(mag->n_16bit_packet_fifo);               // n__16bit
    DMArx_LIS3M->CPAR = (uint32_t)&(LIS3M_SPI->DR); // SPI data
    DMArx_LIS3M->CMAR = (uint32_t)mag->DMA_RX_fifo_buf;
    DMAMUXrx_LIS3M->CCR = DMAMUXrx_id_device_LIS3M ;//SPI1_RX (Table 91)

    //SPI_TX  DIR=1 (Memory-to-Peripheral)

    DMAtx_LIS3M->CCR = DMA_CCR_MINC 	  |     // RAM++
                       DMA_CCR_DIR     |     // Memory-to-Peripheral
                       DMA_CCR_PSIZE_0 |     // 16-bit
                       DMA_CCR_MSIZE_0 ;	   // 16-bit

    DMAtx_LIS3M->CNDTR = *(mag->n_16bit_packet_fifo);  // Количество слов для передачи
    DMAtx_LIS3M->CPAR = (uint32_t)&(LIS3M_SPI->DR); // Адрес регистра SPI
    DMAtx_LIS3M->CMAR = (uint32_t)mag->DMA_TX_fifo_buf;// Адрес буфера передачи
    DMAMUXtx_LIS3M->CCR = DMAMUXtx_id_device_LIS3M ;// SPI1_TX (Table 91)

}

uint8_t init_lis3md (struct data_magnit* mag){

	uint8_t data = 0;
	*(mag->status) = DISABLED_LIS3MXX | CONFIG_MODE_LIS3MXX;

	//reboot
	LIS3M_CS_on
	data = SPI_reg_data(LIS3M_SPI,(LIS3M_CTRL_REG2|WRITE_REG_LIS3M), 0x0C);
	LIS3M_CS_off

	systick_pause = 5;//50ms
	while(systick_pause);

	LIS3M_CS_on
	data = SPI_reg_data(LIS3M_SPI,(LIS3M_WHO_AM_I|READ_REG_LIS3M), 0x00);
	LIS3M_CS_off
	if(data != LIS3MDL) return 1;

	// init block write reg

	data = RW_REG_LIS3M(mag->reg_config,WRITE_REG_LIS3M,mag->n_reg_config);
	if(data)return data;

	*(mag->status) &=~DISABLED_LIS3MXX;// Ok

	 DMA_init_LIS3M(mag);

	 DMA_SPIenable_LIS3M;			  //SPI dma 16bit
	 DMArx_LIS3M->CCR |= DMA_CCR_EN;
     NVIC_EnableIRQ(IRQ_DMArx_LIS3M); //RX_buffer_circle
     NVIC_EnableIRQ(IRQ_pin_LIS3M);  //INT_fifo_ready EN

     mag->DMA_TX_fifo_buf[0]= ((LIS3M_STATUS_REG|INC_REG_LIS3M |READ_REG_LIS3M)<<8|0);

     *(mag->status) &= ~CONFIG_MODE_LIS3MXX;
     *(mag->status) |= OPERATION_MODE_LIS3MXX;

	systick_pause = 2;//2ms
	while(systick_pause);

return 0;
};

///////// READ MAG

uint8_t load_mag_lis3md(struct data_magnit* mag){

	uint8_t  data, *mem = mag->raw_fifo_buffer;
	uint16_t *mem16 = mag->reg_raw_mag;

	if(mag->n_raw_fifo_buffer < mag->n_reg_raw_mag) return 3;

	for(uint8_t i=0;i < mag->n_reg_raw_mag;i++){

		LIS3M_CS_on;
		data = SPI_sensor_reg_data_check(LIS3M_SPI,*(mem16 + i),mem+i);
		if(data)break;
	    LIS3M_CS_off;

	}
return data;
};

///////////////

uint8_t load_mag_lis3mdtr(struct data_magnit* mag){

	uint8_t data;

	if(mag->n_raw_fifo_buffer < mag->n_reg_raw_mag) return 3;

	LIS3M_CS_on;
	data = SPI_read_reg_to_array8_check(
			LIS3M_SPI, ((LIS3M_OUT_X_L|INC_REG_LIS3M |READ_REG_LIS3M)<<8|0),mag->raw_fifo_buffer,5
			);
	LIS3M_CS_off;
	if(data)return data;

	LIS3M_CS_on;
	//SPI_sensor_reg_data_check(LIS3M_SPI,((LIS3M_TEMP_OUT_LR|READ_REG_LIS3M)<<8|0x00),&data);
	data = SPI_sensor_reg_data(LIS3M_SPI,((LIS3M_TEMP_OUT_LR|READ_REG_LIS3M)<<8|0x00));
	LIS3M_CS_off;
	*(mag->raw_fifo_buffer+6) = data;

	LIS3M_CS_on;
	*(mag->raw_fifo_buffer+7) = SPI_sensor_reg_data(LIS3M_SPI,((LIS3M_TEMP_OUT_HR|READ_REG_LIS3M)<<8|0x00));
	LIS3M_CS_off;

	return 0;
};











#endif /* INC_LIS3MD_H_ */
