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


#define OPERATION_MODE_LIS3MXX 0x01
#define CONFIG_MODE_LIS3MXX    0x02
#define DISABLED_LIS3MXX       0x80



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

//		(INT_CFG   << 8)| 0, // def 11101000
//		(INT_THS_L << 8)| 0, // def 00000000
//		(INT_THS_H << 8)| 0, // def 00000000

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
		(LIS3M_INT_SRC | READ_REG_LIS3M)<< 8 | 0,

};

struct data_magnit{

		int* 	 mag;
		uint8_t*  status;


		uint8_t*  raw_fifo_buffer;
		uint16_t* reg_config;
		uint16_t* reg_status;
		uint16_t* reg_raw_mag;

		uint16_t  n_raw_fifo_buf;
		uint16_t  n_reg_config;
		uint16_t  n_reg_status;
		uint16_t  n_reg_raw_mag;


};

int 	mag_data_lis3md [3];
uint8_t lis3mdl_buffer[40],
	    lis3md_status;

struct data_magnit mag_lis3md ={

		.mag = 	mag_data_lis3md,
		.status = &lis3md_status,

		.raw_fifo_buffer = lis3mdl_buffer,
		.reg_config = magnit_config_registr,
		.reg_status = NULL,
		.reg_raw_mag = read_raw_data_magnit,

		.n_raw_fifo_buf =sizeof(lis3mdl_buffer),
		.n_reg_config =  sizeof(magnit_config_registr)/2,
		.n_reg_status =0,
		.n_reg_raw_mag = sizeof(read_raw_data_magnit)/2,

};

uint8_t init_lis3md (struct data_magnit* mag){


	uint8_t data;
	*(mag->status) = DISABLED_LIS3MXX | CONFIG_MODE_LIS3MXX;

	data = SPI1_reg_data((LIS3M_WHO_AM_I|READ_REG_LIS3M), 0x00);
	if(data != LIS3MDL) return 1;

	if(SPI1_WR_reg16_check(mag->reg_config,0,mag->n_reg_config)) return 2;

	*(mag->status) &=~DISABLED_LIS3MXX;// Ok

return 0;
};

uint8_t load_mag_lis3md(struct data_magnit* mag){

	if(mag->n_raw_fifo_buf < mag->n_reg_raw_mag) return 3;
	return  SPI1_array16to8_check(mag->reg_raw_mag,mag->raw_fifo_buffer,mag->n_reg_raw_mag);
};
uint8_t load_mag_lis3mdtr(struct data_magnit* mag){

	if(mag->n_raw_fifo_buf < mag->n_reg_raw_mag) return 3;
	SPI1_read_reg_to_array8_check(((LIS3M_OUT_X_L|INC_REG_LIS3M |READ_REG_LIS3M)<<8|0),
											mag->raw_fifo_buffer,5);
	*(mag->raw_fifo_buffer+6) = SPI1_data((LIS3M_TEMP_OUT_LR|READ_REG_LIS3M)<<8|0x00);
	*(mag->raw_fifo_buffer+7) = SPI1_data((LIS3M_TEMP_OUT_HR|READ_REG_LIS3M)<<8|0x00);
	return 0;
};











#endif /* INC_LIS3MD_H_ */
