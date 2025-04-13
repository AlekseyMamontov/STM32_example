/*
 * IIM_42652.h
 *
ЗМІНА ЗНАЧЕНЬ РЕГІСТРІВ
Єдині налаштування регістрів, які користувач може змінювати під час роботи сенсора, стосуються вибору ODR (частоти вибірки), вибору FSR (діапазону вимірювань) і зміни режимів роботи сенсора (параметри регістрів GYRO_ODR, ACCEL_ODR, GYRO_FS_SEL, ACCEL_FS_SEL, GYRO_MODE, ACCEL_MODE). Користувач не повинен змінювати інші значення регістрів під час роботи сенсора. Для зміни значень інших регістрів необхідно дотримуватись наступної процедури:

Вимкнути акселерометр і гіроскоп.
Змінити значення регістрів.
Увімкнути акселерометр і/або гіроскоп.

*/

//////// DEFINE

#ifndef INC_IIM_42652_H_
#define INC_IIM_42652_H_
#include <Ixm42xxxDefs.h>

#define IIM42XXX_CS_on   GPIOA->BRR =  1<<10;
#define IIM42XXX_CS_off  GPIOA->BSRR = 1<<10;
#define SPI_IIM42XXX     SPI2
#define DMA_SPIenable_IIM42xxx  SPI_IIM42XXX->CR2 |=0x03
#define DMA_SPIdisable_IIM42xxx SPI_IIM42XXX->CR2 &=0xFFFC

#define DMArx_IIM42XXX     DMA1_Channel1
#define DMAtx_IIM42XXX	   DMA1_Channel2
#define DMAMUXrx_IIM42XXX  DMAMUX1_Channel0
#define DMAMUXtx_IIM42XXX  DMAMUX1_Channel1
#define DMAMUXrx_id_device_IIM42XXX 12    // STM32G4  SPI2 12 (tab 91)
#define DMAMUXtx_id_device_IIM42XXX 13    // ---
#define InitRCC_DMA_IIM42XXX  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
#define IRQ_DMArx_IIM42XXX    DMA1_Channel1_IRQn
#define IRQ_pin_IIM42XXX      EXTI15_10_IRQn  //INT_fifo_ready EN


#define OPERATION_MODE_IIM42xxx 0x01
#define CONFIG_MODE_IIM42xxx    0x02
#define DISABLED_IIM42xxx       0x80
#define INT_FIFO_IIM42xxx       0x40
#define DMA_OK_IIM42xxx       	0x20


//////// DATA


uint8_t iim_42652_status = DISABLED_IIM42xxx,
		n_16bit_packet_fifo =12 ;// 20byte

int 	gyro_IIM42652[3]  = { 0 },
		accel_IIM42652[3] = { 0 },
		temp_IIM42652,
		timestamp_IIM42652;

uint8_t   raw_fifo_buffer[40]={0};
uint16_t  DMA_RX_buf_iim42652[40] = {0};
uint16_t  DMA_TX_buf_iim42652[40] = {0};

 struct imu_data{

	uint8_t* 	  status;
	int *		    gyro;
	int *		   aceel;
	int *	 temperature;
	int *	   timestamp;

	uint8_t *  raw_fifo_buf;
	uint16_t*  DMA_RX_fifo_buf;
	uint16_t*  DMA_TX_fifo_buf;
	uint8_t *  n_16bit_packet_fifo;

	uint16_t* reg_status;
	uint16_t* reg_config;
	uint16_t* reg_gyro_accel_temp;

	uint16_t  n_raw_fifo_buf;
	uint16_t  n_reg_config;
	uint16_t  n_reg_status;
	uint16_t  n_reg_gyro_accel_temp;

};

//////// REGISTR IIN42652


/*IIM_42652*/
#define CHIP_ID_42652 			0x6F

/*User bank 0*/
/*
 [4]Вибір режиму SPI
 *	0: Режим 0 та Режим 3 (за замовчуванням)
 *	1: Режим 1 та Режим 2
 [1]Конфігурація програмного скидання
 *	0: Нормальний (за замовчуванням)
 *	1: Увімкнути скидання
 */
#define DEVICE_CONFIG 		0x11

/*
 *[5:3] Керування швидкістю наростання для вихідного піна 14 тільки в режимі I2C
 *	000: 20нс-60нс, 001: 12нс-36нс, 010: 6нс-18нс,011: 4нс-12нс,100: 2нс-6нс
 *	101: < 2нс 110: Резервоване 111: Резервоване
 *[2;0] Керування швидкістю наростання для вихідного піна 14 в режимі SPI або I3CSM, а також для всіх інших вихідних пінів
 *	000: 20нс-60нс,    001: 12нс-36нс, 010: 6нс-18нс, 011: 4нс-12нс, 100: 2нс-6нс, 101: < 2нс
 *	110: Резервоване , 111: Резервоване
 *	*/
#define DRIVE_CONFIG 		0x13

/*
 [5] TEMP_DIS
 *	0: Датчик температури увімкнено (за замовчуванням)
 *	1: Датчик температури вимкнено
 [4]  IDLE
 * 	Якщо цей біт встановлений на 1, RC-осцилятор буде увімкнено, навіть якщо акселерометр і гіроскоп вимкнені.
 * 	Зазвичай цей біт встановлений на 0, тому, коли акселерометр і гіроскоп вимкнені, чіп перейде в стан OFF,
 * 	оскільки RC-осцилятор також буде вимкнено.
 [3:2]Режими гіроскопа
 *	  00: Вимикає гіроскоп (за замовчуванням)
 *    01: Поміщає гіроскоп у режим очікування (Standby Mode)
 *	  10: Резервоване
 *	  11: Поміщає гіроскоп у режим низького шуму (Low Noise Mode)
 *	Гіроскоп потрібно тримати увімкненим щонайменше 45 мс.
 *	Під час переходу з OFF в інші режими не виконуйте запису регістрів протягом 200 мкс.
 [1:0] Режими акселерометра
 *	  00: Вимикає акселерометр (за замовчуванням)
 *	  01: Вимикає акселерометр
 *	  10: Поміщає акселерометр у режим низького споживання (Low Power Mode)
 *	  11: Поміщає акселерометр у режим низького шуму (Low Noise Mode)
 * Під час переходу з OFF в інші режими не виконуйте запису регістрів протягом 200 мкс.
 */
#define PWR_MGMT0 			0x4E

/*
 [6]Коли цей біт встановлено на 1, DMP (Digital Motion Processor) увімкнено.
 [5]Коли цей біт встановлено на 1, пам'ять DMP скидається.
 [4]-----
 [3]Коли цей біт встановлено на 1, шлях сигналу скидається шляхом
 перезапуску лічильника ODR (Output Data Rate) та керування шляхом сигналу.
 [2]Коли цей біт встановлено на 1, лічильник часових міток фіксується в регістрі
 часових міток. Це біт, який очищається при запису.
 [1]При встановленні на 1, FIFO (First In, First Out) буде очищено.
 [0]-----
 */
#define SIGNAL_PATH_RESET 	0x4B

/*
 GYRO_ACCEL_CONFIG0
 Адреса: 82 (52h)(R/W) default: 0x11
 [7:4] ACCEL_UI_FILT_BW (Ширина смуги для фільтра акселерометра у режимі низької латентності):
 *	    0: BW = ODR/2
 *		1: BW = max(400Hz, ODR)/4 (за замовчуванням)
 *		2: BW = max(400Hz, ODR)/5
 *		3: BW = max(400Hz, ODR)/8
 *		4: BW = max(400Hz, ODR)/10
 *		5: BW = max(400Hz, ODR)/16
 *		6: BW = max(400Hz, ODR)/20
 *		7: BW = max(400Hz, ODR)/40
 *		8 до 13: Резервоване
 *		14: Опція низької латентності: тривіальне декімування при ODR виходу фільтра Dec2. Dec2 працює з max(400Hz, ODR)
 *		15: Опція низької латентності: тривіальне декімування при ODR виходу фільтра Dec2. Dec2 працює з max(200Hz, 8*ODR)
 [3:0] GYRO_UI_FILT_BW (Ширина смуги для фільтра гіроскопа у режимі низької латентності):
 *		0: BW = ODR/2
 *		1: BW = max(400Hz, ODR)/4 (за замовчуванням)
 *		2: BW = max(400Hz, ODR)/5
 *		3: BW = max(400Hz, ODR)/8
 *		4: BW = max(400Hz, ODR)/10
 *		5: BW = max(400Hz, ODR)/16
 *		6: BW = max(400Hz, ODR)/20
 *		7: BW = max(400Hz, ODR)/40
 *		8 до 13: Резервоване
 *		14: Опція низької латентності: тривіальне декімування при ODR виходу фільтра Dec2. Dec2 працює з max(400Hz, ODR)
 *		15: Опція низької латентності: тривіальне декімування при ODR виходу фільтра Dec2. Dec2 працює з max(200Hz, 8*ODR)
 */
#define GYRO_ACCEL_CONFIG0 	0x52
#define SELF_TEST_CONFIG 	0x70
#define WHO_AM_I 			0x75
#define REG_BANK_SEL 		0x76

// register banks
#define   IIM42652_REG_BANK_0      0x00
#define   IIM42652_REG_BANK_1      0x01
#define   IIM42652_REG_BANK_2      0x02
#define   IIM42652_REG_BANK_3      0x03
#define   IIM42652_REG_BANK_4      0x04

/*User bank 1*/

#define SENSOR_CONFIG0 	    0x03

/*		User bank 3
 * екекекек
 * ккееккекк*/
#define PU_PD_CONFIG1 0x06
#define PU_PD_CONFIG2 0x0E

///////////////////////// FIFO //////////////////////////

#define FIFO_COUNTH 		0x2E  //		7:0  FIFO_COUNT[15:8]
#define FIFO_COUNTL 		0x2F  //		7:0  FIFO_COUNT[7:0]
#define FIFO_DATA 			0x30  //		7:0 FIFO_DATA FIFO data port

/* [7:6]
 * 		00: Bypass Mode (default)
 *		01: Stream-to-FIFO Mode
 *		10: STOP-on-FULL Mode
 *		11: STOP-on-FULL Mode
 */
#define	FIFO_CONFIG 		0x16



/*FIFO_WM[7:0]
 Функція: Нижні біти рівня FIFO.
 Генерує переривання, коли FIFO досягає або перевищує розмір FIFO_WM
 в байтах або записах відповідно до налаштування FIFO_COUNT_REC.
 Переривання спрацьовує лише один раз. Цей регістр повинен бути
 встановлений на ненульове значення перед вибором цього джерела переривання
 */
#define FIFO_CONFIG2 		0x60

/*FIFO_WM[11:8]
 Функція: Верхні біти рівня FIFO.
 Генерує переривання, коли FIFO досягає або перевищує розмір FIFO_WM
 в байтах або записах      відповідно до налаштування FIFO_COUNT_REC.
 Переривання спрацьовує лише один раз. Цей регістр повинен бути встановлений
 на ненульове значення перед вибором цього джерела переривання.*/

#define FIFO_CONFIG3 		0x61

#define FIFO_LOST_PKT0 		0x6C  // Молодший байт, кількість пакетів, втрачених у FIFO..
#define FIFO_LOST_PKT1 		0x6D  // Старший байт, кількість пакетів, втрачених у FIFO.

/*User bank 4*/
/*
 * [6:1]
 Фактор декаментації швидкості пакетів FIFO. Встановлює кількість відкинутих пакетів FIFO. Дійсний діапазон – від 0 до 127. Користувач повинен вимкнути сенсори при ініціалізації значення FDR_SEL або при внесенні змін до нього.
 0000000: Декаментація вимкнена, всі пакети надсилаються до FIFO.
 0000001: 1 пакет з 2 надсилається до FIFO.
 0000010: 1 пакет з 3 надсилається до FIFO.
 0000011: 1 пакет з 4 надсилається до FIFO.
 …
 1111111: 1 пакет з 128 надсилається до FIFO.
 * */
#define FDR_CONFIG          0x09

/* DATA */
#define   FIFO_BYPASS                  0x00
#define   STREAM_TO_FIFO               0x01
#define   STREAM_TO_FIFO_STOP_ON_FULL  0x10

//////////////////////// INT ////////////////////////////////
/* Біти скидається в 0 після читання реєстру.
 *
 [6] Цей біт автоматично встановлюється в 1, коли генерується переривання UI FSYNC.
 [5] Цей біт автоматично встановлюється в 1, коли генерується переривання PLL Ready.
 [4] Цей біт автоматично встановлюється в 1, коли програмний скидання завершено.
 [3] Цей біт автоматично встановлюється в 1, коли генерується переривання Data Ready.
 [2] Цей біт автоматично встановлюється в 1, коли буфер FIFO досягає порогового значення.
 [1] Цей біт автоматично встановлюється в 1, коли буфер FIFO заповнений.
 [0] Цей біт автоматично встановлюється в 1, коли генерується переривання AGC Ready.

 * */

#define INT_STATUS 			0x2D

/*
 [7-4] reserved
 [3]SMD_INT	Переривання виявлення значного руху, скидається при читанні.
 [2]WOM_Z_INT Переривання пробудження при русі по осі Z, скидається при читанні.
 [1]WOM_Y_INTПереривання пробудження при русі по осі Y, скидається при читанні.
 [0]WOM_X_INT Переривання пробудження при русі по осі X, скидається при читанні.
 */
#define INT_STATUS2 		0x37
/*
 [7-6]
 [5]STEP_DET_INT     Переривання виявлення кроків, скидається при читанні.
 [4]STEP_CNT_OVF_INT Переривання переповнення лічильника кроків, скидається при читанні.
 [3]TILT_DET_INT     Переривання виявлення нахилу, скидається при читанні.
 [2]Резервовано.
 [1]FF_DET_INT       Переривання вільного падіння, скидається при читанні.
 [0]TAP_DET_INT      Переривання виявлення удару, скидається при читанні.
 */

#define INT_STATUS3 		0x38

/* default 0x00
 [5]Режим переривання INT2: 		0: Імпульсний режим 1: Замкнутий режим
 [4]Схема живлення INT2:    		0: Відкритий стік 	1: Витягування
 [3]Полярність переривання INT2: 	0: Активний низький 1: Активний високий
 [2]Режим переривання INT1: 		0: Імпульсний режим 1: Замкнутий режим
 [1]Схема живлення INT1: 			0: Відкритий стік 	1: Витягування
 [0]Полярність переривання INT1:	0: Активний низький 1: Активний високий
 * */

#define INT_CONFIG 			0x14

/*

 [5:4] DRDY_INT_CLEAR Опція скидання переривання "Дані готові" (замкнутий режим):
 *		00: Скидання при читанні статусного біта (за замовчуванням)
 *		01: Скидання при читанні статусного біта
 *		10: Скидання при читанні регістру сенсора
 *		11: Скидання при читанні статусного біта І при читанні реєстру сенсора

 [3:2]FIFO_THS_INT_CLEAR Опція скидання переривання "Поріг FIFO" (замкнутий режим):
 *		00: Скидання при читанні статусного біта (за замовчуванням)
 *		01: Скидання при читанні статусного біта
 *		10: Скидання при читанні даних FIFO 1 байт
 *		11: Скидання при читанні статусного біта І при читанні даних FIFO 1 байт

 [1:0]FIFO_FULL_INT_CLEAR Опція скидання переривання "FIFO заповнений" (замкнутий режим):
 *		00: Скидання при читанні статусного біта (за замовчуванням)
 *		01: Скидання при читанні статусного біта
 *		10: Скидання при читанні даних FIFO 1 байт
 *		11: Скидання при читанні статусного біта І при читанні даних FIFO 1 байт
 * */
#define INT_CONFIG0 		0x63

/*Reset value: 0x10
 [6] Тривалість імпульсу переривання:
		0: Тривалість імпульсу переривання становить 100 мкс. Використовувати тільки якщо ODR < 4 кГц.
 		1: Тривалість імпульсу становить 8 мкс. Вимагається, якщо ODR ≥ 4 кГц, необов'язково для ODR < 4 кГц.
 [5] Тривалість деактивації переривання:
 		0: Тривалість деактивації переривання встановлена на мінімум 100 мкс. Використовувати тільки якщо ODR < 4 кГц.
		1: Вимикає тривалість деактивації. Вимагається, якщо ODR ≥ 4 кГц, необов'язково для ODR < 4 кГц.
 [4] Користувач повинен змінити налаштування з 1 на 0 для правильної роботи контактів INT1 і INT2.
 * */
#define INT_CONFIG1 		0x64


/* Reset value: 0x91
 *
 * 3ACCEL_LP_CLK_SEL
 2RTC_MODE
 1:0CLKSEL

	 0: Режим низького споживання енергії акселерометра використовує тактовий сигнал осцилятора Wake Up.
	 1: Режим низького споживання енергії акселерометра використовує тактовий сигнал RC-осцилятора.

	 0: Не потрібен вхідний тактовий сигнал RTC.
	 1: Потрібен вхідний тактовий сигнал RTC.

 00: Завжди вибирати внутрішній RC-осцилятор.
 01: Вибрати PLL, коли доступно, в іншому випадку вибрати RC-осцилятор (за замовчуванням).
 10: Резервовано.
 11: Вимкнути всі тактові сигнали.
 * */

#define INTF_CONFIG1 		0x4D


/*
 *
[6] T1_EN  I3C_PROTOCOL_ERROR_IN
	 0: Переривання помилки протоколу I3CSM не направлене на INT1.
	 1: Переривання помилки протоколу I3CSM направлене на INT1.
[5:4] -Резервовано.
[3]	SMD_INT1_EN
	 0: Переривання SMD не направлене на INT1.
	 1: Переривання SMD направлене на INT1.
[2] WOM_Z_INT1_EN
	 0: Переривання WOM по осі Z не направлене на INT1.
	 1: Переривання WOM по осі Z направлене на INT1.
[1] WOM_Y_INT1_EN
	 0: Переривання WOM по осі Y не направлене на INT1.
	 1: Переривання WOM по осі Y направлене на INT1.
[0] WOM_X_INT1_EN
	 0: Переривання WOM по осі X не направлене на INT1.
	 1: Переривання WOM по осі X направлене на INT1.
 */

#define INT_SOURCE1 		0x66

/* Reset value: 0x00
[6]	UI_FSYNC_INT2_EN
	 0: Переривання UI FSYNC не направлене на INT2.
	 1: Переривання UI FSYNC направлене на INT2.
[5]	PLL_RDY_INT2_EN
	 0: Переривання готовності PLL не направлене на INT2.
	 1: Переривання готовності PLL направлене на INT2.
[4]	RESET_DONE_INT2_EN
	 0: Переривання завершення скидання не направлене на INT2.
	 1: Переривання завершення скидання направлене на INT2.
[3]	UI_DRDY_INT2_EN
	 0: Переривання готовності даних UI не направлене на INT2.
	 1: Переривання готовності даних UI направлене на INT2.
[2]	FIFO_THS_INT2_EN
	 0: Переривання порогу FIFO не направлене на INT2.
	 1: Переривання порогу FIFO направлене на INT2.
[1]	FIFO_FULL_INT2_EN
	 0: Переривання заповненості FIFO не направлене на INT2.
	 1: Переривання заповненості FIFO направлене на INT2.
[0]	UI_AGC_RDY_INT2_EN
	 0: Переривання готовності AGC UI не направлене на INT2.
	 1: Переривання готовності AGC UI направлене на INT2.*/

#define INT_SOURCE3 		0x68

/*

[6] T2_EN I3C_PROTOCOL_ERROR_IN
	 0: Переривання помилки протоколу I3CSM не направлене на INT2.
	 1: Переривання помилки протоколу I3CSM направлене на INT2.
[3]	SMD_INT2_EN
	 0: Переривання SMD не направлене на INT2.
	 1: Переривання SMD направлене на INT2.
[2] WOM_Z_INT2_EN
	 0: Переривання WOM по осі Z не направлене на INT2.
	 1: Переривання WOM по осі Z направлене на INT2.
[1]WOM_Y_INT2_EN
	 0: Переривання WOM по осі Y не направлене на INT2.
	 1: Переривання WOM по осі Y направлене на INT2.
[0]WOM_X_INT2_EN
	 0: Переривання WOM по осі X не направлене на INT2.
	 1: Переривання WOM по осі X направлене на INT2.*/

#define INT_SOURCE4 		0x69

/*User bank 1*/
/*
 [6]
 0: Device is on a bus with I2C and I3CSM devices
 1: Device is on a bus with I3CSM devices only
 [3]
 0: AP interface uses 3-wire SPI mode
 1: AP interface uses 4-wire SPI mode (default)
 * */
#define INTF_CONFIG4			0x7A
/*
 [2;1]
 Selects among the following functionalities for pin 9
 00: INT2
 01: FSYNC
 10: CLKIN
 11: Reserved */
#define INTF_CONFIG5 			0x7B

#define INTF_CONFIG6 			0x7C




////////////////////// TEMPERATURE //////////////////////

#define TEMP_DATA1_UI 		0x1D
#define TEMP_DATA0_UI 		0x1E

/////////////////////// ACCEL  //////////////////////////

#define SET_ACCEL_FS_SEL_16g                    (0x00 << 5)
#define SET_ACCEL_FS_SEL_8g                     (0x01 << 5)
#define SET_ACCEL_FS_SEL_4g                     (0x02 << 5)
#define SET_ACCEL_FS_SEL_2g                     (0x03 << 5)

#define SET_ACCEL_ODR_32kHz                     0x01
#define SET_ACCEL_ODR_16kHz                     0x02
#define SET_ACCEL_ODR_8kHz                      0x03
#define SET_ACCEL_ODR_4kHz                      0x04
#define SET_ACCEL_ODR_2kHz                      0x05
#define SET_ACCEL_ODR_1kHz                      0x06
#define SET_ACCEL_ODR_200Hz                     0x07
#define SET_ACCEL_ODR_100Hz                     0x08
#define SET_ACCEL_ODR_50Hz                      0x09
#define SET_ACCEL_ODR_25Hz                      0x0A
#define SET_ACCEL_ODR_12_5Hz                    0x0B
#define SET_ACCEL_ODR_6_25HZ                    0x0C
#define SET_ACCEL_ODR_3_125HZ                   0x0D
#define SET_ACCEL_ODR_1_5625HZ                  0x0E

/*
 Reset value: 0x06
 [7:5] ACCEL_FS_SEL Full scale select for accelerometer UI interface output
		000: ±16g (default)
		001: ±8g
		010: ±4g
		011: ±2g

 [3:0] ACCEL_ODR	Accelerometer ODR selection for UI interface output
		0000: Reserved
		0001: 32kHz
		0010: 16kHz
		0011: 8kHz (LN mode)
		0100: 4kHz (LN mode)
		0101: 2kHz (LN mode)
		0110: 1kHz (LN mode) (default)
		0111: 200Hz (LP or LN mode)
		1000: 100Hz (LP or LN mode)
		1001: 50Hz (LP or LN mode)
		1010: 25Hz (LP or LN mode)
		1011: 12.5Hz (LP or LN mode)
		1100: 6.25Hz (LP mode)
		1101: 3.125Hz (LP mode)
		1110: 1.5625Hz (LP mode)
		1111: 500Hz (LP or LN mode)
 */

#define ACCEL_CONFIG0 		0x50

/*
 Reset value: 0x0D
 [4:3]ACCEL_UI_FILT_ORD Selects order of ACCEL UI filter
		00: 1st Order
		01: 2nd Order
		10: 3rd Order
		11: Reserved
 [2:1]ACCEL_DEC2_M2_ORD Order of Accelerometer DEC2_M2 filter
		00: Reserved
		01: Reserved
		10: 3rd order
		11: Reserved
 */
#define ACCEL_CONFIG1 		0x53

#define ACCEL_DATA_X1_UI 	0x1F
#define ACCEL_DATA_X0_UI 	0x20
#define ACCEL_DATA_Y1_UI 	0x21
#define ACCEL_DATA_Y0_UI 	0x22
#define ACCEL_DATA_Z1_UI 	0x23
#define ACCEL_DATA_Z0_UI 	0x24

/*User bank 2*/
#define ACCEL_CONFIG_STATIC2 	0x03
#define ACCEL_CONFIG_STATIC3 	0x04
#define ACCEL_CONFIG_STATIC4 	0x05
#define XA_ST_DATA 				0x3B
#define YA_ST_DATA 				0x3C
#define ZA_ST_DATA 				0x3D




//////////////////////  GYRO  //////////////////////////

#define SET_GYRO_FS_SEL_2000_dps                (0x00 << 5)
#define SET_GYRO_FS_SEL_1000_dps                (0x01 << 5)
#define SET_GYRO_FS_SEL_500_dps                 (0x02 << 5)
#define SET_GYRO_FS_SEL_250_dps                 (0x03 << 5)
#define SET_GYRO_FS_SEL_125_dps                 (0x04 << 5)
#define SET_GYRO_FS_SEL_62_5_dps                (0x05 << 5)
#define SET_GYRO_FS_SEL_31_25_dps               (0x06 << 5)
#define SET_GYRO_FS_SEL_16_625_dps              (0x07 << 5)

#define SET_GYRO_ODR_32kHz                      0x01
#define SET_GYRO_ODR_16kHz                      0x02
#define SET_GYRO_ODR_8kHz                       0x03
#define SET_GYRO_ODR_4kHz                       0x04
#define SET_GYRO_ODR_2kHz                       0x05
#define SET_GYRO_ODR_1kHz                       0x06
#define SET_GYRO_ODR_200Hz                      0x07
#define SET_GYRO_ODR_100Hz                      0x08
#define SET_GYRO_ODR_50Hz                       0x09
#define SET_GYRO_ODR_25Hz                       0x0A
#define SET_GYRO_ODR_12_5Hz                     0x0B

/*
 Reset value: 0x06
 [7:5] GYRO_FS_SEL  Full scale select for gyroscope UI interface output
		000: ±2000dps (default)
		001: ±1000dps
		010: ±500dps
		011: ±250dps
		100: ±125dps
		101: ±62.5dps
		110: ±31.25dps
		111: ±15.625dps
 [3:0] GYRO_ODR
		0000: Reserved
		0001: 32kHz
		0010: 16kHz
		0011: 8kHz
		0100: 4kHz
		0101: 2kHz
		0110: 1kHz (default)
		0111: 200Hz
		1000: 100Hz
		1001: 50Hz
		1010: 25Hz
		1011: 12.5Hz
		1100: Reserved
		1101: Reserved
		1110: Reserved
		1111: 500Hz*/
#define GYRO_CONFIG0 		0x4F

/*Reset value: 0x16
[7:5] TEMP_FILT_BW Sets the bandwidth of the temperature signal DLPF
		000: DLPF BW = 4000Hz; DLPF Latency = 0.125ms (default)
		001: DLPF BW = 170Hz; DLPF Latency = 1ms
		010: DLPF BW = 82Hz; DLPF Latency = 2ms
		011: DLPF BW = 40Hz; DLPF Latency = 4ms
		100: DLPF BW = 20Hz; DLPF Latency = 8ms
		101: DLPF BW = 10Hz; DLPF Latency = 16ms
		110: DLPF BW = 5Hz; DLPF Latency = 32ms
		111: DLPF BW = 5Hz; DLPF Latency = 32ms
[3:2] GYRO_UI_FILT_ORD Selects order of GYRO UI filter
		00: 1st Order
		01: 2nd Order
		10: 3rd Order
		11: Reserved
[1:0] GYRO_DEC2_M2_ORDSelects order of GYRO DEC2_M2 Filter
		00: Reserved
		01: Reserved
		10: 3rd Order
		11: Reserved
 */
#define GYRO_CONFIG1 		0x51

#define GYRO_DATA_X1_UI 	0x25
#define GYRO_DATA_X0_UI 	0x26
#define GYRO_DATA_Y1_UI 	0x27
#define GYRO_DATA_Y0_UI 	0x28
#define GYRO_DATA_Z1_UI 	0x29
#define GYRO_DATA_Z0_UI 	0x2A

/*User bank 1*/

#define GYRO_CONFIG_STATIC2 	0x0B
#define GYRO_CONFIG_STATIC3 	0x0C
#define GYRO_CONFIG_STATIC4 	0x0D
#define GYRO_CONFIG_STATIC5 	0x0E
#define GYRO_CONFIG_STATIC6 	0x0F
#define GYRO_CONFIG_STATIC7 	0x10
#define GYRO_CONFIG_STATIC8 	0x11
#define GYRO_CONFIG_STATIC9 	0x12
#define GYRO_CONFIG_STATIC10 	0x13
#define XG_ST_DATA 				0x5F
#define YG_ST_DATA 				0x60
#define ZG_ST_DATA 				0x61

/* DATA*/





//////////////////////////// TMST ////////////////////////////////

#define TMST_CONFIG 		0x54
#define TMST_FSYNCH 		0x2B
#define TMST_FSYNCL 		0x2C

/*User bank 1*/

#define TMSTVAL0 				0x62
#define TMSTVAL1 				0x63
#define TMSTVAL2 				0x64

#define FSYNC_CONFIG 			0x62

//////////////////////// WOM //////////////////////////////////

#define APEX_DATA0 			0x31
#define APEX_DATA1 			0x32
#define APEX_DATA2 			0x33
#define APEX_DATA3 			0x34
#define APEX_DATA4 			0x35
#define APEX_DATA5 			0x36
#define APEX_CONFIG0 		0x55
#define SMD_CONFIG 			0x56

/*User bank 4*/
#define FDR_CONFIG              0x09
#define APEX_CONFIG1            0x40
#define APEX_CONFIG2            0x41
#define APEX_CONFIG3            0x42
#define APEX_CONFIG4            0x43
#define APEX_CONFIG5            0x44
#define APEX_CONFIG6            0x45
#define APEX_CONFIG7            0x46
#define APEX_CONFIG8            0x47
#define APEX_CONFIG9            0x48
#define APEX_CONFIG10           0x49
#define ACCEL_WOM_X_THR         0x4A
#define ACCEL_WOM_Y_THR         0x4B
#define ACCEL_WOM_Z_THR         0x4C
#define INT_SOURCE6                         0x4D
#define INT_SOURCE7                         0x4E

/*
[7:6] -
[5] FSYNC_IBI_EN  (частота синхронизации)
0: Прерывание FSYNC не направляется на IBI
1: Прерывание FSYNC направляется на IBI
[4] PLL_RDY_IBI_EN  (фаза-замкнута петля)
0: Прерывание готовности PLL не направляется на IBI
1: Прерывание готовности PLL направляется на IBI
[3] UI_DRDY_IBI_EN (дані готовності)
0: Прерывание готовности данных UI не направляется на IBI
1: Прерывание готовности данных UI направляется на IBI
[2] FIFO_THS_IBI_EN
0: Прерывание порогового значения FIFO не направляется на IBI
1: Прерывание порогового значения FIFO направляется на IBI
[1] FIFO_FULL_IBI_EN
0: Прерывание переполнения FIFO не направляется на IBI
1: Прерывание переполнения FIFO направляется на IBI
[0] AGC_RDY_IBI_EN (автоматичний контроль посилення)
0: Прерывание готовности AGC не направляется на IBI
*/


#define INT_SOURCE8                         0x4F
#define INT_SOURCE9                         0x50
#define INT_SOURCE10                        0x51
#define OFFSET_USER0                        0x77
#define OFFSET_USER1                        0x78
#define OFFSET_USER2                        0x79
#define OFFSET_USER3                        0x7A
#define OFFSET_USER4                        0x7B
#define OFFSET_USER5                        0x7C
#define OFFSET_USER6                        0x7D
#define OFFSET_USER7                        0x7E
#define OFFSET_USER8                        0x7F

#define READ_REG_II42xxx 		0x80
#define READ_REG16_II42xxx 		0x8000
#define WRITE_REG_II42xxx 		0
//////////////////////function///////////////////////////////////


////////////////////////////////////////////////////////
uint16_t imu_config_registr[]={

 		(FIFO_CONFIG  << 8)		| 0b01000000,			//  default: 0  | [7:6] 01: Stream-to-FIFO Mode

#define FIFO_CONFIG1 		0x5F

 		/*  Регістр FIFO_CONFIG1
 		 [7] Резерв
 		 [6] FIFO_WM_GT_TH
 		 * 	  0: Часткове читання FIFO вимкнено, потрібно повторно читати весь FIFO.
 		 * 	  1: Читання FIFO може бути частковим і відновлюється з останньої точки читання.
 		 [5] Триггерити переривання по рівню FIFO на кожному ODR (DMA запису), якщо FIFO_COUNT ≥ FIFO_WM_TH.
 		 [4] FIFO_HIRES_EN
 		 *	  0: Значення за замовчуванням; дані сенсора мають звичайну роздільну здатність.
 		 *	  1: Дані сенсора в FIFO матимуть розширену роздільну здатність, що дозволяє 20-байтовий пакет.
 		 [3] FIFO_TMST_FSYNC_EN
 		 [2] FIFO_TEMP_EN Увімкнути пакети з температурного сенсора для запису в FIFO.
 		 [1] FIFO_GYRO_EN Увімкнути пакети з гіроскопа для запису в FIFO.
 		 [0] FIFO_ACCEL_EN Увімкнути пакети з акселерометра для запису в FIFO.
 		 */
 		(FIFO_CONFIG1 << 8)	 	| 0b01110111,	//FIFO TEMP_EN FIFO_GYRO_EN FIFO_ACCEL_EN



		(FIFO_CONFIG2 << 8)	 	| 0x14,	//
		(FIFO_CONFIG3 << 8)	 	| 0x00,



//		(SIGNAL_PATH_RESET << 8)| 0x00,// default: 0
//		(INT_CONFIG << 8)	 	| 0x00,// default: 0
//		(INT_CONFIG0 << 8)	 	| 0x00,// default: 0x00
//		(INT_CONFIG1 << 8)	 	| 0x10,// default: 0x10


#define INTF_CONFIG0 		0x4C
		/*
		 Reset value: 0x30
		 Встановлення цього біта в 0:
		 Щоб сигналізувати про недійсний зразок і відрізнити його від дійсного зразка на основі значень:

	 [7] FIFO_HOLD_LAST_DATA_EN

	   0:
		 16-бітний пакет FIFO: Такі ж, як реєстри сенсора, за винятком:
		 Тегування FSYNC не застосовується до даних у FIFO.
		 20-бітний пакет FIFO:
		 Недійсні зразки позначаються значенням -524288.
		 Дійсні зразки в {-524288 до -524258} замінюються на -524256.
		 Дійсні зразки гіроскопа: Всі парні числа в {-524256 до +524286}.
		 Дійсні зразки акселерометра: Всі числа, які діляться на 4 в {-524256 до +524284}.
		 Тегування FSYNC не застосовується до даних у FIFO.
		 Встановлення цього біта в 1:
		 Реєстри сенсора:


		1: Недійсний зразок отримає копію останнього дійсного зразка.
		  16-бітний пакет FIFO: Такі ж, як реєстри сенсора, за винятком:
		  Тегування FSYNC не застосовується до даних у FIFO.
		  20-бітний пакет FIFO:
		  Дійсні зразки гіроскопа: Всі парні числа в {-524288 до +524286}.
		  Дійсні зразки акселерометра: Всі числа, які діляться на 4 в {-524288 до +524284}.


	 [6] FIFO_COUNT_REC
		 0: Кількість у FIFO звітується в байтах.
		 1: Кількість у FIFO звітується в записах (
			 1 запис = 16 байтів для заголовка
			  + дані гіроскопа + дані акселерометра
			  + дані температурного датчика
			  + мітка часу,
			  або 8 байтів для заголовка + дані гіроскопа/акселерометра + дані температурного датчика).

	 [5] FIFO_COUNT_ENDIAN
		 0: Кількість у FIFO звітується в форматі Little Endian.
		 1: Кількість у FIFO звітується в форматі Big Endian (за замовчуванням).

	 [4] SENSOR_DATA_ENDIAN
		 0: Дані сенсора звітуються в форматі Little Endian.
		 1: Дані сенсора звітуються в форматі Big Endian (за замовчуванням).

	[3l2] Резервовано.

	[0:1] UI_SIFS_CFG

		 10: Вимкнути SPI.
		 11: Вимкнути I2C.
*/
		(INTF_CONFIG0 <<8)		| 0b10000011,// default: 0x30 /  i2c disabled



//		(INTF_CONFIG1 <<8)		| 0x91,// default: 0x91

		(GYRO_CONFIG0 <<8)		| 0b00000111,// default: 0x06
//		(GYRO_CONFIG1 <<8)		| 0x16,// default: 0x16
//		(GYRO_ACCEL_CONFIG0 <<8)| 0x11,// default: 0x11
		(ACCEL_CONFIG0 << 8)	| 0b00000111,// default: 0x06
//		(ACCEL_CONFIG1 << 8)	| 0x0D,// default: 0x0D
//		(TMST_CONFIG << 8)		| 0x23,// default: 0x23
//		(APEX_CONFIG0 << 8)		| 0x82,// default: 0x82
//		(SMD_CONFIG << 8)		| 0x00,// default: 0x00

#define INT_SOURCE0 		0x65

/*Reset value: 0x10
		[6]	UI_FSYNC_INT1_EN
			 0: Переривання UI FSYNC не направлене на INT1.
			 1: Переривання UI FSYNC направлене на INT1.
		[5]	PLL_RDY_INT1_EN
			 0: Переривання готовності PLL не направлене на INT1.
			 1: Переривання готовності PLL направлене на INT1.
		[4] RESET_DONE_INT1_EN
			 0: Переривання завершення скидання не направлене на INT1.
			 1: Переривання завершення скидання направлене на INT1.
		[3]	UI_DRDY_INT1_EN
			 0: Переривання готовності даних UI не направлене на INT1.
			 1: Переривання готовності даних UI направлене на INT1.
		[2] FIFO_THS_INT1_EN
			 0: Переривання порогу FIFO не направлене на INT1.
			 1: Переривання порогу FIFO направлене на INT1.
		[1]	FIFO_FULL_INT1_EN
			 0: Переривання заповненості FIFO не направлене на INT1.
			 1: Переривання заповненості FIFO направлене на INT1.
		[0]	UI_AGC_RDY_INT1_EN
			 0: Переривання готовності AGC UI не направлене на INT1.
			 1: Переривання готовності AGC UI направлене на INT1.
		 * */
		(INT_SOURCE0 << 8)		| 0b00000110,// default: 0x10



//		(INT_SOURCE1 << 8)		| 0x00,// default: 0x00
//		(INT_SOURCE3 << 8)		| 0x00,// default: 0x00
//		(INT_SOURCE4 << 8)		| 0x00,// default: 0x00

		////////////////////bank sel 1 /////////////////////////

//		(REG_BANK_SEL << 8)		| 0x01,// default: 0x00
//		(SENSOR_CONFIG0 << 8)	| 0x80,// default: 0x80
//		(GYRO_CONFIG_STATIC2<<8)| 0xA0,// default: 0xA0
//		(GYRO_CONFIG_STATIC3<<8)| 0x0D,// default: 0x0D
//		(GYRO_CONFIG_STATIC4<<8)| 0xAA,// default: 0xAA
//		(GYRO_CONFIG_STATIC5<<8)| 0x80,// default: 0x80
		//(GYRO_CONFIG_STATIC6<<8)| 0x00,// default: 0xXX
		//(GYRO_CONFIG_STATIC7<<8)| 0x00,// default: 0xXX
		//(GYRO_CONFIG_STATIC8<<8)| 0x00,// default: 0xXX
		//(GYRO_CONFIG_STATIC9<<8)| 0x00,// default: 0xXX
		//(GYRO_CONFIG_STATIC10<<8)| 0x00,// default: 0xXX
//		(INTF_CONFIG4	<<8	)	| 0x03,// default: 0x03
//		(INTF_CONFIG5	<<8	)	| 0x20,// default: 0x20
//		(INTF_CONFIG6	<<8	)	| 0x5F,// default: 0x5F


		////////////////////bank sel 2 /////////////////////////

//		(REG_BANK_SEL << 8)		|  0x02,// default: 0x00
//		(ACCEL_CONFIG_STATIC2<<8)| 0x30,// default: 0x30
//		(ACCEL_CONFIG_STATIC3<<8)| 0x40,// default: 0x40
//		(ACCEL_CONFIG_STATIC4<<8)| 0x62,// default: 0x62

		////////////////////bank sel 3 /////////////////////////

//		(REG_BANK_SEL << 8)		| 0x03,// default: 0x00
//		(PU_PD_CONFIG1<<8)		| 0x88,// default: 0x88
//		(PU_PD_CONFIG2<<8)		| 0x20,// default: 0x20

		////////////////////bank sel 4 ////////////////////////

//		(REG_BANK_SEL << 8)		| 0x04,//default: 0x00
//		(FDR_CONFIG << 8)	 	| 0x00,//default: 0x00
//		(APEX_CONFIG1 << 8)	 	| 0xA2,//default: 0xA2
//		(APEX_CONFIG2 << 8)	 	| 0x85,//default: 0x85
//		(APEX_CONFIG3 << 8)	 	| 0x51,//default: 0x51
//		(APEX_CONFIG4 << 8)	 	| 0xA4,//default: 0xA4
//		(APEX_CONFIG5 << 8)	 	| 0x8C,//default: 0x8C
//		(APEX_CONFIG6 << 8)	 	| 0x5C,//default: 0x5C
//		(APEX_CONFIG7 << 8)	 	| 0x45,//default: 0x45
//		(APEX_CONFIG8 << 8)	 	| 0x5B,//default: 0x5B
//		(APEX_CONFIG9 << 8)	 	| 0x00,//default: 0x00
//		(APEX_CONFIG10 << 8)	| 0x00,//default: 0x00
//		(ACCEL_WOM_X_THR << 8)	| 0x00,//default: 0x00
//		(ACCEL_WOM_Y_THR << 8)	| 0x00,//default: 0x00
//		(ACCEL_WOM_Z_THR << 8)	| 0x00,//default: 0x00
//		(INT_SOURCE6 << 8)		| 0x00,//default: 0x00
//		(INT_SOURCE7 << 8)		| 0x00,//default: 0x00
//		(INT_SOURCE8 << 8)		| 0x00,//default: 0x00
//		(INT_SOURCE9 << 8)		| 0x00,//default: 0x00
//		(INT_SOURCE10 << 8)		| 0x00,//default: 0x00
//		(ACCEL_WOM_X_THR << 8)	| 0x00,//default: 0x00

//		(OFFSET_USER0 << 8)		| 0x00,//default: 0x00
//		(OFFSET_USER1 << 8)		| 0x00,//default: 0x00
//		(OFFSET_USER2 << 8)		| 0x00,//default: 0x00
//		(OFFSET_USER3 << 8)		| 0x00,//default: 0x00
//		(OFFSET_USER4 << 8)		| 0x00,//default: 0x00
//		(OFFSET_USER5 << 8)		| 0x00,//default: 0x00
//		(OFFSET_USER6 << 8)		| 0x00,//default: 0x00
//		(OFFSET_USER7 << 8)		| 0x00,//default: 0x00
//		(OFFSET_USER8 << 8)		| 0x00,//default: 0x00

		////////////////////bank sel 0 ////////////////////////
//		(REG_BANK_SEL << 8)		| 0x0,// default: 0x00
		(PWR_MGMT0 << 8) 	    | 0x1f,	// Reset value: 0x00/ GYRO,ACEEL,RC enable

};

uint16_t read_data_aceel_gyro_temp[]={

		(TEMP_DATA0_UI <<8)| READ_REG16_II42xxx | 0,
		(TEMP_DATA1_UI <<8)| READ_REG16_II42xxx | 0,
		(ACCEL_DATA_X0_UI <<8)| READ_REG16_II42xxx| 0,// ACCEL_DATA X,Y,Z
		(ACCEL_DATA_X1_UI <<8)| READ_REG16_II42xxx| 0,
		(ACCEL_DATA_Y0_UI <<8)| READ_REG16_II42xxx| 0,
		(ACCEL_DATA_Y1_UI <<8)| READ_REG16_II42xxx| 0,
		(ACCEL_DATA_Z0_UI <<8)| READ_REG16_II42xxx| 0,
		(ACCEL_DATA_Z1_UI <<8)| READ_REG16_II42xxx| 0,

		(GYRO_DATA_X0_UI <<8)| READ_REG16_II42xxx | 0,// GYRO_DATA X,Y,Z
		(GYRO_DATA_X1_UI <<8)| READ_REG16_II42xxx | 0,
		(GYRO_DATA_Y0_UI <<8)| READ_REG16_II42xxx | 0,
		(GYRO_DATA_Y1_UI <<8)| READ_REG16_II42xxx | 0,
		(GYRO_DATA_Z0_UI <<8)| READ_REG16_II42xxx | 0,
		(GYRO_DATA_Z1_UI <<8)| READ_REG16_II42xxx | 0,

};

uint16_t read_reg_status[]={

		(INT_STATUS <<8)  | 0,  // def 0x10
		(FIFO_COUNTH <<8) | 0,
		(FIFO_COUNTL <<8) | 0,
		(APEX_DATA0 <<8)  | 0,
		(APEX_DATA1 <<8)  | 0,
		(APEX_DATA2 <<8)  | 0,
		(APEX_DATA3 <<8)  | 0,
		(APEX_DATA4 <<8)  | 0,
		(APEX_DATA5 <<8)  | 0,
		(INT_STATUS2 <<8) | 0,
		(INT_STATUS3 <<8) | 0,
		(FIFO_LOST_PKT0 <<8) | 0,
		(FIFO_LOST_PKT1 <<8) | 0,

		(WHO_AM_I <<8) 		| 0,// GYRO_DATA X,Y,Z
		(REG_BANK_SEL <<8)  | 0,

//////////////////////// bank 1////////////////////////

		(XG_ST_DATA<<8)  | 0,
		(YG_ST_DATA <<8) | 0,
		(ZG_ST_DATA <<8) | 0,
		(TMSTVAL0 <<8) 	 | 0,
		(TMSTVAL1 <<8) 	 | 0,
		(TMSTVAL2 <<8)   | 0,

/////////////////////// BANK2 ///////////////////

};



 struct imu_data imu_iim42652 ={

//// Data sensor

	.status 	 = &iim_42652_status,
	.gyro 		 = gyro_IIM42652,
	.aceel  	 = accel_IIM42652,
	.temperature = &temp_IIM42652,

//// buffer,dma

	.raw_fifo_buf    = raw_fifo_buffer,
	.DMA_RX_fifo_buf = DMA_RX_buf_iim42652,
	.DMA_TX_fifo_buf = DMA_TX_buf_iim42652,
	.n_16bit_packet_fifo = &n_16bit_packet_fifo,

//// base reg

	.reg_status   	 	 = read_reg_status,
	.reg_config 		 = imu_config_registr,
	.reg_gyro_accel_temp = read_data_aceel_gyro_temp,

//// n - size

	.n_raw_fifo_buf 	= sizeof(raw_fifo_buffer),
	.n_reg_config 		= sizeof(imu_config_registr)/2,
	.n_reg_status 		= sizeof(read_reg_status)/2,
	.n_reg_gyro_accel_temp = sizeof(read_data_aceel_gyro_temp)/2,

 };

//////////////////// func communication

void DMA_init_IIM42xxx(struct imu_data* imu) {

	InitRCC_DMA_IIM42XXX // RCC

    // SPI_RX  DIR = 0 (Peripheral-to-Memory)

    DMArx_IIM42XXX->CCR = DMA_CCR_MINC   |    // RAM++
                          DMA_CCR_TCIE    |   // IRQ_RXNE
                          DMA_CCR_PSIZE_0 |   // 16 bit
                          DMA_CCR_MSIZE_0 |   // 16 bit
                         // DMA_CCR_PL_1   |    // Priority level HIGH
                          DMA_CCR_CIRC;       // circle RX

    DMArx_IIM42XXX->CNDTR = *(imu->n_16bit_packet_fifo);               // n__16bit
    DMArx_IIM42XXX->CPAR = (uint32_t)&(SPI_IIM42XXX->DR); // SPI data
    DMArx_IIM42XXX->CMAR = (uint32_t)imu->DMA_RX_fifo_buf;
    DMAMUXrx_IIM42XXX->CCR = DMAMUXrx_id_device_IIM42XXX ;//SPI2_RX (Table 91)


    //SPI_TX  DIR=1 (Memory-to-Peripheral)

    DMAtx_IIM42XXX->CCR = DMA_CCR_MINC 	  |     // RAM++
                          DMA_CCR_DIR     |     // Memory-to-Peripheral
                          DMA_CCR_PSIZE_0 |     // 16-bit
                          DMA_CCR_MSIZE_0 ;	   // 16-bit

    DMAtx_IIM42XXX->CNDTR = *(imu->n_16bit_packet_fifo);  // Количество слов для передачи
    DMAtx_IIM42XXX->CPAR = (uint32_t)&(SPI_IIM42XXX->DR); // Адрес регистра SPI
    DMAtx_IIM42XXX->CMAR = (uint32_t)imu->DMA_TX_fifo_buf;// Адрес буфера передачи

    DMAMUXtx_IIM42XXX->CCR = DMAMUXtx_id_device_IIM42XXX ;// SPI2_TX (Table 91)

}


/////////////////// sensor block ////////////////

/*  uint16_t block[] ={
 *  ((reg<<8) | dir | data),
 *  ((reg<<8) | dir | data),
 *  ((banksel)| dir | N),
 *  ((reg<<8) | dir | data),
 *  ,,, }

*/
uint8_t RW_REG_IIM42xxx(uint16_t* reg,uint16_t dir, uint16_t len) {

	uint8_t data, error = 0;

	while (len--){

		IIM42XXX_CS_on
		error = SPI_sensor_reg_data_check(SPI_IIM42XXX,(*reg)|(dir),&data);
		IIM42XXX_CS_off
		if(error) break;
		if(dir) *reg = ((*reg)&0xff00)|data;// read 1, write 0
		reg++;
	};

	return error;
};

/////// init IIM42652

uint8_t init_iim42652(struct imu_data* imu){

	uint8_t  data;

	*(imu->status) = DISABLED_IIM42xxx | CONFIG_MODE_IIM42xxx ;

	IIM42XXX_CS_on
	data = SPI_reg_data(SPI_IIM42XXX,(WHO_AM_I|READ_REG_II42xxx), 0x00);
	IIM42XXX_CS_off
	if(data != CHIP_ID_42652) return 1;

	// init block write reg

	data = RW_REG_IIM42xxx(imu->reg_config,WRITE_REG_II42xxx,imu->n_reg_config);
	if(data)return data;

	*(imu->status) &=~DISABLED_IIM42xxx;// Ok

	DMA_init_IIM42xxx(imu);

	 DMA_SPIenable_IIM42xxx;			     //SPI dma 16bit
	 DMArx_IIM42XXX->CCR |= DMA_CCR_EN;
     NVIC_EnableIRQ(IRQ_DMArx_IIM42XXX); //RX_buffer_circle
     NVIC_EnableIRQ(IRQ_pin_IIM42XXX);   //INT_fifo_ready EN

     *(imu->status) &= ~CONFIG_MODE_IIM42xxx;
     *(imu->status) |= OPERATION_MODE_IIM42xxx | DMA_OK_IIM42xxx ;

	systick_pause = 2;//2ms
	while(systick_pause);

	return 0;
}

///////////// DATA - OPERATION_MODE

uint8_t Data_fifo_pack20byte_IIM42xxx(struct imu_data* imu){

	//uint8_t  error= 1;
	uint8_t *data8 = (uint8_t*)imu->DMA_RX_fifo_buf;

	if( ((*data8) & 0x04) == 0)return 1;

/*
 [7]	HEADER_MSG
		 1: FIFO порожній<br>
		 0: Пакет містить дані сенсорів

 [6]	HEADER_ACCEL
		 1: Пакет має розмір, щоб дані акселерометра мали місце в пакеті, FIFO_ACCEL_EN має бути 1
         0: Пакет не містить вибірки акселерометра
[5]		HEADER_GYRO
		 1: Пакет має розмір, щоб дані гіроскопа мали місце в пакеті, FIFO_GYRO_EN має бути 1
		 0: Пакет не містить вибірки гіроскопа
[4]	    HEADER_20
		 1: Пакет містить новий і дійсний зразок розширених 20-бітних даних для гіроскопа і/або акселерометра
		 0: Пакет не містить нового і дійсного розширеного 20-бітного даних

[3:2]	HEADER_TIMESTAMP_FSYNC
		00: Пакет не містить часові мітки або дані часу FSYNC
		01: Зарезервовано<br>
		10: Пакет містить часову мітку ODR<br>11: Пакет містить час FSYNC, і цей пакет позначений як перший
		    ODR після FSYNC (тільки якщо FIFO_TMST_FSYNC_EN дорівнює 1)
[1]	HEADER_ODR_ACCEL
		1: ODR для акселерометра відрізняється для цього пакета даних акселерометра в порівнянні з попереднім пакетом
		0: ODR для акселерометра такий же, як і в попередньому пакеті з акселерометром
[0]	HEADER_ODR_GYRO
		1: ODR для гіроскопа відрізняється для цього пакета даних гіроскопа в порівнянні з попереднім пакетом гіроскопа
		0: ODR для гіроскопа такий же, як і в попередньому пакеті з гіроскопом

20bytes [5] FIFO_COUNT_ENDIAN =1 :  0
[0]FIFO Header[]					[0]FIFO Header[]
[1]Accel X [19:12]					[1]Accel X [11:4]
[2]Accel X [11:4]					[2]Accel X [19:12]
[3]Accel Y [19:12]
[4]Accel Y [11:4]
[5]Accel Z [19:12]
[6]Accel Z [11:4]
[7]Gyro X [19:12]
[8]Gyro X [11:4]
[9Gyro Y [19:12]
[10]Gyro Y [11:4]
[11]Gyro Z [19:12]
[12]Gyro Z [11:4]
[13]Temperature[15:8]
[14]Temperature[7:0]
[15]TimeStamp[15:8]
[16]TimeStamp[7:0]
[17]Accel X [3:0] | Gyro X [3:0]
[18]Accel Y [3:0] | Gyro Y [3:0]
[19]Accel Z [3:0] | Gyro Z [3:0]

*/

	if( ((*data8+5)&0x80))return 2; //HEADER_MSG ? new msg == 0

	if( ((*data8+5)&0x40)){ //HEADER_ACCE

		imu->aceel[0] = *(data8+7) << 12 | *(data8+4) <<4 | (*(data8+20)&0xf0) >> 4;
		imu->aceel[1] = *(data8+9) << 12 | *(data8+6) <<4 | (*(data8+23)&0xf0) >> 4; //X
		imu->aceel[2] = *(data8+11)<< 12 | *(data8+8) <<4 | (*(data8+22)&0xf0) >> 4; //X
	};

	if( ((*data8+5)&0x20)){ //GYRO

		imu->gyro [0] = *(data8+13)<<12	 | *(data8+10) << 4| (*(data8+20)&0x0f); //X
		imu->gyro [1] = *(data8+15)<<12  | *(data8+12) << 4| (*(data8+23)&0x0f); //X
		imu->gyro [2] = *(data8+17)<<12  | *(data8+14) << 4| (*(data8+22)&0x0f); //X


    };

	*imu->temperature  = *(data8+19)<<8 | *(data8+16);
	*imu->timestamp    = *(data8+21)<<8 | *(data8+18);

return 0;
}

uint8_t Data_fifo_pack16byte_IIM42xxx(struct imu_data* imu){

//	uint8_t  error= 1;
	uint8_t *data8 = (uint8_t*)imu->DMA_RX_fifo_buf;

	if( ((*data8) & 0x04) == 0)return 1;

/*
0x00Заголовок FIFO
0x01Accel X       [15:8]
0x02Прискорення X [7:0]
0x03Прискорення Y [15:8]
0x04Прискорення Y [7:0]
0x05Accel Z [15:8]
0x06Accel Z [7:0]
0x07Г роскоп X [15:8]
0x08Gyro X [7:0]
0x09Г роскоп Y [15:8]
0x0AГ роскоп Y [7:0]
0x0BГ роскоп Z [15:8]
0x0CGyro Z [7:0]
0x0DТемпература[7:0]
0x0EМ тка часу[15:8]
0x0FМ тка часу[7:0]
*/

	if( ((*data8+5)&0x80))return 2; //HEADER_MSG ? new msg == 0

	if( ((*data8+5)&0x40)){ //HEADER_ACCE

		imu->aceel[0] = *(data8+7) << 8 | *(data8+4);
		imu->aceel[1] = *(data8+9) << 8 | *(data8+6);
		imu->aceel[2] = *(data8+11)<< 8 | *(data8+8);
	};

	if( ((*data8+5)&0x20)){ //GYRO

		imu->gyro [0] = *(data8+13)<<8	| *(data8+10);
		imu->gyro [1] = *(data8+15)<<8  | *(data8+12);
		imu->gyro [2] = *(data8+17)<<8  | *(data8+14);


    };

	*imu->temperature  = *(data8+16);
	*imu->timestamp    = *(data8+21)<<8 | *(data8+18);

return 0;
}






/*
 *
 ///
  *
  *

void DMA_Init_SPI2_IIM42652(struct imu_data* imu, uint16_t n_16bit) {

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

    // DMA1 Channel 1 для SPI2_RX (Peripheral-to-Memory)

    DMA1_Channel1->CCR = DMA_CCR_MINC |        // RAM++
                         DMA_CCR_TCIE |        // IRQ_RXNE
                         DMA_CCR_PSIZE_0 |     // 16 bit
                         DMA_CCR_MSIZE_0 |     // 16 bit
                         DMA_CCR_CIRC;
                         				         // DIR = 0 (Peripheral-to-Memory)
    DMA1_Channel1->CNDTR = n_16bit;               // n__16bit
    DMA1_Channel1->CPAR = (uint32_t)&(SPI2->DR); // SPI data
    DMA1_Channel1->CMAR = (uint32_t)imu->raw_RX_fifo_buf;
    DMAMUX1_Channel0->CCR = 12 ;                // SPI2_RX (Table 91)


    // DMA1 Channel 2 для SPI2_TX  DIR=1 (Memory-to-Peripheral)

    DMA1_Channel2->CCR = DMA_CCR_MINC 	 |     // RAM++
                         DMA_CCR_DIR     |     // Memory-to-Peripheral
                         DMA_CCR_PSIZE_0 |     // 16-bit
                         DMA_CCR_MSIZE_0 ;	   // 16-bit

    DMA1_Channel2->CNDTR = n_16bit;             // Количество слов для передачи
    DMA1_Channel2->CPAR = (uint32_t)&(SPI2->DR); // Адрес регистра SPI
    DMA1_Channel2->CMAR = (uint32_t)imu->raw_TX_fifo_buf;  // Адрес буфера передачи

    DMAMUX1_Channel1->CCR = 13;                // SPI2_TX (Table 91)


}

///////////////////////////////
uint8_t load_gyro_aceel_temp(struct imu_data* imu){

	if(imu->n_raw_fifo_buf < imu->n_reg_gyro_accel_temp) return 3;
	return  SPI2_read_reg_to_array8_check(((TEMP_DATA1_UI <<8)| READ_REG16_II42xxx),
											imu->raw_fifo_buf,
											imu->n_reg_gyro_accel_temp);
};
uint8_t load_gyro_aceel_temp2(struct imu_data* imu){

	if(imu->n_raw_fifo_buf < imu->n_reg_gyro_accel_temp) return 3;
	return  SPI2_array16to8_check(imu->reg_gyro_accel_temp,imu->raw_fifo_buf,imu->n_reg_gyro_accel_temp);
};
*/

#endif /* INC_IIM_42652_H_ */
