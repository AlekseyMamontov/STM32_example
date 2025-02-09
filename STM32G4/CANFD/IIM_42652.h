/*
 * IIM_42652.h
 *
 *  Created on: Feb 1, 2025
 *      Author: oleksii
 */

#ifndef INC_IIM_42652_H_
#define INC_IIM_42652_H_
#include <Ixm42xxxDefs.h>

/*IIM_42652*/
#define CHIP_ID 			0x6F

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
#define BANK_SEL 			0x76

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
#define FIFO_CONFIG1 		0x5F

/*FIFO_WM[7:0]
 Функція: Нижні біти рівня вFIFO.
 Генерує переривання, коли FIFO досягає або перевищує розмір FIFO_WM
 в байтах або записах відповідно до налаштування FIFO_COUNT_REC.
 Переривання спрацьовує лише один раз. Цей регістр повинен бути
 встановлений на ненульове значення перед вибором цього джерела переривання
 */
#define FIFO_CONFIG2 		0x60

/*FIFO_WM[11:8]
 Функція: Верхні біти рівня вFIFO.
 Генерує переривання, коли FIFO досягає або перевищує розмір FIFO_WM
 в байтах або записах      відповідно до налаштування FIFO_COUNT_REC.
 Переривання спрацьовує лише один раз. Цей регістр повинен бути встановлений
 на ненульове значення перед вибором цього джерела переривання.*/
#define FIFO_CONFIG3 		0x61

#define FIFO_LOST_PKT0 		0x6C  // Молодший байт, кількість пакетів, втрачених у FIFO..
#define FIFO_LOST_PKT1 		0x6D  // Старший байт, кількість пакетів, втрачених у FIFO.

/* DATA */
#define   FIFO_BYPASS                  0x00
#define   STREAM_TO_FIFO               0x01
#define   STREAM_TO_FIFO_STOP_ON_FULL  0x10

//////////////////////// INT ////////////////////////////////

#define INT_STATUS 			0x2D
#define INT_STATUS2 		0x37
#define INT_STATUS3 		0x38

#define INT_CONFIG 			0x14
#define INT_CONFIG0 		0x63
#define INT_CONFIG1 		0x64

#define INTF_CONFIG0 		0x4C
#define INTF_CONFIG1 		0x4D

#define INT_SOURCE0 		0x65
#define INT_SOURCE1 		0x66
#define INT_SOURCE3 		0x68
#define INT_SOURCE4 		0x69

/*User bank 1*/

#define INTF_CONFIG4			0x7A
#define INTF_CONFIG5 			0x7B
#define INTF_CONFIG6 			0x7C

////////////////////// TEMPERATURE //////////////////////

#define TEMP_DATA1_UI 		0x1D
#define TEMP_DATA0_UI 		0x1E

/////////////////////// ACCEL  //////////////////////////

#define ACCEL_CONFIG0 		0x50
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

/*Data*/

#define SET_ACCEL_FS_SEL_16g                    0x00
#define SET_ACCEL_FS_SEL_8g                     0x01
#define SET_ACCEL_FS_SEL_4g                     0x02
#define SET_ACCEL_FS_SEL_2g                     0x03

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

//////////////////////  GYRO  //////////////////////////

#define GYRO_CONFIG0 		0x4F
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

#define SET_GYRO_FS_SEL_2000_dps                0x00
#define SET_GYRO_FS_SEL_1000_dps                0x01
#define SET_GYRO_FS_SEL_500_dps                 0x02
#define SET_GYRO_FS_SEL_250_dps                 0x03
#define SET_GYRO_FS_SEL_125_dps                 0x04
#define SET_GYRO_FS_SEL_62_5_dps                0x05
#define SET_GYRO_FS_SEL_31_25_dps               0x06
#define SET_GYRO_FS_SEL_16_625_dps              0x07

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

uint8_t int_iim_42652() {

}
;

#endif /* INC_IIM_42652_H_ */
