/*
 * BMP280.h
 *
 *  Created on: Apr 12, 2025
 *      Author: oleksii
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "I2C.h"
////////////////////////////// BMP280 /////////////////////////

#define OPERATION_MODE_BMP280 0x01
#define CONFIG_MODE_BMP280    0x02
#define DISABLED_BMP280       0x80
#define INT_FIFO_BMP280       0x40
#define DMA_OK_BMP280         0x20

// Адрес датчика BMP280

#define BMP280_ADDR 			0x76
#define BMP280_REG_DATA 		0xF7 //(0xF7-0xFC reg block 6 bytes)
#define BMP280_REG_CONFIG 		0xF5
#define BMP280_REG_CTRL_MEAS 	0xF4
#define BMP280_REG_STATUS   	0xF3
#define BMP280_REG_RESET   		0xE0
#define BMP280_REG_ID    		0xD0

// Регистры BMP280



int16_t data_calib[12];
int32_t raw_temperature;
int32_t raw_pressure;
int32_t t_fine;
uint8_t BMP280_dmarx_buffer[7]={0};

struct BMP280{

    uint16_t* dig_T1;
    int16_t*  dig_T2;
    int16_t*  dig_T3;
    uint16_t* dig_P1;
    int16_t*  dig_P2;
    int16_t*  dig_P3;
    int16_t*  dig_P4;
    int16_t*  dig_P5;
    int16_t*  dig_P6;
    int16_t*  dig_P7;
    int16_t*  dig_P8;
    int16_t*  dig_P9;


    int32_t* raw_t;
    int32_t* raw_p;
    int32_t* fin_t;

    uint8_t* status;
    uint8_t* DMArx_buf;
    uint8_t addr;
};

static struct BMP280 bmp280_sensor1 = {

		.dig_T1 = (uint16_t*)data_calib,
		.dig_T2 = data_calib+1,
		.dig_T3 = data_calib+2,
		.dig_P1 = (uint16_t*)data_calib+3,
		.dig_P2 = data_calib+4,
		.dig_P3 = data_calib+5,
		.dig_P4 = data_calib+6,
		.dig_P5 = data_calib+7,
		.dig_P6 = data_calib+8,
		.dig_P7 = data_calib+9,
		.dig_P8 = data_calib+10,
		.dig_P9 = data_calib+11,

		.raw_t = &raw_temperature,
		.raw_p = &raw_pressure,
		.fin_t = &t_fine,
		.addr = 0x76,


};

uint8_t  BMP280_Read_Calibration_Data(struct BMP280 *sensor) {
 return  I2C2_ReadBytes(sensor->addr, 0x88 ,(uint8_t*)data_calib, 24);
}



// Чтение данных (TX через прерывания, RX через DMA)

void BMP280_Read_Data(struct BMP280 *sensor) {
    if (!(*(sensor->status) & INT_FIFO_BMP280)) {
        *(sensor->status) |= INT_FIFO_BMP280;
        // Запустить отправку адреса регистра через прерывания
        I2C2->CR2 = (BMP280_ADDR << 1) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    }
}
//////////////////////

// Обработчик прерываний I2C (EV)
void I2C1_EV_IRQHandler(void) {

    if (I2C2->ISR & I2C_ISR_TXIS){ I2C2->TXDR = BMP280_REG_DATA;// Отправить адрес регистра

    }else if (I2C2->ISR & I2C_ISR_TC)  // Завершена отправка адреса, запустить чтение через DMA
        I2C2->CR2 = (BMP280_ADDR << 1) | I2C_CR2_RD_WRN | (6 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;
}


// Обработчик ошибок I2C

void I2C2_ER_IRQHandler(void) {
    if (I2C2->ISR & I2C_ISR_NACKF) {
        I2C2->ICR |= I2C_ICR_NACKCF;
        *(bmp280_sensor1.status) &= ~INT_FIFO_BMP280;
    }
    if (I2C2->ISR & I2C_ISR_BERR) {
        I2C2->ICR |= I2C_ICR_BERRCF;
        *(bmp280_sensor1.status) &= ~INT_FIFO_BMP280;
    }
}

// Обработчик DMA RX

void DMA1_Channel6_IRQHandler(void) {

    if (DMA1->ISR & DMA_ISR_TCIF6) {
        DMA1->IFCR |= DMA_IFCR_CTCIF6;
        *(bmp280_sensor1.status) &= ~INT_FIFO_BMP280;
        *(bmp280_sensor1.status) |= DMA_OK_BMP280;
    }
    DMA1->IFCR |= DMA_IFCR_CGIF6;
}

uint8_t init_bmp280(struct BMP280 *sensor) {

    uint8_t error =0;

    *(sensor->status) = DISABLED_BMP280 | CONFIG_MODE_BMP280;
    I2C2_ReadBytes(sensor->addr,BMP280_REG_ID,&error, 1);

    if (error != 0x58) return 1; // id  BMP280

    error = I2C2_WriteCheck(sensor->addr, BMP280_REG_CTRL_MEAS, 0x27);

	if(error) return error;

	*(sensor->status) &= ~DISABLED_BMP280;

	error = BMP280_Read_Calibration_Data(sensor);

	if(error) return error;

	I2C2->CR1 &= ~I2C_CR1_PE;

    I2C2->CR1 |= I2C_CR1_RXDMAEN | I2C_CR1_ERRIE | I2C_CR1_TXIE | I2C_CR1_TCIE;

    // DMA1_Channel6 (RX, круговой режим)

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

    DMA1_Channel6->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE ;

    DMA1_Channel6->CPAR = (uint32_t)&I2C2->RXDR;
    DMA1_Channel6->CMAR = (uint32_t)sensor->DMArx_buf;
    DMA1_Channel6->CNDTR = 6; // 6 байт
	DMAMUX1_Channel5->CCR = 18; // I2C2_RX  DMAMUX (таб 91, RM0440)

	I2C2->CR1 |= I2C_CR1_PE;
	DMA1_Channel6->CCR |= DMA_CCR_EN;

	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    NVIC_EnableIRQ(I2C2_EV_IRQn);
    NVIC_EnableIRQ(I2C2_ER_IRQn);

    *(sensor->status) &= ~CONFIG_MODE_BMP280;
	*(sensor->status) |= OPERATION_MODE_BMP280;

return 0 ;
};

////////////

void  BMP280_Read_Raw_Data(struct BMP280 *sensor) {

    uint8_t data[6]; // Массив для хранения данных
    I2C2_ReadBytes(sensor->addr, 0xF7, data, 6); // Чтение 6 байт (давление + температура)

    *(sensor->raw_p) = ((int32_t)(data[0]) << 12) | ((int32_t)(data[1]) << 4) | ((data[2] >> 4) & 0x0F);
    *(sensor->raw_t) = ((int32_t)(data[3]) << 12) | ((int32_t)(data[4]) << 4) | ((data[5] >> 4) & 0x0F);
}

////////////

float BMP280_Compensate_Temperature(struct BMP280 *sensor) {
    int32_t var1, var2;
    var1 = ((((sensor->raw_t[0] >> 3) - ((int32_t)(*sensor->dig_T1) << 1)) * ((int32_t)(*sensor->dig_T2))) >> 11);
    var2 = (((((sensor->raw_t[0] >> 4) - (int32_t)(*sensor->dig_T1)) * ((sensor->raw_t[0] >> 4) - (int32_t)(*sensor->dig_T1)) >> 12) * (int32_t)(*sensor->dig_T3)) >> 14);
    *sensor->fin_t = var1 + var2; // Обновление подправленного значения
    return (*sensor->fin_t * 5 + 128) >> 8; // Возвращает температуру в градусах Цельсия
}

////////////

float BMP280_Compensate_Pressure(struct BMP280 *sensor) {
    int64_t var1, var2, p;
    var1 = ((int64_t)(*sensor->fin_t)) - 128000;
    var2 = var1 * var1 * (int64_t)(*sensor->dig_P6);
    var2 = var2 + ((var1 * (int64_t)(*sensor->dig_P5)) << 17);
    var2 = var2 + (((int64_t)(*sensor->dig_P4)) << 35);
    var1 = ((var1 * var1 * (int64_t)(*sensor->dig_P3)) >> 8) + ((var1 * (int64_t)(*sensor->dig_P2)) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)(*sensor->dig_P1)) >> 33;

    if (var1 == 0) return 0; // Ошибка деления на ноль

    p = 1048576 - sensor->raw_p[0];
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)(*sensor->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)(*sensor->dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)(*sensor->dig_P7)) << 4);
    return (float)p / 256; // Возвращает давление в Паскалях
}


#endif /* INC_BMP280_H_ */
