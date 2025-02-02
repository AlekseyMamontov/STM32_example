/*
 * I2C.h
 *
 *  Created on: Feb 1, 2025
 *      Author: oleksii
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include <stdint.h>

#define TIMEOUT_I2C 160000 // Таймаут 1 мс при тактовой частоте 160 МГц


void I2C2_Init(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
    //RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C2RST; // Устанавливаем бит сброса
    //RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C2RST; // Сбрасываем бит сброса
    I2C2->TIMINGR = 0x30D29DE4;// 400кгц 160 мгц
    //I2C2->TIMINGR = ((9 << 28) | (10 << 20) | (9 << 16) | (799 << 8) | 799);  // 100 кГц
    I2C2->CR1 |= I2C_CR1_PE;   // Включаем I2C2
}

void I2C2_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
	   // Ожидание, пока шина не будет свободна
	    while (I2C2->ISR & I2C_ISR_BUSY);

	    // Установка адреса устройства и регистра для записи
	    I2C2->CR2 = (dev_addr << 1) | (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	    while (!(I2C2->ISR & I2C_ISR_TXIS)) {
	        if (I2C2->ISR & I2C_ISR_NACKF) {
	            // Обработка ошибки NACK (неподтверждение устройства)
	            I2C2->ICR |= I2C_ICR_NACKCF;  // Сброс флага NACK
	            return;  // Выход из функции при ошибке
	        }
	    }

	    // Отправка адреса регистра
	    I2C2->TXDR = reg_addr;
	    while (!(I2C2->ISR & I2C_ISR_TXIS)) {
	      if (I2C2->ISR & I2C_ISR_NACKF) {
	        // Обработка ошибки NACK (неподтверждение устройства)
	            I2C2->ICR |= I2C_ICR_NACKCF;  // Сброс флага NACK
	           return;  // Выход из функции при ошибке
	       }
	    }

	    // Отправка данных
	    I2C2->TXDR = data;
	    while (!(I2C2->ISR & I2C_ISR_TC)) {
	        if (I2C2->ISR & I2C_ISR_NACKF) {
	            // Обработка ошибки NACK (неподтверждение устройства)
	            I2C2->ICR |= I2C_ICR_NACKCF;  // Сброс флага NACK
	            return;  // Выход из функции при ошибке
	        }
	    }

	    // Завершение передачи
	    I2C2->CR2 |= I2C_CR2_STOP;
}

// info error



uint8_t I2C2_WriteCheck(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {

    // Ожидание, пока шина не будет свободна
    uint32_t timeout = TIMEOUT_I2C;
    while (I2C2->ISR & I2C_ISR_BUSY) {
        if (--timeout == 0) {
            return 1; // Ошибка: превышено время ожидания на BUSY
        }
    }
    // Установка адреса устройства и количества байтов для передачи
    I2C2->CR2 = (dev_addr << 1) | I2C_CR2_START | (2 << I2C_CR2_NBYTES_Pos);

    // Отправка адреса регистра
    I2C2->TXDR = reg_addr;

    // Ожидание отправки адреса регистра или таймаут
    timeout = TIMEOUT_I2C;
    while (!(I2C2->ISR & I2C_ISR_TXIS)) {
        if (--timeout == 0) {
            return 2; // Ошибка: превышено время ожидания на TXIS
        }
    }

    // Проверка на NACK
    if (I2C2->ISR & I2C_ISR_NACKF) {
        I2C2->ICR |= I2C_ICR_NACKCF; // Сброс флага NACK
        return 3; // Ошибка: NACK
    }

    // Отправка данных
    I2C2->TXDR = data;

    // Ожидание завершения передачи данных или таймаут
    timeout = TIMEOUT_I2C;
    while (!(I2C2->ISR & I2C_ISR_TC)) {
        if (--timeout == 0) {
            return 4; // Ошибка: превышено время ожидания на TC
        }
    }

    // Проверка на NACK
    if (I2C2->ISR & I2C_ISR_NACKF) {
        I2C2->ICR |= I2C_ICR_NACKCF; // Сброс флага NACK
        return 5; // Ошибка: NACK
    }

    // Завершение передачи
    I2C2->CR2 |= I2C_CR2_STOP;

    return 0; // Успех

}


///////////////////////////////////////////////////////////////////////


uint8_t I2C2_ReadByte(uint8_t dev_addr, uint8_t reg_addr) {
    uint8_t data = 0;

    // Ожидание, пока шина не будет свободна
    uint32_t timeout = TIMEOUT_I2C;
    while (I2C2->ISR & I2C_ISR_BUSY) {
        if (--timeout == 0) {
            return 0; // Ошибка: превышено время ожидания на BUSY
        }
    }

    // Установка адреса устройства и регистра для чтения
    I2C2->CR2 = (dev_addr << 1) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    timeout = TIMEOUT_I2C;
    while (!(I2C2->ISR & I2C_ISR_TXIS)) {
        if (--timeout == 0) {
            return 0; // Ошибка: превышено время ожидания на TXIS
        }
    }

    I2C2->TXDR = reg_addr;
    timeout = TIMEOUT_I2C;
    while (!(I2C2->ISR & I2C_ISR_TC)) {
        if (--timeout == 0) {
            return 0; // Ошибка: превышено время ожидания на TC
        }
        if (I2C2->ISR & I2C_ISR_NACKF) {
            // Обработка ошибки NACK (неподтверждение устройства)
            I2C2->ICR |= I2C_ICR_NACKCF;  // Сброс флага NACK
            return 0; // Возвращаем ошибку
        }
    }

    // Установка адреса устройства для чтения данных
    I2C2->CR2 = (dev_addr << 1) | I2C_CR2_RD_WRN | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    timeout = TIMEOUT_I2C;
    while (!(I2C2->ISR & I2C_ISR_RXNE)) {
        if (--timeout == 0) {
            return 0; // Ошибка: превышено время ожидания на RXNE
        }
    }

    data = I2C2->RXDR;

    // Завершение передачи
    I2C2->CR2 |= I2C_CR2_STOP;

    return data; // Возвращаем считанное значение
}


uint8_t I2C2_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t length) {

    // Ожидание, пока шина не будет свободна
    uint32_t timeout = TIMEOUT_I2C;
    while (I2C2->ISR & I2C_ISR_BUSY) {
        if (--timeout == 0) {
            return 1; // Ошибка: превышено время ожидания на BUSY
        }
    }

    // Установка адреса устройства и регистра для чтения
    I2C2->CR2 = (dev_addr << 1) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    timeout = TIMEOUT_I2C;
    while (!(I2C2->ISR & I2C_ISR_TXIS)) {
        if (--timeout == 0) {
            return 2; // Ошибка: превышено время ожидания на TXIS
        }
    }

    I2C2->TXDR = reg_addr;
    timeout = TIMEOUT_I2C;
    while (!(I2C2->ISR & I2C_ISR_TC)) {
        if (--timeout == 0) {
            return 3; // Ошибка: превышено время ожидания на TC
        }
        if (I2C2->ISR & I2C_ISR_NACKF) {
            I2C2->ICR |= I2C_ICR_NACKCF;  // Сброс флага NACK
            return 4; // Ошибка: NACK
        }
    }

    // Установка адреса устройства для чтения данных
    I2C2->CR2 = (dev_addr << 1) | I2C_CR2_RD_WRN | (length << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    timeout = TIMEOUT_I2C;
    while (!(I2C2->ISR & I2C_ISR_RXNE)) {
        if (--timeout == 0) {
            return 5; // Ошибка: превышено время ожидания на RXNE
        }
    }

    // Чтение данных
    for (uint8_t i = 0; i < length; i++) {
        timeout = TIMEOUT_I2C;
        while (!(I2C2->ISR & I2C_ISR_RXNE)) {
            if (--timeout == 0) {
                return 6; // Ошибка: превышено время ожидания на RXNE
            }
        }
        data[i] = I2C2->RXDR;
    }

    // Завершение передачи
    I2C2->CR2 |= I2C_CR2_STOP;

    return 0;
}

////////////////////////////// BMP280 /////////////////////////


// Адрес датчика BMP280

// Регистры BMP280



int16_t data_calib[12];
int32_t raw_temperature;
int32_t raw_pressure;
int32_t t_fine;

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

    uint8_t addr;
};

static struct BMP280 BMP280_sensor1 = {

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

uint8_t BMP280_Init(struct BMP280 *sensor) {

    uint8_t per;

    I2C2_ReadBytes(sensor->addr,0xD0,&per, 1);

    if (per != 0x58) { // id  BMP280
        return 1; // Ошибка: неверный идентификатор
    }

    per = I2C2_WriteCheck(sensor->addr, 0xF4, 0x27);
	if(per) return per;



return BMP280_Read_Calibration_Data(sensor);
};


void  BMP280_Read_Raw_Data(struct BMP280 *sensor) {

    uint8_t data[6]; // Массив для хранения данных
    I2C2_ReadBytes(sensor->addr, 0xF7, data, 6); // Чтение 6 байт (давление + температура)

    *(sensor->raw_p) = ((int32_t)(data[0]) << 12) | ((int32_t)(data[1]) << 4) | ((data[2] >> 4) & 0x0F);
    *(sensor->raw_t) = ((int32_t)(data[3]) << 12) | ((int32_t)(data[4]) << 4) | ((data[5] >> 4) & 0x0F);
}

float BMP280_Compensate_Temperature(struct BMP280 *sensor) {
    int32_t var1, var2;
    var1 = ((((sensor->raw_t[0] >> 3) - ((int32_t)(*sensor->dig_T1) << 1)) * ((int32_t)(*sensor->dig_T2))) >> 11);
    var2 = (((((sensor->raw_t[0] >> 4) - (int32_t)(*sensor->dig_T1)) * ((sensor->raw_t[0] >> 4) - (int32_t)(*sensor->dig_T1)) >> 12) * (int32_t)(*sensor->dig_T3)) >> 14);
    *sensor->fin_t = var1 + var2; // Обновление подправленного значения
    return (*sensor->fin_t * 5 + 128) >> 8; // Возвращает температуру в градусах Цельсия
}

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




///////////////////////////////////////////////////////////////////////////


/*
 *
void DMA_Init(void) {
    // Включение тактирования DMA1
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Настройка DMA для I2C2
    DMA1_Channel1->CCR &= ~DMA_CCR_EN; // Отключение канала
    DMA1_Channel1->CPAR = (uint32_t)&I2C2->TXDR; // Адрес периферии
    DMA1_Channel1->CMAR = (uint32_t)dataToSend; // Адрес памяти (измените на нужный)
    DMA1_Channel1->CNDTR = 10; // Количество данных для передачи
    DMA1_Channel1->CCR |= DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_TCIE; // Настройка канала
    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // Разрешение прерывания
}

void I2C2_DMA_Transmit(uint8_t* data, uint16_t size) {
    // Настройка DMA для передачи
    DMA1_Channel1->CMAR = (uint32_t)data; // Установка адреса памяти
    DMA1_Channel1->CNDTR = size; // Установка количества данных

    // Запуск передачи
    I2C2->CR2 |= (size << 16); // Установка количества байтов
    I2C2->CR2 |= (I2C2_ADDRESS << 1); // Установка адреса устройства
    I2C2->CR2 |= I2C_CR2_START; // Начало передачи

    // Включение DMA
    DMA1_Channel1->CCR |= DMA_CCR_EN;
}

void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF1) { // Проверка флага завершения передачи
        DMA1->IFCR |= DMA_IFCR_CTCIF1; // Сброс флага
        I2C2->CR2 |= I2C_CR2_STOP; // Завершение передачи
    }
}
 * */






















#endif /* INC_I2C_H_ */
