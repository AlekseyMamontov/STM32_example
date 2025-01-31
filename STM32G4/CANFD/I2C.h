/*
 * I2C.h
 *
 *  Created on: Feb 1, 2025
 *      Author: oleksii
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include <stdint.h>

void I2C2_Init(void);
int32_t BMP280_ReadTemperature(void);
void    BMP280_ReadCalibrationData(void);
void    BMP280_Init(void);
uint8_t I2C2_ReadByte(uint8_t dev_addr, uint8_t reg_addr);


#define BMP280_I2C_ADDRESS 0x76  // Адрес датчика BMP280

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
} BMP280_CalibData;

BMP280_CalibData calib_data;

void I2C2_Init(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
    I2C2->CR1 |= I2C_CR1_PE;
    for (volatile int i = 0; i <10; i++);
    I2C2->CR1 &= ~I2C_CR1_PE; // Set PE to 0
    for (volatile int i = 0; i < 10; i++); // Wait for at least 3 APB clock cycles


    //RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C2RST; // Устанавливаем бит сброса
    //RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C2RST; // Сбрасываем бит сброса
    I2C2->TIMINGR = 0x30D29DE4;
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

uint8_t I2C2_ReadByte(uint8_t dev_addr, uint8_t reg_addr) {
    uint8_t data = 0;

    // Ожидание, пока шина не будет свободна
    while (I2C2->ISR & I2C_ISR_BUSY);

    // Установка адреса устройства и регистра для чтения
    I2C2->CR2 = (dev_addr << 1) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    while (!(I2C2->ISR & I2C_ISR_TXIS));

    I2C2->TXDR = reg_addr;
    while (!(I2C2->ISR & I2C_ISR_TC)) {
        if (I2C2->ISR & I2C_ISR_NACKF) {
            // Обработка ошибки NACK (неподтверждение устройства)
            I2C2->ICR |= I2C_ICR_NACKCF;  // Сброс флага NACK
            return 0xFF;  // Возвращаем ошибку
        }
    }

    // Установка адреса устройства для чтения данных
    I2C2->CR2 = (dev_addr << 1) | I2C_CR2_RD_WRN | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    while (!(I2C2->ISR & I2C_ISR_RXNE));

    data = I2C2->RXDR;

    // Завершение передачи
    I2C2->CR2 |= I2C_CR2_STOP;

    return data;
}

void BMP280_ReadCalibrationData(void) {
    calib_data.dig_T1 = (I2C2_ReadByte(BMP280_I2C_ADDRESS, 0x89) << 8) | I2C2_ReadByte(BMP280_I2C_ADDRESS, 0x88);
    calib_data.dig_T2 = (I2C2_ReadByte(BMP280_I2C_ADDRESS, 0x8B) << 8) | I2C2_ReadByte(BMP280_I2C_ADDRESS, 0x8A);
    calib_data.dig_T3 = (I2C2_ReadByte(BMP280_I2C_ADDRESS, 0x8D) << 8) | I2C2_ReadByte(BMP280_I2C_ADDRESS, 0x8C);
}

void BMP280_Init(void) {
    I2C2_Write(BMP280_I2C_ADDRESS, 0xF4, 0x27);
}


int32_t BMP280_ReadTemperature(void) {
    uint8_t msb = I2C2_ReadByte(BMP280_I2C_ADDRESS, 0xFA);
    uint8_t lsb = I2C2_ReadByte(BMP280_I2C_ADDRESS, 0xFB);
    uint8_t xlsb = I2C2_ReadByte(BMP280_I2C_ADDRESS, 0xFC);

    int32_t adc_T = (msb << 12) | (lsb << 4) | (xlsb >> 4);

    int32_t var1, var2, T;
    // Убедитесь, что calib_data.dig_T1, dig_T2, dig_T3 правильно инициализированы
    var1 = ((((adc_T >> 3) - (calib_data.dig_T1 << 1)) * calib_data.dig_T2) >> 11);

    // Расчет var2
    int32_t temp = (adc_T >> 4) - (int32_t)calib_data.dig_T1; // Временная переменная для промежуточного результата
    var2 = (((temp * temp) >> 12) * (int32_t)calib_data.dig_T3) >> 14;

    // Объединение var1 и var2
    T = (var1 + var2) / 5120;  // Температура в градусах Цельсия * 100
    return T;
}


///////////////////////////////////////////////////////////////////////////
























#endif /* INC_I2C_H_ */
