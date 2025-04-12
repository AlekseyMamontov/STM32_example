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

    I2C2->CR1 |= I2C_CR1_RXDMAEN | I2C_CR1_ERRIE | I2C_CR1_TXIE | I2C_CR1_TCIE; // TXIE для прерываний
    NVIC_EnableIRQ(I2C2_EV_IRQn);
    NVIC_EnableIRQ(I2C2_ER_IRQn);
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
