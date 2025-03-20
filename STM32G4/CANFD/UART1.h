/*
 * UART3.h
 *
 *  Created on: Mar 3, 2025
 *      Author: oleksii
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32g4xx.h"

// Буферы для DMA
#define RX_BUFFER_SIZE 256
volatile uint8_t rxBuffer[RX_BUFFER_SIZE]; // Кольцевой буфер для приёма
volatile uint8_t txBuffer[] = "RS-422 DMA Test\r\n"; // Тестовая строка для передачи
volatile uint32_t rxHead = 0; // Указатель на новые данные в кольцевом буфере


// Инициализация UART3 с DMA
void UART1_Init(void) {

    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;

    USART1->CR1 &= ~USART_CR1_UE; // Отключаем перед настройкой
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE; // TX и RX
    USART1->CR2 = 0; // 1 стоп-бит
    USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR; // Включаем DMA для TX и RX
    USART1->BRR = 1389; // 115200 бод при 160 МГц
    USART1->CR1 |= USART_CR1_UE; // Включаем UART
}

// Отправка данных через DMA
void UART1_SendDMA(uint8_t *data, uint16_t size) {

    while (DMA1_Channel3->CCR & DMA_CCR_EN); // Ждём завершения предыдущей передачи
    DMA1_Channel3->CCR &= ~DMA_CCR_EN; // Отключаем канал
    DMA1_Channel3->CMAR = (uint32_t)data; // Новый буфер
    DMA1_Channel3->CNDTR = size; // Новый размер
    DMA1_Channel3->CCR |= DMA_CCR_EN; // Запускаем передачу

}

// Обработка принятых данных

void UART1_ProcessRX(void) {
    uint32_t tail = RX_BUFFER_SIZE - DMA1_Channel2->CNDTR; // Текущая позиция DMA
    while (rxHead != tail) { // Есть новые данные
        uint8_t received = rxBuffer[rxHead]; // Читаем байт
        UART1_SendDMA(&received, 1); // Эхо: отправляем обратно
        rxHead = (rxHead + 1) % RX_BUFFER_SIZE; // Обновляем указатель
    }
}



#endif /* INC_UART_H_ */
