/*
 * UART3.h
 *
 *  Created on: Mar 3, 2025
 *      Author: oleksii
 */

#ifndef INC_UART1_H_
#define INC_UART1_H_

#include "stm32g4xx.h"
#include "string.h"

// Буферы для DMA
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 32
#define MESSAGE_BUFFER_SIZE 64 // Для сообщений до 22 байт (например, 0x28)

volatile uint8_t rxBuffer[RX_BUFFER_SIZE]; // Буфер для приёма (DMA RX)
volatile uint8_t txBuffer[TX_BUFFER_SIZE]; // Буфер для отправки (DMA TX)
volatile uint8_t messageBuffer[MESSAGE_BUFFER_SIZE]; // Буфер для сообщения
volatile uint8_t messageLength = 0; // Длина скопированного сообщения
volatile bool messageReceived = false; // Флаг завершения сообщения
volatile bool txBusy = false; // Флаг занятости TX DMA
volatile uint32_t rangeData[3] = {0}; // Дальность для 1-й, 2-й, 3-й целей (в 0.1 м)
volatile uint8_t rangeFlags = 0; // Флаги D9
volatile uint32_t rxHead = 0; // Индекс чтения в rxBuffer

// Callback для завершения передачи
void (*txCompleteCallback)(void) = NULL;

// Прототипы функций
void UART1_Init(void);
bool UART1_SendCommand(uint8_t command, uint8_t *params, uint8_t paramLen);
bool UART1_SendDMA(uint8_t *data, uint16_t size, void (*callback)(void));
bool UART1_ParseMessage(void);

// Инициализация UART1 с DMA для RS422
void UART1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // UART1
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMAMUX1EN; // DMA2 и DMAMUX
    // RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // GPIOB (закомментировано)

    /*
    GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOB->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
    GPIOB->AFR[0] |= (7 << GPIO_AFRL_AFSEL6_Pos) | (7 << GPIO_AFRL_AFSEL7_Pos);
    */

    USART1->CR1 &= ~USART_CR1_UE;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE; // Включаем прерывание IDLE
    USART1->CR2 = 0;
    USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
    USART1->BRR = 1389; // 160 МГц / 115200

    DMAMUX1_Channel6->CCR = 24 | DMAMUX_CxCR_EGE; // UART1_RX
    DMAMUX1_Channel7->CCR = 25; // UART1_TX

    // Сброс флагов для DMA2_Channel1 (UART1_RX) и DMA2_Channel2 (UART1_TX)
    DMA2->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    DMA2_Channel1->CCR &= ~DMA_CCR_EN;
    DMA2_Channel1->CPAR = (uint32_t)&(USART1->RDR);
    DMA2_Channel1->CMAR = (uint32_t)rxBuffer;
    DMA2_Channel1->CNDTR = RX_BUFFER_SIZE;
    DMA2_Channel1->CCR = DMA_CCR_MINC
    		           | DMA_CCR_CIRC
    		           | DMA_CCR_TEIE
    		           | (2 << DMA_CCR_PL_Pos);

    DMA2_Channel1->CCR |= DMA_CCR_EN;

    DMA2_Channel2->CCR &= ~DMA_CCR_EN;
    DMA2_Channel2->CPAR = (uint32_t)&(USART1->TDR);
    DMA2_Channel2->CMAR = 0;
    DMA2_Channel2->CNDTR = 0;
    DMA2_Channel2->CCR = DMA_CCR_MINC
    				   | DMA_CCR_DIR
    				   | (2 << DMA_CCR_PL_Pos);

    NVIC_EnableIRQ(DMA2_Channel1_IRQn); // Прерывание для RX DMA
    NVIC_EnableIRQ(DMA2_Channel2_IRQn); // Прерывание для TX DMA
    NVIC_EnableIRQ(USART1_IRQn); // Прерывание UART1 (IDLE)
    USART1->CR1 |= USART_CR1_UE;
}

///////// Отправка команды дальномеру

bool UART1_SendCommand(uint8_t command, uint8_t *params, uint8_t paramLen) {

    if (txBusy || paramLen + 4 > TX_BUFFER_SIZE) {
        return false;
    }

    txBuffer[0] = 0x55; // Заголовок
    txBuffer[1] = command; // Команда
    txBuffer[2] = paramLen; // Длина параметров

    for (uint8_t i = 0; i < paramLen; i++) {
        txBuffer[3 + i] = params[i]; // Параметры
    }

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 3 + paramLen; i++) {
        checksum ^= txBuffer[i];
    }

    txBuffer[3 + paramLen] = checksum; // XOR

    return UART1_SendDMA((uint8_t*)txBuffer, 4 + paramLen, NULL);

}

// Парсинг сообщения (вызывается в основном цикле)

bool UART1_ParseMessage(void) {

    if (!messageReceived || messageLength < 6) {
        messageReceived = false;
        messageLength = 0;
        return false;
    }

    // Проверяем заголовок
    if (messageBuffer[0] != 0x55) {
        messageReceived = false;
        messageLength = 0;
        return false;
    }

    // Проверяем длину
    uint8_t len = messageBuffer[2];
    if (messageLength != len + 4) {
        messageReceived = false;
        messageLength = 0;
        return false;
    }

    // Проверяем контрольную сумму
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len + 3; i++) {
        checksum ^= messageBuffer[i];
    }
    if (checksum != messageBuffer[len + 3]) {
        messageReceived = false;
        messageLength = 0;
        return false;
    }

    // Парсим данные для 0x01/0x02
    uint8_t cmd = messageBuffer[1];
    if ((cmd == 0x01 || cmd == 0x02) && len >= 10) {
        rangeFlags = messageBuffer[3]; // D9
        rangeData[0] = (messageBuffer[4] << 16) | (messageBuffer[5] << 8) | messageBuffer[6]; // 1-я цель
        rangeData[1] = (messageBuffer[7] << 16) | (messageBuffer[8] << 8) | messageBuffer[9]; // 2-я цель
        rangeData[2] = (messageBuffer[10] << 16) | (messageBuffer[11] << 8) | messageBuffer[12]; // 3-я цель
    }
    // Другие команды считаются валидными, но не парсятся

    messageReceived = false;
    messageLength = 0;
    return true;
}

// Обработчик прерывания UART1 (IDLE)

void USART1_IRQHandler(void) {

    if (USART1->ISR & USART_ISR_IDLE) {
        USART1->ICR = USART_ISR_IDLE; // Сбрасываем флаг IDLE

        // Проверяем ошибки ORE/FE
        if (USART1->ISR & (USART_ISR_ORE | USART_ISR_FE)) {
            USART1->ICR = USART_ISR_ORE | USART_ISR_FE;
            rxHead = RX_BUFFER_SIZE - DMA2_Channel1->CNDTR; // Сбрасываем при ошибке
            return;
        }

        uint32_t tail = RX_BUFFER_SIZE - DMA2_Channel1->CNDTR;
        // Проверяем минимальную длину (6 байт)
        if ((tail >= rxHead ? tail - rxHead : RX_BUFFER_SIZE - rxHead + tail) < 6) {
            rxHead = tail; // Недостаточно данных
            return;
        }

        // Проверяем заголовок
        if (rxBuffer[rxHead] != 0x55) {
            rxHead = tail; // Неверный заголовок
            return;
        }

        // Проверяем длину сообщения
        uint8_t len = rxBuffer[(rxHead + 2) % RX_BUFFER_SIZE];
        uint32_t msgLen = len + 4; // Заголовок, команда, длина, данные, XOR

        if (msgLen > MESSAGE_BUFFER_SIZE ||
            (tail >= rxHead ? tail - rxHead : RX_BUFFER_SIZE - rxHead + tail) < msgLen) {
            rxHead = tail; // Недостаточно данных или переполнение буфера
            return;
        }

        // Копируем сообщение в messageBuffer
        for (uint8_t i = 0; i < msgLen; i++) {
            messageBuffer[i] = rxBuffer[(rxHead + i) % RX_BUFFER_SIZE];
        }
        messageLength = msgLen;
        rxHead = (rxHead + msgLen) % RX_BUFFER_SIZE;
        messageReceived = true;
    }
}

// Обработчик прерывания DMA RX

void DMA2_Channel1_IRQHandler(void) {

    if (DMA2->ISR & DMA_ISR_TEIF1) {

        DMA2->IFCR = DMA_IFCR_CGIF1; // Сбрасываем флаги для канала 1
        DMA2_Channel1->CCR &= ~DMA_CCR_EN;
        DMA2_Channel1->CNDTR = RX_BUFFER_SIZE;
        DMA2_Channel1->CCR |= DMA_CCR_EN;
        rxHead = 0; // Сбрасываем индекс при ошибке
        messageLength = 0;

    }
}

// Отправка данных через DMA

bool UART1_SendDMA(uint8_t *data, uint16_t size, void (*callback)(void)) {

    if (txBusy || size > TX_BUFFER_SIZE) {
        return false;
    }

    txBusy = true;
    txCompleteCallback = callback;

    DMA2_Channel2->CCR &= ~DMA_CCR_EN;
    DMA2_Channel2->CMAR = (uint32_t)data;
    DMA2_Channel2->CNDTR = size;
    DMA2_Channel2->CCR |= DMA_CCR_EN;

    return true;
}

// Обработчик прерывания DMA TX
void DMA2_Channel2_IRQHandler(void) {
    if (DMA2->ISR & DMA_ISR_TCIF2) {
        DMA2->IFCR = DMA_IFCR_CGIF2; // Сбрасываем флаги для канала 2
        txBusy = false;
        if (txCompleteCallback) {
            txCompleteCallback();
        }
    }
    if (DMA2->ISR & DMA_ISR_TEIF2) {
        DMA2->IFCR = DMA_IFCR_CGIF2; // Сбрасываем флаги для канала 2
        txBusy = false;
    }
}

#endif /* INC_UART1_H_ */
/*

UART1 на мікроконтролері STM32G431, описаний у документі RM0440, використовує набір регістрів для керування універсальним синхронно-асинхронним приймачем-передавачем (USART/UART). Ці регістри дозволяють налаштовувати параметри зв’язку, керувати передачею та прийомом даних, а також обробляти переривання. Нижче наведено опис ключових регістрів UART1, їх функцій, адрес та значень скидання, заснований на наданому документі, українською мовою.

---

### 1. **Огляд регістрів UART1**
Регістри UART1 розташовані в адресному просторі периферійного пристрою і використовуються для:
- Налаштування параметрів зв’язку (швидкість передачі, довжина слова, стоп-біти, парність).
- Керування передачею та прийомом даних.
- Обробки помилок і переривань.
- Налаштування FIFO (якщо використовується).
- Підтримки додаткових функцій, таких як LIN, Smartcard, IrDA та Modbus.

Таблиця регістрів UART1 подана в розділі 37.8.15 документа (сторінки 89-90). Усі регістри мають базову адресу, до якої додається зсув (offset). Значення скидання вказані для кожного регістру.

---

### 2. **Основні регістри UART1**
Нижче описано ключові регістри UART1, їх призначення, адреси та значення скидання:

#### **USART_CR1 (Регістр керування 1)**
- **Адреса**: Базова адреса + 0x00
- **Значення скидання**: 0x0000 0000
- **Опис**: Основний регістр керування для вмикання UART, налаштування довжини слова, увімкнення переривань та активації FIFO.
- **Ключові біти**:
  - **UE (Біт 0)**: Увімкнення UART (1 = увімкнено).
  - **RE (Біт 2)**: Увімкнення приймача.
  - **TE (Біт 3)**: Увімкнення передавача.
  - **M[1:0] (Біти 28, 12)**: Довжина слова:
    - `00`: 8 біт.
    - `01`: 9 біт.
    - `10`: 7 біт.
  - **FIFOEN (Біт 29)**: Увімкнення режиму FIFO.
  - **RXNEIE, TXEIE, TCIE**: Біти для увімкнення переривань (прийом, передача, завершення передачі).
  - **PEIE**: Переривання за помилкою парності.

#### **USART_CR2 (Регістр керування 2)**
- **Адреса**: Базова адреса + 0x04
- **Значення скидання**: 0x0000 0000
- **Опис**: Налаштування додаткових параметрів, таких як стоп-біти, режими LIN, керування потоком і порядок передачі даних.
- **Ключові біти**:
  - **STOP[1:0] (Біти 13:12)**: Кількість стоп-бітів:
    - `00`: 1 стоп-біт.
    - `01`: 0.5 стоп-біта (для Smartcard).
    - `10`: 2 стоп-біти.
    - `11`: 1.5 стоп-біта (для Smartcard).
  - **MSBFIRST (Біт 19)**: Порядок передачі (1 = MSB першим, 0 = LSB першим).
  - **LINEN (Біт 14)**: Увімкнення режиму LIN.
  - **CLKEN (Біт 11)**: Увімкнення синхронного режиму (виведення тактового сигналу на CK).

#### **USART_CR3 (Регістр керування 3)**
- **Адреса**: Базова адреса + 0x08
- **Значення скидання**: 0x0000 0000
- **Опис**: Налаштування апаратного керування потоком, DMA, FIFO та інших функцій.
- **Ключові біти**:
  - **DMAR (Біт 6)**: Увімкнення DMA для прийому.
  - **DMAT (Біт 7)**: Увімкнення DMA для передачі.
  - **CTSE (Біт 9)**: Увімкнення CTS (апаратне керування потоком).
  - **RTSE (Біт 8)**: Увімкнення RTS.
  - **RXFTCFG/TXFTCFG**: Налаштування порогів FIFO для переривань.
  - **OVRDIS (Біт 12)**: Вимкнення виявлення переповнення (для придушення помилки ORE).

#### **USART_BRR (Регістр швидкості передачі)**
- **Адреса**: Базова адреса + 0x0C
- **Значення скидання**: 0x0000 0000
- **Опис**: Встановлює дільник для генерації швидкості передачі (baud rate).
- **Формула**:
  ```
  Baud Rate = f_CK / USARTDIV
  ```
  де `f_CK` — частота ядра (usart_ker_ck), а `USARTDIV` — значення, записане в регістр BRR.
- **Біти**:
  - **BRR[15:0]**: Дільник для встановлення швидкості передачі. Значення залежить від частоти тактування та бажаної швидкості (наприклад, 9600 бод, 115200 бод).

#### **USART_TDR (Регістр даних передачі)**
- **Адреса**: Базова адреса + 0x28
- **Значення скидання**: 0x0000 0000
- **Опис**: Регістр для запису даних, які будуть передані.
- **Біти**:
  - **TDR[8:0]**: Дані для передачі (7, 8 або 9 біт залежно від налаштування M[1:0] у CR1).
- **Примітка**: Дані записуються в TDR, коли прапорець TXE/TXFNF = 1. Якщо увімкнено FIFO, дані додаються до черги TXFIFO.

#### **USART_RDR (Регістр даних прийому)**
- **Адреса**: Базова адреса + 0x24
- **Значення скидання**: 0x0000 0000
- **Опис**: Регістр для читання отриманих даних.
- **Біти**:
  - **RDR[8:0]**: Отримані дані (7, 8 або 9 біт).
- **Примітка**: Читання RDR очищає прапорець RXNE. Якщо увімкнено FIFO, дані зчитуються з RXFIFO.

#### **USART_ISR (Регістр стану та переривань)**
- **Адреса**: Базова адреса + 0x1C
- **Значення скидання**: Залежить від стану (наприклад, TXE = 1 при скиданні)
- **Опис**: Містить прапорці стану та переривань, такі як готовність даних, помилки та події.
- **Ключові біти**:
  - **RXNE (Біт 5)**: Дані готові до читання (1 = дані в RDR або RXFIFO).
  - **TXE (Біт 7)**: Регістр передачі порожній (без FIFO).
  - **TXFNF (Біт 7, у режимі FIFO)**: TXFIFO не повний.
  - **TC (Біт 6)**: Передачу завершено.
  - **ORE (Біт 3)**: Помилка переповнення.
  - **NE (Біт 2)**: Виявлення шуму.
  - **FE (Біт 1)**: Помилка кадру.
  - **PE (Біт 0)**: Помилка парності.
  - **LBD (Біт 8)**: Виявлення LIN Break.
  - **RTOF (Біт 11)**: Тайм-аут приймача.

#### **USART_ICR (Регістр очищення прапорців переривань)**
- **Адреса**: Базова адреса + 0x20
- **Значення скидання**: 0x0000 0000
- **Опис**: Використовується для скидання прапорців переривань у USART_ISR.
- **Ключові біти**:
  - **PECF (Біт 0)**: Очищення прапорця помилки парності.
  - **FECF (Біт 1)**: Очищення прапорця помилки кадру.
  - **NECF (Біт 2)**: Очищення прапорця шуму.
  - **ORECF (Біт 3)**: Очищення прапорця переповнення.
  - **IDLECF (Біт 4)**: Очищення прапорця простою.
  - **TCCF (Біт 6)**: Очищення прапорця завершення передачі.
  - **LBDCF (Біт 8)**: Очищення прапорця LIN Break.
  - **RTOCF (Біт 11)**: Очищення прапорця тайм-ауту.

#### **USART_RQR (Регістр запитів)**
- **Адреса**: Базова адреса + 0x18
- **Значення скидання**: 0x0000 0000
- **Опис**: Використовується для запитів певних дій, таких як надсилання Break або скидання даних.
- **Ключові біти**:
  - **SBKRQ (Біт 1)**: Запит надсилання Break.
  - **RXFRQ (Біт 3)**: Запит очищення RXFIFO.
  - **TXFRQ (Біт 4)**: Запит очищення TXFIFO.

#### **USART_PRESC (Регістр попереднього дільника)**
- **Адреса**: Базова адреса + 0x2C
- **Значення скидання**: 0x0000 0000
- **Опис**: Налаштування попереднього дільника для тактування UART.
- **Біти**:
  - **PRESC[3:0]**: Коефіцієнт попереднього дільника (від 1 до 16).
- **Примітка**: Регістр записується лише при вимкненому UART (UE = 0).

---

### 3. **Особливості UART1**
UART1 підтримує розширені функції, включаючи:
- **FIFO**: 8-байтові буфери TXFIFO та RXFIFO для передачі та прийому.
- **Довжина слова**: 7, 8 або 9 біт.
- **Швидкість передачі**: Налаштовується через USART_BRR з використанням дробового дільника.
- **Режими**:
  - Асинхронний UART.
  - Синхронний режим (Master/Slave).
  - Smartcard (T=0, T=1).
  - LIN (13-бітний Break, виявлення Break).
  - IrDA SIR.
  - Modbus (тайм-аут, розпізнавання CR/LF).
- **DMA**: Підтримка передачі та прийому через DMA для багатобуферного зв’язку.
- **Апаратне керування потоком**: CTS/RTS для RS232, DE для RS485.
- **Переривання**: Підтримка багатьох джерел переривань (помилки, завершення передачі, тайм-аут тощо).

---

### 4. **Процедура налаштування UART1**
Для налаштування UART1 необхідно:
1. Налаштувати довжину слова в **USART_CR1** (біти M[1:0]).
2. Встановити швидкість передачі в **USART_BRR**.
3. Задати кількість стоп-бітів у **USART_CR2**.
4. Увімкнути UART, встановивши **UE = 1** у **USART_CR1**.
5. (Опціонально) Увімкнути DMA у **USART_CR3** для багатобуферної передачі.
6. Встановити **TE = 1** для початку передачі (надсилання idle-кадру).
7. Записувати дані в **USART_TDR** для передачі, перевіряючи прапорець **TXE** або **TXFNF**.
8. Читати дані з **USART_RDR** при встановленому прапорці **RXNE**.

---

### 5. **Примітки**
- **FIFO**: Якщо FIFO увімкнено (FIFOEN = 1), використовуйте прапорці TXFNF та RXFT для керування передачею та прийомом.
- **Помилки**: Обробляйте помилки (ORE, NE, FE, PE) через **USART_ISR** і скидайте їх у **USART_ICR**.
- **Синхронний режим**: Для синхронного режиму налаштуйте виведення тактового сигналу (CK) через **USART_CR2**.
- **LIN/Smartcard**: Ці режими доступні лише для USART1/2/3, але не для UART4/5 або LPUART1 (див. таблицю 345, сторінка 6).

---

Якщо потрібен детальніший опис окремого регістру, приклад коду для налаштування UART1 або переклад конкретних частин документа, повідомте!

 INC_UART_H_ */
