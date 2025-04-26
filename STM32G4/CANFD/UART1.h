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

#define TX_MSG_SEND 1

#define RX_MSG_READ 1
#define RX_MSG_NEW 2
#define RX_MSG_ERR 4
#define RX_MSG_FULL 8

#define RS422_rxDMA    DMA2_Channel1
#define RS422_rxDMAMUX DMAMUX1_Channel6
#define RS422_txDMA    DMA2_Channel2
#define RS422_txDMAMUX DMAMUX1_Channel7

#define RS422_UART USART1
#define RS422_RCC_UART  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
#define RS422_RCC_DMA   RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMAMUX1EN;

#define RS422_RX_IRQ DMA2_Channel1_IRQn
#define RS422_TX_IRQ DMA2_Channel2_IRQn
#define RS422_UART_IRQ USART1_IRQn



struct RS422_driver{

		uint8_t  *status_rxDMA;
		uint8_t  *rx_buffer;
		uint8_t *rx_w_index;
		uint8_t *rx_r_index;

		uint8_t *mesg_buf;
		uint8_t *n_mesg_buf;

		uint8_t  *status_txDMA;
		uint8_t  *tx_buffer;
		uint8_t *tx_w_index;
		uint8_t *tx_r_index;

		uint16_t rx_size_buffer;
		uint16_t tx_size_buffer;

};

uint8_t status_rxDMA = 0,
		status_txDMA = 0,
        UART1_rxBuffer[RX_BUFFER_SIZE], // Буфер для приёма (DMA RX)
        UART1_txBuffer[TX_BUFFER_SIZE], // Буфер для отправки (DMA TX)
        RS422_messageBuffer[MESSAGE_BUFFER_SIZE],
        messageLength = 0;             // Длина скопированного сообщения


bool txBusy = false; // Флаг занятости TX DMA
uint32_t rangeData[3] = {0}; // Дальность для 1-й, 2-й, 3-й целей (в 0.1 м)
uint8_t rangeFlags = 0; // Флаги D9
uint32_t rxHead = 0; // Индекс чтения в rxBuffer



struct RS422_driver dalnomer={

		.status_rxDMA = &status_rxDMA,

		.rx_buffer =  UART1_rxBuffer,
		.rx_r_index = UART1_rxBuffer,
		.rx_w_index = UART1_rxBuffer,
		.rx_size_buffer = RX_BUFFER_SIZE,

		.mesg_buf = UART1_rxBuffer,
		.n_mesg_buf = &messageLength,


		.status_txDMA = &status_txDMA,

		.tx_buffer =  UART1_txBuffer,
		.tx_r_index = UART1_txBuffer,
		.tx_w_index = UART1_txBuffer,
		.tx_size_buffer = TX_BUFFER_SIZE,

};


// Callback для завершения передачи
void (*txCompleteCallback)(void) = NULL;

// Прототипы функций
//void UART1_Init(struct RS422_driver *);

//bool UART1_SendCommand(struct RS422_driver * , uint8_t command, uint8_t *params, uint8_t paramLen);
//bool UART1_SendDMA(struct RS422_driver * , uint8_t *data, uint16_t size, void (*callback)(void));

//bool UART1_ParseMessage(void);

// Инициализация UART1 с DMA для RS422

void UART1_Init(struct RS422_driver *rs){

	RS422_RCC_UART
	RS422_RCC_DMA

    RS422_UART->CR1 &= ~USART_CR1_UE;
    RS422_rxDMA->CCR &= ~DMA_CCR_EN;
    RS422_txDMA->CCR &= ~DMA_CCR_EN;

    RS422_UART->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE; // Включаем прерывание IDLE
    RS422_UART->CR2 = 0;
    RS422_UART->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
    RS422_UART->BRR = 1389; // 160 МГц / 115200

    RS422_rxDMAMUX->CCR = 24 | DMAMUX_CxCR_EGE; // UART1_RX
    RS422_txDMAMUX->CCR = 25;                   // UART1_TX

    // Сброс флагов для DMA2_Channel1 (UART1_RX) и DMA2_Channel2 (UART1_TX)
    DMA2->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

    RS422_rxDMA->CPAR = (uint32_t)&(RS422_UART->RDR);
    RS422_rxDMA->CMAR = (uint32_t)rs->rx_buffer;
    RS422_rxDMA->CNDTR = rs->rx_size_buffer;
    RS422_rxDMA->CCR = DMA_CCR_MINC |
    				   DMA_CCR_CIRC |
    				   DMA_CCR_TEIE |
    				   (2 << DMA_CCR_PL_Pos);

    RS422_txDMA->CPAR = (uint32_t)&(RS422_UART->TDR);
    RS422_txDMA->CMAR =  0;
    RS422_txDMA->CNDTR = 0;
    RS422_txDMA->CCR = DMA_CCR_MINC |
    				   DMA_CCR_DIR  |
    				   (2 << DMA_CCR_PL_Pos);



    NVIC_EnableIRQ(RS422_RX_IRQ); // Прерывание для RX DMA
    NVIC_EnableIRQ(RS422_TX_IRQ); // Прерывание для TX DMA
    NVIC_EnableIRQ(RS422_UART_IRQ);        // Прерывание UART1 (IDLE)

    RS422_rxDMA->CCR |= DMA_CCR_EN;
    RS422_UART->CR1 |= USART_CR1_UE;
}

// Отправка команды дальномерy
bool UART1_SendDMA(struct RS422_driver* rs, uint8_t *data, uint16_t size) {

    if (*(rs->status_txDMA) & TX_MSG_SEND || size > rs->tx_size_buffer) {
        return false;
    }

    RS422_txDMA->CCR &= ~DMA_CCR_EN;
    RS422_txDMA->CMAR = (uint32_t)data;
    RS422_txDMA->CNDTR = size;
    RS422_txDMA->CCR |= DMA_CCR_EN;

    return true;
}

bool UART1_SendCommand(struct RS422_driver* rs,uint8_t command, uint8_t *params, uint8_t paramLen) {

    uint8_t checksum = 0;

    if (*(rs->status_txDMA) & TX_MSG_SEND || paramLen + 4 > rs->tx_size_buffer)  return false;

    rs->tx_buffer[0] = 0x55;    // Заголовок
    rs->tx_buffer[1] = command; // Команда
    rs->tx_buffer[2] = paramLen;// Длина параметров

    for (uint8_t i = 0; i < paramLen; i++){rs->tx_buffer[3 + i] = params[i];} // Параметры

    for (uint8_t i = 0; i < 3 + paramLen; i++){checksum ^= rs->tx_buffer[i];}

    rs->tx_buffer[3 + paramLen] = checksum; // XOR

    return UART1_SendDMA(rs,(uint8_t*)rs->tx_buffer, 4 + paramLen);
}


// Обработчик прерывания UART1 (IDLE)
void USART1_IRQHandler(void) {

	uint16_t n_byte = 0;


/////// error

    if (RS422_UART->ISR & (USART_ISR_ORE | USART_ISR_FE)) {

    	RS422_UART->ICR =  USART_ISR_ORE | USART_ISR_FE;
    	RS422_rxDMA->CCR &= ~DMA_CCR_EN;
    	RS422_rxDMA->CNDTR = dalnomer.rx_size_buffer;
    	RS422_rxDMA->CCR |=  DMA_CCR_EN;

        dalnomer.rx_r_index = dalnomer.rx_w_index = dalnomer.rx_buffer;
        messageLength = 0;
        return;
    }



////// n_bayt ?

    if (USART1->ISR & USART_ISR_IDLE) {

    	  USART1->ICR = USART_ISR_IDLE;

    	  dalnomer.rx_w_index = (uint8_t*) RS422_rxDMA->CMAR;

///// n_byte

    	  if(dalnomer.rx_r_index <= dalnomer.rx_w_index){

    		  n_byte = dalnomer.rx_w_index - dalnomer.rx_r_index;

    	  }else{

    		  n_byte = (dalnomer.rx_size_buffer + dalnomer.rx_buffer) - dalnomer.rx_r_index;
    		  n_byte += (dalnomer.rx_buffer - dalnomer.rx_w_index);

    	  };

 //// block for test

    	  if( n_byte < 6 || *(dalnomer.rx_r_index) != 0x55 ){

    		  dalnomer.rx_r_index = dalnomer.rx_w_index;
    		  *(dalnomer.status_rxDMA) |= RX_MSG_ERR;
    		  return;
    	  }

    	  if( *(dalnomer.status_rxDMA) & RX_MSG_NEW){

    		  dalnomer.rx_r_index = dalnomer.rx_w_index;
    		  *(dalnomer.status_rxDMA) |= RX_MSG_FULL;
    		  return;
    	  }


    	  //

    	  *(dalnomer.status_rxDMA) &= ~(RX_MSG_ERR|RX_MSG_FULL) ;
    	  *(dalnomer.status_rxDMA) |=RX_MSG_NEW;
    	  dalnomer.mesg_buf = dalnomer.rx_r_index;
    	  *(dalnomer.n_mesg_buf) = n_byte;

     }
}

/*

Интерфейс: RS422, 115200 бод, 8 бит данных, 1 стартовый бит, 1 стоповый бит,
без проверки чётности.
Протокол: Master -> slave .
STX0: Заголовок, 0x55 .
CMD:  Команда или код ответа.
LEN:  Длина данных (байт).
DATA: Параметры или данные ответа.
CHK:  Контрольная сумма (XOR всех байтов, кроме CHK).

 */



uint8_t Dalnomer_Message(struct RS422_driver *rs) {

	uint8_t*data = rs->mesg_buf,
		   *end_buf = rs->rx_buffer + rs->rx_size_buffer,
		   checksum = 0, len = *rs->n_mesg_buf;

	if(!(*(rs->status_rxDMA)& RX_MSG_NEW)) return 1; //  new msg?

	if (len > 22) len = 22;

	for(uint8_t i=0; i< len;i++){
		RS422_messageBuffer[i] = *data++;
		if(data >= end_buf) data = rs->rx_buffer;
	}

	*rs->status_rxDMA &= ~RX_MSG_NEW;

	if(RS422_messageBuffer[0] != 0x55) return 2;

	// Проверяем контрольную сумму

		len  = RS422_messageBuffer[2];
	    for (uint8_t i = 0; i < len + 3; i++)checksum ^= RS422_messageBuffer[i];
	    if (checksum != RS422_messageBuffer[len + 3]) return 3;


	switch(RS422_messageBuffer[1]){

		case 0:

		break;

		case 1:
			rangeFlags =    RS422_messageBuffer[3]; // D9
			rangeData[0] = (RS422_messageBuffer[4] << 16)|(RS422_messageBuffer[5] << 8) |
					        RS422_messageBuffer[6];//  1_
			rangeData[1] = (RS422_messageBuffer[7] << 16) | (RS422_messageBuffer[8] << 8)|
							RS422_messageBuffer[9]; // 2
			rangeData[2] = (RS422_messageBuffer[10] << 16) | (RS422_messageBuffer[11] << 8)|
							RS422_messageBuffer[12]; // 3-

		case 2:
			rangeFlags =    RS422_messageBuffer[3]; // D9
			rangeData[0] = (RS422_messageBuffer[4] << 16)|(RS422_messageBuffer[5] << 8) |
					        RS422_messageBuffer[6];//  1_
			rangeData[1] = (RS422_messageBuffer[7] << 16) | (RS422_messageBuffer[8] << 8)|
							RS422_messageBuffer[9]; // 2
			rangeData[2] = (RS422_messageBuffer[10] << 16) | (RS422_messageBuffer[11] << 8)|
							RS422_messageBuffer[12]; // 3-
		break;



		default: break;

	};


    return 0;
}



// Обработчик прерывания DMA RX

void DMA2_Channel1_IRQHandler(void) {

    if (DMA2->ISR & DMA_ISR_TEIF1) {

        DMA2_Channel1->CCR &= ~DMA_CCR_EN;
        DMA2_Channel1->CNDTR = RX_BUFFER_SIZE;
        DMA2_Channel1->CCR |= DMA_CCR_EN;
        rxHead = 0; // Сбрасываем индекс при ошибке

        messageLength = 0;

        DMA2->IFCR = DMA_IFCR_CGIF1;// Сбрасываем флаги для канала 1


    }
}
/*
// Отправка данных через DMA


// Обработчик прерывания DMA TX

void DMA2_Channel2_IRQHandler(void) {

    if (DMA2->ISR & DMA_ISR_TCIF2) {

        txBusy = false;

        if (txCompleteCallback) txCompleteCallback();
    }

    if (DMA2->ISR & DMA_ISR_TEIF2)  txBusy = false;

    DMA2->IFCR = DMA_IFCR_CGIF2;
}
*/
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
