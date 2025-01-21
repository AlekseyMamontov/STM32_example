/*
 * CanFD_stm32G4.h
 *
 *  Created on: Jan 17, 2025
 *      Author: oleksii
 */

#ifndef INC_CANFD_STM32G4_H_
#define INC_CANFD_STM32G4_H_
#include "stm32g4xx.h"
#include <math.h>


FDCAN_GlobalTypeDef *FDCAN = FDCAN1;

// RAM  CANFD module
/*
0x4000 AC00 - 0x4000 AFFF1 KB
0x4000 A800 - 0x4000 ABFF1 KB
0x4000 A400 - 0x4000 A7FF1 KB
 */


#define RAMBaseFDCAN1  0x4000A400
#define RAMBaseFDCAN2  0x4000A800
#define RAMBaseFDCAN3  0x4000AC00

// RAM Message

#define Ram11bitfilter 0x0000
#define Ram29bitfilter 0x0070
#define RamFIFO0RX	   0x00B0
#define RamFIFO1RX	   0x0188
#define RamTxeventFIFO 0x0260
#define RamBaseTX      0x0278


#define ESIbit			   0x80000000
#define XTDbit  		   0x40000000
#define RTRbit  		   0x20000000
#define ID_msk  		   0x3FFFFFFF
#define Message_status_msk 0xFF000000
#define EFCbit   0x800000 // FIFO control – 0: Do not store Tx events – 1: Store Tx events
#define FDFbit	 0x200000 // FD format 0: Frame transmitted in classic CAN format /
						  // 1: Frame transmitted in CAN FD format
#define DLC_msk  0xF0000  // 0 - 8: Classic CAN + CAN FD: received frame has 0-8 data bytes
#define DLC_msk_pos 16	  // 9 - 15: Classic CAN: received frame has 8 data bytes
						  // 9 - 15: CAN FD: received frame has 12/16/20/24/32/48/64 data bytes

/*
 *

///////////////////// Header 0

T0 Bit 31 ESI(1)Error state indicator
– 0: ESI bit in CAN FD format depends only on error passive flag
– 1: ESI bit in CAN FD format transmitted recessive
T0 Bit 30 Standard or extended identifier depending on bit XTD
– 0: 11-bit standard identifier
– 1: 29-bit extended identifier
T0 Bit 29 RTR(2) Remote transmission request
– 0: Transmit data frame
– 1: Transmit remote frame
T0 Bits 28:0 . written to ID[28:18].

///////////////////// Header 1

T1 Bits 31:24 Message marker Written by CPU during Tx buffer configuration. Copied into Tx event FIFO element for
MM[7:0] identification of Tx message status.

T1 Bit 23 EFCEvent FIFO control
– 0: Do not store Tx events
– 1: Store Tx events
T1 Bit 21 FDF FD format
– 0: Frame transmitted in classic CAN format
– 1: Frame transmitted in CAN FD format
T1 Bit 20 BRS(3)Bit rate switching
– 0: CAN FD frames transmitted without bit rate switching
– 1: CAN FD frames transmitted with bit rate switching
Data length code DLC[3:0]
T1 Bits 19:16 –
0 - 8: Classic CAN + CAN FD: received frame has 0-8 data bytes
9 - 15: Classic CAN: received frame has 8 data bytes
9 - 15: CAN FD: received frame has 12/16/20/24/32/48/64 data bytes

///////////////////// data

T2 Bits 31:24
Data byte 3
Data byte 2
Data byte 1
n...
 */
struct TX_canFD_message{

	uint32_t Header0;
	uint32_t Header1;
	uint32_t data[16];

};


struct RX_canFD_message{

	uint32_t Header0;
	uint32_t Header1;
	uint32_t data[16];

};



// Функция для отправки CAN сообщения
uint8_t SendCANframe(uint32_t* msg) {

	uint8_t   error = 1, status =0;
    uint32_t  PutIndex;
    uint32_t* TxBuffer;
    //uint32_t *MessageRAMBase = (uint32_t *)RAMBaseFDCAN1; // Адрес начала Message RAM

    // Проверяем, что Tx FIFO не заполнен
    if ((FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) == 0) {


        PutIndex = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos; // Получаем индекс вставки в Tx FIFO
        TxBuffer = (uint32_t *)(RAMBaseFDCAN1 + RamBaseTX + (PutIndex * 18 *4));// Указатель на буфер передачи в Message RAM/
        *TxBuffer ++ = *msg ++;
         status = (*msg ) & FDFbit;
        *TxBuffer ++ = *msg ++; //header1
        *TxBuffer ++ = *msg ++; //data 1-4
        *TxBuffer ++ = *msg ++; //data 5-8
        if(status){
        	for (uint8_t i=0; i<14;i++){*TxBuffer++ = *msg++;}
         };

        FDCAN1->TXBAR = (1 << PutIndex);// Активация соответствующего запроса на передачу
        error = 0;
    };

 return error;
}

uint8_t CAN_SendMessage(uint32_t id,uint8_t* data, uint32_t len){

	struct TX_canFD_message msg;

	uint8_t *ram = (uint8_t*) msg.data;

	msg.Header0  =  (id <= 0x7FF) ? (id << 18) : (id | (1 << 30));
	msg.Header0 |=  (id <= 0x7FF) ?  0: XTDbit; //extendet
	msg.Header1  =  (len << 16);

	for(uint8_t i = 0; i < len; i++) {
	    ram[i] = data[i];
	}
	for(uint8_t i = len; i < 8; i++) {
	    ram[i] = 0; // Заполнение оставшихся байт нулями
	}
return SendCANframe((uint32_t*) &msg);
}


void FDCAN1_IT1_IRQHandler(void) {
    // Проверить, было ли прерывание из FIFO 0
    if (FDCAN1->IR & (1 << FDCAN_IR_RF1N_Pos)) {
        // Обработка полученного сообщения
        // Например, чтение сообщения из FIFO 0

    	 uint32_t getIndex = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;

 /*   	        // Адрес буфера Rx FIFO 0
    	        uint32_t *rxBuffer = (uint32_t *)(BASE_ADDRESS_FOR_RX_FIFO + getIndex * 18 * 4); // Замените BASE_ADDRESS_FOR_RX_FIFO на правильный адрес

    	        // Чтение заголовка сообщения
    	        uint32_t rxHeader0 = rxBuffer[0];
    	        uint32_t rxHeader1 = rxBuffer[1];

    	        // Извлечение полей из заголовка
    	        uint8_t ESI = (rxHeader0 >> 31) & 0x1;
    	        uint8_t XTD = (rxHeader0 >> 30) & 0x1;
    	        uint8_t RTR = (rxHeader0 >> 29) & 0x1;
    	        uint32_t ID = rxHeader0 & 0x1FFFFFFF;

    	        uint8_t ANMF = (rxHeader1 >> 31) & 0x1;
    	        uint8_t FIDX = (rxHeader1 >> 24) & 0x7F;
    	        uint8_t FDF = (rxHeader1 >> 21) & 0x1;
    	        uint8_t BRS = (rxHeader1 >> 20) & 0x1;
    	        uint8_t DLC = (rxHeader1 >> 16) & 0xF;
    	        uint16_t RXTS = rxHeader1 & 0xFFFF;

    	        // Чтение данных сообщения
    	        uint8_t rxData[64];
    	        uint8_t *dataBuffer = (uint8_t *)(rxBuffer + 2); // Указатель на данные
    	        for (int i = 0; i < DLC; i++) {
    	            rxData[i] = dataBuffer[i]; // Чтение данных
    	        }
*/
    	        // Обработка принятого сообщения
    	        // Ваш код обработки здесь

    	        // Обновление индекса подтверждения
    	        FDCAN1->RXF0A = getIndex; // Убедитесь, что это правильный способ записи
    	    }

        // Сбросить флаг прерывания
        FDCAN1->IR |= (1 << FDCAN_IR_RF1N_Pos); // Сбросить флаг нового сообщения
    }














void SetClassicCANBitTime(FDCAN_GlobalTypeDef *FDCANx, uint32_t prescaler, uint32_t ts1, uint32_t ts2, uint32_t sjw) {
    FDCANx->NBTP = ((prescaler - 1) << FDCAN_NBTP_NBRP_Pos) |
                   ((ts1 - 1) << FDCAN_NBTP_NTSEG1_Pos) |
                   ((ts2 - 1) << FDCAN_NBTP_NTSEG2_Pos) |
                   ((sjw - 1) << FDCAN_NBTP_NSJW_Pos);
}

void SetDataCANBitTime(FDCAN_GlobalTypeDef *FDCANx, uint32_t prescaler, uint32_t ts1, uint32_t ts2, uint32_t sjw) {
    FDCANx->DBTP = ((prescaler - 1) << FDCAN_DBTP_DBRP_Pos) |
                   ((ts1 - 1) << FDCAN_DBTP_DTSEG1_Pos) |
                   ((ts2 - 1) << FDCAN_DBTP_DTSEG2_Pos) |
                   ((sjw - 1) << FDCAN_DBTP_DSJW_Pos);
}


#define NOMINAL_TIME_SEG1_MAX 256
#define NOMINAL_TIME_SEG2_MAX 128

typedef struct {
    uint32_t fdcan_core_clock_hz; // Частота тактирования ядра FDCAN в Гц
    uint32_t baud_rate_tolerance; // Допустимое отклонение скорости передачи данных в процентах
    uint32_t nominal_rate_bps; // Номинальная скорость (CLASSIC CAN) в кбит/с
    uint32_t sampling_point_nominal; // Точка выборки для номинального режима в процентах
    uint32_t prescaler; // Заданный предделитель
    uint32_t nsjw; // Значение для переходного сегмента

} FDCAN_BaudRate;

// Функция для расчета параметров FDCAN

uint32_t FDCAN_SetBaudRate(FDCAN_BaudRate* config) {

	    uint32_t tfdcan_tq_clk = config->fdcan_core_clock_hz; // Используем полные числа вместо экспоненциальной записи
	    uint32_t tq_per_bit, tseg1, tseg2;
	    double ftq, tfq_real, freq_diff;

	    // Вычисление тактового сигнала FDCAN
	    ftq = (double)tfdcan_tq_clk / config->prescaler;
	    tq_per_bit = (uint32_t)round(ftq / config->nominal_rate_bps);

	    // Проверка минимального значения tq_per_bit
	    if (tq_per_bit < 4) {
	        // Ошибка: tq_per_bit меньше минимально допустимого значения 4.
	        return 0;
	    }

	    // Вычисление временных сегментов tseg1 и tseg2
	    tseg1 = (uint32_t)round(tq_per_bit * config->sampling_point_nominal / 100);
	    tseg2 = tq_per_bit - tseg1;
	    tseg1 --;

	    // Проверка допустимости значений tseg1 и tseg2
	    if (tseg1 < 1 || tseg1 > NOMINAL_TIME_SEG1_MAX || tseg2 < 1 || tseg2 > NOMINAL_TIME_SEG2_MAX) {
	        // Ошибка: Значения временных сегментов tseg1 или tseg2 выходят за допустимые пределы.
	        return 0;
	    }

	    // Проверка допустимости отклонения частоты передачи данных
	    tfq_real = tq_per_bit * config->nominal_rate_bps;
	    freq_diff = (ftq - tfq_real) * 2 * 100 / (ftq + tfq_real);

	    if (fabs(freq_diff) > config->baud_rate_tolerance) {
	        // Ошибка: Отклонение частоты передачи данных превышает допустимое значение.
	        return 0;
	    }

	    // Формирование значения регистра FDCAN_NBTP
	    uint32_t FDCAN_NBTP = ((config->prescaler - 1) << FDCAN_NBTP_NBRP_Pos) & FDCAN_NBTP_NBRP_Msk;
	    FDCAN_NBTP |= ((tseg1-1) << FDCAN_NBTP_NTSEG1_Pos) & FDCAN_NBTP_NTSEG1_Msk;
	    FDCAN_NBTP |= ((tseg2-1)<< FDCAN_NBTP_NTSEG2_Pos) & FDCAN_NBTP_NTSEG2_Msk;
	    FDCAN_NBTP |= (((config->nsjw ? config->nsjw : 1) - 1) << FDCAN_NBTP_NSJW_Pos) & FDCAN_NBTP_NSJW_Msk;

	    return FDCAN_NBTP;
	}




void FDCAN_INIT(void){

	RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;

    FDCAN1->CCCR |= FDCAN_CCCR_INIT; 			// Войти в режим инициализации
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT));  // Ожидание инициализации

    FDCAN1->CCCR |= FDCAN_CCCR_CCE; // Включить доступ к конфигурационным регис














};



void FDCAN_Config(FDCAN_BaudRate* config) {

    // Включение тактирования для FDCAN и GPIO
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;

    // Выключаем FDCAN перед настройкой
    FDCAN->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN->CCCR & FDCAN_CCCR_INIT));

    // Разрешаем доступ к конфигурационным регистрам
    FDCAN->CCCR |= FDCAN_CCCR_CCE;

    // Установка скорости передачи данных
    FDCAN_SetBaudRate(config);







    // Завершение настройки

    FDCAN->CCCR &= ~FDCAN_CCCR_INIT;
    while (FDCAN->CCCR & FDCAN_CCCR_INIT);

 //   NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
 // NVIC_EnableIRQ(FDCAN1_IT1_IRQn);

}



/*
Регистры включения прерываний FDCAN (FDCAN_IE)
Настройки в регистре включения прерываний определяют, какие изменения статуса в регистре прерываний сигнализируются по линии прерывания.

Смещение адреса: 0x0054
Начальное значение: 0x0000 0000
Биты регистра
Биты 31:24: Зарезервировано, должно сохраняться на начальном значении.
Бит 23 (ARAE): Разрешение доступа к зарезервированному адресу.
Бит 22 (PEDE): Разрешение прерывания при ошибке протокола в фазе данных.
Бит 21 (PEAE): Разрешение прерывания при ошибке протокола в фазе арбитрации.
Бит 20 (WDIE): Разрешение прерывания таймера watchdog.
0: Прерывание отключено
1: Прерывание включено
Бит 19 (BOE): Статус Bus-off.
0: Прерывание отключено
1: Прерывание включено
Бит 18 (EWE): Разрешение прерывания статуса предупреждения.
0: Прерывание отключено
1: Прерывание включено
Бит 17 (EPE): Разрешение прерывания в режиме "ошибка пассивная".
0: Прерывание отключено
1: Прерывание включено
Бит 16 (ELOE): Разрешение прерывания переполнения журналирования ошибок.
0: Прерывание отключено
1: Прерывание включено
Бит 15 (TOOE): Разрешение прерывания при возникновении таймаута.
0: Прерывание отключено
1: Прерывание включено
Бит 14 (MRAFE): Разрешение прерывания при сбое доступа к RAM сообщениям.
0: Прерывание отключено
1: Прерывание включено
Бит 13 (TSWE): Разрешение прерывания при переполнении временной метки.
0: Прерывание отключено
1: Прерывание включено
Бит 12 (TEFLE): Разрешение прерывания при потере элемента в FIFO событий Tx.
0: Прерывание отключено
1: Прерывание включено
Бит 11 (TEFFE): Разрешение прерывания при переполнении FIFO событий Tx.
0: Прерывание отключено
1: Прерывание включено
Бит 10 (TEFNE): Разрешение прерывания при новом элементе в FIFO событий Tx.
0: Прерывание отключено
1: Прерывание включено
Бит 9 (TFEE): Разрешение прерывания при пустом FIFO Tx.
0: Прерывание отключено
1: Прерывание включено
Бит 8 (TCFE): Разрешение прерывания при завершении отмены передачи.
0: Прерывание отключено
1: Прерывание включено
Бит 7 (TCE): Разрешение прерывания при завершении передачи.
0: Прерывание отключено
1: Прерывание включено
Бит 6 (HPME): Разрешение прерывания для сообщений с высоким приоритетом.
0: Прерывание отключено
1: Прерывание включено
Бит 5 (RF1LE): Разрешение прерывания при потере сообщения в FIFO 1 приема.
0: Прерывание отключено
1: Прерывание включено
Бит 4 (RF1FE): Разрешение прерывания при переполнении FIFO 1 приема.
0: Прерывание отключено
1: Прерывание включено
Бит 3 (RF1NE): Разрешение прерывания при новом сообщении в FIFO 1 приема.
0: Прерывание отключено
1: Прерывание включено
Бит 2 (RF0LE): Разрешение прерывания при потере сообщения в FIFO 0 приема.
0: Прерывание отключено
1: Прерывание включено
Бит 1 (RF0FE): Разрешение прерывания при переполнении FIFO 0 приема.
0: Прерывание отключено
1: Прерывание включено
Бит 0 (RF0NE): Разрешение прерывания при новом сообщении в FIFO 0 приема.
0: Прерывание отключено
1: Прерывание включено
*/
/*
Регистры прерываний FDCAN (FDCAN_IR)
Флаги устанавливаются, когда обнаруживаются одни из перечисленных условий (чувствительность к фронту). Флаги остаются установленными до тех пор, пока хост не сбросит их. Флаг сбрасывается записью 1 в соответствующую позицию бита. Запись 0 не имеет эффекта. Жесткий сброс очищает регистр. Конфигурация FDCAN_IE управляет тем, генерируется ли прерывание. Конфигурация FDCAN_ILS управляет тем, на какой линии прерывания сигнализируется прерывание.

Смещение адреса: 0x0050
Начальное значение: 0x0000 0000
Биты регистра
Биты 31:24: Зарезервировано, должно сохраняться на начальном значении.
Бит 23 (ARA): Доступ к зарезервированному адресу.
0: Доступ к зарезервированному адресу не произошел.
1: Произошел доступ к зарезервированному адресу.
Бит 22 (PED): Ошибка протокола в фазе данных (используется время бита данных).
0: Ошибки протокола в фазе данных нет.
1: Обнаружена ошибка протокола в фазе данных.
Бит 21 (PEA): Ошибка протокола в фазе арбитрации (используется номинальное время бита).
0: Ошибки протокола в фазе арбитрации нет.
1: Обнаружена ошибка протокола в фазе арбитрации.
Бит 20 (WDI): Прерывание таймера watchdog.
0: Событие watchdog сообщения RAM не произошло.
1: Произошло событие watchdog сообщения RAM из-за отсутствия READY.
Бит 19 (BO): Статус Bus-off.
0: Статус Bus-off не изменился.
1: Статус Bus-off изменен.
Бит 18 (EW): Статус предупреждения.
0: Статус предупреждения об ошибке не изменился.
1: Статус предупреждения об ошибке изменен.
Бит 17 (EP): Ошибка в пассивном режиме.
0: Статус ошибки в пассивном режиме не изменился.
1: Статус ошибки в пассивном режиме изменен.
Бит 16 (ELO): Переполнение журнала ошибок.
0: Счетчик журналирования ошибок CAN не переполнился.
1: Произошло переполнение счетчика журналирования ошибок CAN.
Бит 15 (TOO): Произошел таймаут.
0: Таймаут не произошел.
1: Таймаут достигнут.
Бит 14 (MRAF): Ошибка доступа к RAM сообщений.
Флаг устанавливается, когда обработчик Rx:
Не завершил фильтрацию или хранение принятого сообщения до получения поля арбитрации следующего сообщения.
Не смог записать сообщение в RAM сообщений.
0: Ошибки доступа к RAM сообщений не произошло.
1: Произошла ошибка доступа к RAM сообщений.
Бит 13 (TSW): Переполнение временной метки.
0: Переполнения счетчика временной метки не произошло.
1: Счетчик временной метки переполнился.
Бит 12 (TEFL): Потеря элемента FIFO событий Tx.
0: Элементы FIFO событий Tx не потеряны.
1: Элемент FIFO событий Tx потерян.
Бит 11 (TEFF): FIFO событий Tx полон.
0: FIFO событий Tx не полон.
1: FIFO событий Tx полон.
Бит 10 (TEFN): Новый элемент в FIFO событий Tx.
0: FIFO событий Tx не изменен.
1: Обработчик Tx записал элемент в FIFO событий Tx.
Бит 9 (TFE): FIFO Tx пуст.
0: FIFO Tx не пуст.
1: FIFO Tx пуст.
Бит 8 (TCF): Завершение отмены передачи.
0: Завершения отмены передачи не произошло.
1: Завершение отмены передачи произошло.
Бит 7 (TC): Завершение передачи.
0: Передача не завершена.
1: Передача завершена.
Бит 6 (HPM): Получено сообщение с высоким приоритетом.
0: Сообщение с высоким приоритетом не получено.
1: Сообщение с высоким приоритетом получено.
Бит 5 (RF1L): Потеря сообщения в FIFO 1 приема.
0: Сообщение в FIFO 1 приема не потеряно.
1: Сообщение в FIFO 1 приема потеряно.
Бит 4 (RF1F): FIFO 1 приема полон.
0: FIFO 1 приема не полон.
1: FIFO 1 приема полон.
Бит 3 (RF1N): Новое сообщение в FIFO 1 приема.
0: Новое сообщение не записано в FIFO 1 приема.
1: Новое сообщение записано в FIFO 1 приема.
Бит 2 (RF0L): Потеря сообщения в FIFO 0 приема.
0: Сообщение в FIFO 0 приема не потеряно.
1: Сообщение в FIFO 0 приема потеряно.
Бит 1 (RF0F): FIFO 0 приема полон.
0: FIFO 0 приема не полон.
1: FIFO 0 приема полон.
Бит 0 (RF0N): Новое сообщение в FIFO 0 приема.
0: Новое сообщение не записано в FIFO 0 приема.
1: Новое сообщение записано в FIFO 0 приема.
*/
#endif /* INC_CANFD_STM32G4_H_ */
