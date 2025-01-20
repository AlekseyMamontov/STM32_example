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


#define RAMBaseFDCAN1  0x4000A400
#define RAMBaseFDCAN2  0x4000A800
#define RAMBaseFDCAN3  0x4000AC00
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
        TxBuffer = (uint32_t *)RAMBaseFDCAN1 + (RamBaseTX / 4) + (PutIndex * 18);// Указатель на буфер передачи в Message RAM/
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


}



#endif /* INC_CANFD_STM32G4_H_ */
