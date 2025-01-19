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
	    tseg2 = tq_per_bit - tseg1 - 1;

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
	    FDCAN_NBTP |= (tseg1 << FDCAN_NBTP_NTSEG1_Pos) & FDCAN_NBTP_NTSEG1_Msk;
	    FDCAN_NBTP |= (tseg2 << FDCAN_NBTP_NTSEG2_Pos) & FDCAN_NBTP_NTSEG2_Msk;
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
