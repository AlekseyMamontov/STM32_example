/*
 * Qwarterion.h
 *
 *  Created on: Feb 16, 2025
 *      Author: oleksii
 */

#ifndef INC_QWARTERION_H_
#define INC_QWARTERION_H_

#include <math.h>
#define FILTER_ORDER 5
#include "stm32g4xx.h"  // Подключите заголовочный файл для STM32G4
#include <math.h>

#define FILTER_ORDER 5

// Структура для хранения данных сенсоров
typedef struct {
    float accel[3];   // Данные акселерометра (x, y, z)
    float gyro[3];    // Данные гироскопа (x, y, z)
    float mag[3];     // Данные магнитометра (x, y, z)
    float pressure;    // Давление от барометра
} SensorData;

// Структура для хранения кватерниона
typedef struct {
    float q[4];       // Кватернион (w, x, y, z)
} Quaternion;

// Глобальные переменные для фильтрации
float coeff[FILTER_ORDER] = {0.1, 0.15, 0.4, 0.15, 0.1}; // Коэффициенты фильтра Меджвика
float accel_filtered[3] = {0.0, 0.0, 0.0};
float gyro_filtered[3] = {0.0, 0.0, 0.0};
float mag_filtered[3] = {0.0, 0.0, 0.0};

// Функция для применения фильтра Меджвика
void medwick_filter(float *input, float *output, int length) {
    for (int i = 0; i < length; i++) {
        output[i] = 0;
        for (int j = 0; j < FILTER_ORDER; j++) {
            if (i - j >= 0) {
                output[i] += coeff[j] * input[i - j];
            }
        }
    }
}

// Функция для обновления кватернионов
void update_quaternion(Quaternion *q, float gx, float gy, float gz, float dt) {
    float q1 = q->q[0];
    float q2 = q->q[1];
    float q3 = q->q[2];
    float q4 = q->q[3];

    // Преобразование углов в радианы
    gx *= (M_PI / 180.0);
    gy *= (M_PI / 180.0);
    gz *= (M_PI / 180.0);

    // Обновление кватернионов
    q->q[0] += 0.5f * (-q2 * gx - q3 * gy - q4 * gz) * dt;
    q->q[1] += 0.5f * (q1 * gx + q3 * gz - q4 * gy) * dt;
    q->q[2] += 0.5f * (q1 * gy - q2 * gz + q4 * gx) * dt;
    q->q[3] += 0.5f * (q1 * gz + q2 * gy - q3 * gx) * dt;

    // Нормализация кватерниона
    float norm = sqrt(q->q[0] * q->q[0] + q->q[1] * q->q[1] + q->q[2] * q->q[2] + q->q[3] * q->q[3]);
    for (int i = 0; i < 4; i++) {
        q->q[i] /= norm;
    }
}

// Имитация получения данных от сенсоров
void get_sensor_data(SensorData *data) {
    // Здесь добавьте код для получения данных от акселерометра, гироскопа, магнитометра и барометра
}

// Главная функция
int main(void) {
    // Инициализация периферии STM32 и сенсоров
    HAL_Init();
    // Инициализация сенсоров здесь...

    SensorData sensor_data;
    Quaternion q = {{1.0, 0.0, 0.0, 0.0}}; // Начальный кватернион

    while (1) {
        // Получение данных от сенсоров
        get_sensor_data(&sensor_data);

        // Применение фильтра Меджвика к акселерометру
        medwick_filter(sensor_data.accel, accel_filtered, 3);

        // Применение фильтра Меджвика к гироскопу
        medwick_filter(sensor_data.gyro, gyro_filtered, 3);

        // Применение фильтра Меджвика к магнитометру
        medwick_filter(sensor_data.mag, mag_filtered, 3);

        // Обновление кватерниона
        update_quaternion(&q, gyro_filtered[0], gyro_filtered[1], gyro_filtered[2], 0.01); // dt = 0.01 сек

        // Обработка давления от барометра
        float pressure = sensor_data.pressure; // Данные давления
        // Здесь добавьте код для работы с давлением, например, расчет высоты

        // Обработка отфильтрованных данных и кватерниона
        // Здесь добавьте код для дальнейшей работы с отфильтрованными данными и кватернионом
    }
}
#endif /* INC_QWARTERION_H_ */
