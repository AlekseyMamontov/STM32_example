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
#include <math.h>

#define FILTER_ORDER 5

// Структура для зберігання даних сенсорів
typedef struct {
    int16_t accel[3];   // Дані акселерометра (x, y, z) в 16-бітному форматі
    int16_t gyro[3];    // Дані гироскопа (x, y, z) в 16-бітному форматі
    int16_t mag[3];     // Дані магнітометра (x, y, z) в 16-бітному форматі
} SensorData;

// Структура для зберігання кватерніона
typedef struct {
    float q[4];         // Кватерніон (w, x, y, z)
} Quaternion;

// Глобальні змінні для фільтрації
float coeff[FILTER_ORDER] = {0.1, 0.15, 0.4, 0.15, 0.1}; // Коефіцієнти фільтра Меджвика
float accel_filtered[3] = {0.0, 0.0, 0.0};
float gyro_filtered[3] = {0.0, 0.0, 0.0};
float mag_filtered[3] = {0.0, 0.0, 0.0};

// Функція для застосування фільтра Меджвика до 16-бітних даних
void medwick_filter(int16_t *input, float *output, int length) {
    for (int i = 0; i < length; i++) {
        output[i] = 0;
        for (int j = 0; j < FILTER_ORDER; j++) {
            if (i - j >= 0) {
                output[i] += coeff[j] * input[i - j];
            }
        }
    }
}

// Функція для оновлення кватерніона
void update_quaternion(Quaternion *q, float gx, float gy, float gz, float dt) {
    float q1 = q->q[0];
    float q2 = q->q[1];
    float q3 = q->q[2];
    float q4 = q->q[3];

    // Перетворення кутових швидкостей в радіани
    gx *= (M_PI / 180.0);
    gy *= (M_PI / 180.0);
    gz *= (M_PI / 180.0);

    // Оновлення кватерніона
    q->q[0] += 0.5f * (-q2 * gx - q3 * gy - q4 * gz) * dt;
    q->q[1] += 0.5f * (q1 * gx + q3 * gz - q4 * gy) * dt;
    q->q[2] += 0.5f * (q1 * gy - q2 * gz + q4 * gx) * dt;
    q->q[3] += 0.5f * (q1 * gz + q2 * gy - q3 * gx) * dt;

    // Нормалізація кватерніона
    float norm = sqrt(q->q[0] * q->q[0] + q->q[1] * q->q[1] + q->q[2] * q->q[2] + q->q[3] * q->q[3]);
    for (int i = 0; i < 4; i++) {
        q->q[i] /= norm;
    }
}

// Імітація отримання даних від сенсорів
void get_sensor_data(SensorData *data) {
    // Тут додайте код для отримання даних від акселерометра, гироскопа, магнітометра та барометра
}

// Головна функція
int main(void) {
    // Ініціалізація периферії STM32 та сенсорів
    HAL_Init();
    // Ініціалізація сенсорів тут...

    SensorData sensor_data;
    Quaternion q = {{1.0, 0.0, 0.0, 0.0}}; // Початковий кватерніон

    while (1) {
        // Отримання даних від сенсорів
        get_sensor_data(&sensor_data);

        // Застосування фільтра Меджвика до акселерометра
        medwick_filter(sensor_data.accel, accel_filtered, 3);

        // Застосування фільтра Меджвика до гироскопа
        medwick_filter(sensor_data.gyro, gyro_filtered, 3);

        // Застосування фільтра Меджвика до магнітометра
        medwick_filter(sensor_data.mag, mag_filtered, 3);

        // Оновлення кватерніона на основі відфільтрованих даних гироскопа
        update_quaternion(&q, gyro_filtered[0], gyro_filtered[1], gyro_filtered[2], 0.01); // dt = 0.01 сек

        // Обробка відфільтрованих даних та кватерніона
        // Тут додайте код для подальшої роботи з відфільтрованими даними та кватерніоном
    }
}
#endif /* INC_QWARTERION_H_ */
