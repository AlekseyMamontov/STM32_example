#ifndef INC_QWARTERION_H_
#define INC_QWARTERION_H_

#include "math.h"
#include "Fusion.h"

#define SAMPLE_RATE 1000.0f
#define DELTA_TIME (1.0f / SAMPLE_RATE)
#define ACCEL_SCALE 2048.0f  // ±16g
#define GYRO_SCALE 16.384f   // ±2000 dps
#define MAG_SCALE 0.15f      // Пример (настройте под ваш магнитометр)

typedef struct {
    int16_t acc[3];
    int16_t gyro[3];
    int16_t mag[3];
} ImuData;


FusionAhrs ahrs;
FusionQuaternion quaternion;
void update_quaternion(ImuData* raw_data) {

    // Инициализация FusionVector
    FusionVector gyroscope = {{raw_data->gyro[0],raw_data->gyro[1],raw_data->gyro[2]}};
    FusionVector accelerometer = {{raw_data->acc[0],raw_data->acc[1],raw_data->acc[2]}};

    // Обновление с гироскопом и акселерометром
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, DELTA_TIME);

    // Обновление магнитометра с указанием системы координат
    FusionVector magnetometer = {{raw_data->mag[0],raw_data->mag[1],raw_data->mag[2]}};
    float heading = FusionCompassCalculateHeading(FusionConventionNwu, accelerometer, magnetometer);
    FusionAhrsSetHeading(&ahrs, heading);

    quaternion = FusionAhrsGetQuaternion(&ahrs);
}

void xmain(void) {
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSetSettings(&ahrs, &(FusionAhrsSettings){
        .gain = 0.5f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f
        // Убрано rejectionTimeout, так как его нет в вашей версии
    });



}


#endif // INC_QWARTERION_H_
