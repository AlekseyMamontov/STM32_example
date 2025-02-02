/*
 * IIM_42652.h
 *
 *  Created on: Feb 1, 2025
 *      Author: oleksii
 */

#ifndef INC_IIM_42652_H_
#define INC_IIM_42652_H_
#define "Ixm42xxxDefs.h"



#define IIM_ADR 0x68 << 1

/*Registers addresses*/

/*User bank 0*/
#define DEVICE_CONFIG 0x11
#define DRIVE_CONFIG 0x13
#define INT_CONFIG 0x14
#define FIFO_CONFIG 0x16
#define TEMP_DATA1_UI 0x1D
#define TEMP_DATA0_UI 0x1E
#define ACCEL_DATA_X1_UI 0x1F
#define ACCEL_DATA_X0_UI 0x20
#define ACCEL_DATA_Y1_UI 0x21
#define ACCEL_DATA_Y0_UI 0x22
#define ACCEL_DATA_Z1_UI 0x23
#define ACCEL_DATA_Z0_UI 0x24
#define GYRO_DATA_X1_UI 0x25
#define GYRO_DATA_X0_UI 0x26
#define GYRO_DATA_Y1_UI 0x27
#define GYRO_DATA_Y0_UI 0x28
#define GYRO_DATA_Z1_UI 0x29
#define GYRO_DATA_Z0_UI 0x2A
#define TMST_FSYNCH 0x2B
#define TMST_FSYNCL 0x2C
#define INT_STATUS 0x2D
#define FIFO_COUNTH 0x2E
#define FIFO_COUNTL 0x2F
#define FIFO_DATA 0x30
#define APEX_DATA0 0x31
#define APEX_DATA1 0x32
#define APEX_DATA2 0x33
#define APEX_DATA3 0x34
#define APEX_DATA4 0x35
#define APEX_DATA5 0x36
#define INT_STATUS2 0x37
#define INT_STATUS3 0x38
#define SIGNAL_PATH_RESET 0x4B
#define INTF_CONFIG0 0x4C
#define INTF_CONFIG1 0x4D
#define PWR_MGMT0 0x4E
#define GYRO_CONFIG0 0x4F
#define ACCEL_CONFIG0 0x50
#define GYRO_CONFIG1 0x51
#define GYRO_ACCEL_CONFIG0 0x52
#define ACCEL_CONFIG1 0x53
#define TMST_CONFIG 0x54
#define APEX_CONFIG0 0x55
#define SMD_CONFIG 0x56
#define FIFO_CONFIG1 0x5F
#define FIFO_CONFIG2 0x60
#define FIFO_CONFIG3 0x61
#define FSYNC_CONFIG 0x62
#define INT_CONFIG0 0x63
#define INT_CONFIG1 0x64
#define INT_SOURCE0 0x65
#define INT_SOURCE1 0x66
#define INT_SOURCE3 0x68
#define INT_SOURCE4 0x69
#define FIFO_LOST_PKT0 0x6C
#define FIFO_LOST_PKT1 0x6D
#define SELF_TEST_CONFIG 0x70
#define WHO_AM_I 0x75
#define BANK_SEL 0x76
#define CHIP_ID 0x6F

/*User bank 1*/
#define SENSOR_CONFIG0 0x03
#define GYRO_CONFIG_STATIC2 0x0B
#define GYRO_CONFIG_STATIC3 0x0C
#define GYRO_CONFIG_STATIC4 0x0D
#define GYRO_CONFIG_STATIC5 0x0E
#define GYRO_CONFIG_STATIC6 0x0F
#define GYRO_CONFIG_STATIC7 0x10
#define GYRO_CONFIG_STATIC8 0x11
#define GYRO_CONFIG_STATIC9 0x12
#define GYRO_CONFIG_STATIC10 0x13
#define XG_ST_DATA 0x5F
#define YG_ST_DATA 0x60
#define ZG_ST_DATA 0x61
#define TMSTVAL0 0x62
#define TMSTVAL1 0x63
#define TMSTVAL2 0x64
#define INTF_CONFIG4 0x7A
#define INTF_CONFIG5 0x7B
#define INTF_CONFIG6 0x7C

/*User bank 2*/
#define ACCEL_CONFIG_STATIC2 0x03
#define ACCEL_CONFIG_STATIC3 0x04
#define ACCEL_CONFIG_STATIC4 0x05
#define XA_ST_DATA 0x3B
#define YA_ST_DATA 0x3C
#define ZA_ST_DATA 0x3D

/*User bank 3*/
#define PU_PD_CONFIG1 0x06
#define PU_PD_CONFIG2 0x0E

/*User bank 4*/
#define FDR_CONFIG                          0x09
#define APEX_CONFIG1                        0x40
#define APEX_CONFIG2                        0x41
#define APEX_CONFIG3                        0x42
#define APEX_CONFIG4                        0x43
#define APEX_CONFIG5                        0x44
#define APEX_CONFIG6                        0x45
#define APEX_CONFIG7                        0x46
#define APEX_CONFIG8                        0x47
#define APEX_CONFIG9                        0x48
#define APEX_CONFIG10                       0x49
#define ACCEL_WOM_X_THR                     0x4A
#define ACCEL_WOM_Y_THR                     0x4B
#define ACCEL_WOM_Z_THR                     0x4C
#define INT_SOURCE6                         0x4D
#define INT_SOURCE7                         0x4E
#define INT_SOURCE8                         0x4F
#define INT_SOURCE9                         0x50
#define INT_SOURCE10                        0x51
#define OFFSET_USER0                        0x77
#define OFFSET_USER1                        0x78
#define OFFSET_USER2                        0x79
#define OFFSET_USER3                        0x7A
#define OFFSET_USER4                        0x7B
#define OFFSET_USER5                        0x7C
#define OFFSET_USER6                        0x7D
#define OFFSET_USER7                        0x7E
#define OFFSET_USER8                        0x7F

/**
 * @brief Command macros
 *
 */

#define SET_GYRO_FS_SEL_2000_dps                0x00
#define SET_GYRO_FS_SEL_1000_dps                0x01
#define SET_GYRO_FS_SEL_500_dps                 0x02
#define SET_GYRO_FS_SEL_250_dps                 0x03
#define SET_GYRO_FS_SEL_125_dps                 0x04
#define SET_GYRO_FS_SEL_62_5_dps                0x05
#define SET_GYRO_FS_SEL_31_25_dps               0x06
#define SET_GYRO_FS_SEL_16_625_dps              0x07

#define SET_GYRO_ODR_32kHz                      0x01
#define SET_GYRO_ODR_16kHz                      0x02
#define SET_GYRO_ODR_8kHz                       0x03
#define SET_GYRO_ODR_4kHz                       0x04
#define SET_GYRO_ODR_2kHz                       0x05
#define SET_GYRO_ODR_1kHz                       0x06
#define SET_GYRO_ODR_200Hz                      0x07
#define SET_GYRO_ODR_100Hz                      0x08
#define SET_GYRO_ODR_50Hz                       0x09
#define SET_GYRO_ODR_25Hz                       0x0A
#define SET_GYRO_ODR_12_5Hz                     0x0B

#define SET_ACCEL_FS_SEL_16g                    0x00
#define SET_ACCEL_FS_SEL_8g                     0x01
#define SET_ACCEL_FS_SEL_4g                     0x02
#define SET_ACCEL_FS_SEL_2g                     0x03

#define SET_ACCEL_ODR_32kHz                     0x01
#define SET_ACCEL_ODR_16kHz                     0x02
#define SET_ACCEL_ODR_8kHz                      0x03
#define SET_ACCEL_ODR_4kHz                      0x04
#define SET_ACCEL_ODR_2kHz                      0x05
#define SET_ACCEL_ODR_1kHz                      0x06
#define SET_ACCEL_ODR_200Hz                     0x07
#define SET_ACCEL_ODR_100Hz                     0x08
#define SET_ACCEL_ODR_50Hz                      0x09
#define SET_ACCEL_ODR_25Hz                      0x0A
#define SET_ACCEL_ODR_12_5Hz                    0x0B


#endif /* INC_IIM_42652_H_ */
