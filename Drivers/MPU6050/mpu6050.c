/*
 *******************************************************************************
 * @file    ${file_name}
 * @author  ${user}
 * @date    ${date}
 * @brief   
 *
 * Copyright (c) 2025 Jann Feilberg Zachariasen
 * 
 * This software is released under the MIT License.
 * See LICENSE file in the project root for full license information.
 *******************************************************************************
 */

/* Private includes ----------------------------------------------------------*/
#include "mpu6050.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

extern I2C_HandleTypeDef hi2c1;

/* Macros and defines --------------------------------------------------------*/

#define MPU6050_ADDR            (0x68 << 1)  // AD0 = GND
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_WHO_AM_I     0x75


/* Private typedef -----------------------------------------------------------*/

/* Static variables ----------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Static function prototypes-------------------------------------------------*/

/* Static functions ----------------------------------------------------------*/
static int16_t MPU6050_Read16(uint8_t *data) {
    return (int16_t)((data[0] << 8) | data[1]);
}

/* Global functions ----------------------------------------------------------*/

Mpu6050_Status_t mpu6050_init() {

    uint8_t check, data;

    // Check WHO_AM_I
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_WHO_AM_I, 1, &check, 1, 100) != HAL_OK) {
        return MPU6050_ERROR;
    }
    if (check != 0x68) {
        return MPU6050_ERROR;
    }

    // Wake up device (clear sleep bit)
    data = 0x00;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, 100) != HAL_OK) {
        return MPU6050_ERROR;
    }

    return MPU6050_OK;
}

Mpu6050_Status_t mpu6050_readAccelGyro(Mpu6050_Sample_t *data) {
    uint8_t buf[14];

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, buf, 14, 100) != HAL_OK)
        return MPU6050_ERROR;

    data->ax = MPU6050_Read16(&buf[0]);
    data->ay = MPU6050_Read16(&buf[2]);
    data->az = MPU6050_Read16(&buf[4]);
    data->gx = MPU6050_Read16(&buf[8]);
    data->gy = MPU6050_Read16(&buf[10]);
    data->gz = MPU6050_Read16(&buf[12]);

    return MPU6050_OK;
}

Mpu6050_Status_t mpu6050_readAverage(Mpu6050_Sample_t *avg_data, uint16_t samples) {
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    Mpu6050_Sample_t temp;

    for (uint16_t i = 0; i < samples; i++) {
        if (mpu6050_readAccelGyro(&temp) == MPU6050_OK) {
            sum_ax += temp.ax;
            sum_ay += temp.ay;
            sum_az += temp.az;
            sum_gx += temp.gx;
            sum_gy += temp.gy;
            sum_gz += temp.gz;
        }
        HAL_Delay(1); // Small delay between samples (optional)
    }

    avg_data->ax = sum_ax / samples;
    avg_data->ay = sum_ay / samples;
    avg_data->az = sum_az / samples;
    avg_data->gx = sum_gx / samples;
    avg_data->gy = sum_gy / samples;
    avg_data->gz = sum_gz / samples;

    return MPU6050_OK;
}


void mpu6050_convert(const Mpu6050_Sample_t *sample, Mpu6050_Data_t *data) {
    data->ax = (sample->ax / 16384.0f) * 9.81f;
    data->ay = (sample->ay / 16384.0f) * 9.81f;
    data->az = (sample->az / 16384.0f) * 9.81f;

    float deg2rad = 3.14159265f / 180.0f;
    data->gx = (sample->gx / 131.0f) * deg2rad;
    data->gy = (sample->gy / 131.0f) * deg2rad;
    data->gz = (sample->gz / 131.0f) * deg2rad;
}

