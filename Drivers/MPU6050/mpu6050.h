/**
 ********************************************************************************
 * @file    ${file_name}
 * @author  ${user}
 * @date    ${date}
 * @brief   
 *
 * Copyright (c) 2025 Jann Feilberg Zachariasen
 * 
 * This software is released under the MIT License.
 * See LICENSE file in the project root for full license information.
 ********************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "error.h"

/* Macros and defines --------------------------------------------------------*/


/* Typedefs ------------------------------------------------------------------*/

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} Mpu6050_Sample_t;

typedef struct {
    float ax, ay, az; // in m/s²
    float gx, gy, gz; // in rad/s
} Mpu6050_Data_t;

typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERROR = 1,
    MPU6050_I2C_ERROR = 2,
    MPU6050_INVALID_PARAM = 3,
    MPU6050_INIT = 4,
} Mpu6050_Status_t;

/* Exported variables --------------------------------------------------------*/

/* Global function prototypes ------------------------------------------------*/
Mpu6050_Status_t mpu6050_init();
Mpu6050_Status_t mpu6050_readAccelGyro(Mpu6050_Sample_t *data);
Mpu6050_Status_t mpu6050_readAverage(Mpu6050_Sample_t *avg_data, uint16_t samples);
void mpu6050_convert(const Mpu6050_Sample_t *sample, Mpu6050_Data_t *data);


#ifdef __cplusplus
}
#endif

#endif
