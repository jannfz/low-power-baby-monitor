/**
  ******************************************************************************
  * @file    sensors.h
  * @author  Jann Feilberg Zachariasen
  * @brief   Header file for app_bluenrg_ms.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSORS_H
#define SENSORS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "error.h"

typedef int16_t Temperature_t;
typedef uint16_t Humidity_t;

typedef struct {
    Temperature_t temperature;
    Humidity_t humidity;
} EnvData_t;

/* Exported Variables --------------------------------------------------------*/

extern EnvData_t environmental_data;

/* Exported Functions --------------------------------------------------------*/

Error_t update_environmental_data();

#ifdef __cplusplus
}
#endif
#endif /* SENSORS_H */
