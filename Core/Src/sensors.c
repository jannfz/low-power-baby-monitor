/*
 *******************************************************************************
 * @file    ${file_name}
 * @author  ${user}
 * @date    ${date}
 * @brief
 *******************************************************************************
 *
 * Copyright (c) 2025 Jann Feilberg Zachariasen
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for full license information.
 */

/* Private includes ----------------------------------------------------------*/
#include "sensors.h"
#include "DHT22.h"

#include <stdio.h>

/* Macros and defines --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Static variables ----------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

EnvData_t environmental_data = {0, 0};

/* Static function prototypes-------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/


Error_t update_environmental_data()
{
    Temperature_t temp;
    Humidity_t hum;

    if (DHT22_Read(&temp, &hum) != DHT22_ERR_OK) {
        return ERR_UNKNOWN;
    }

    printf("Temperature: %5d    Humidity: %5d \n", temp, hum);

    environmental_data.temperature = temp * 10;
    environmental_data.humidity = hum * 10;

    return ERR_OK;
}

/* Static functions ----------------------------------------------------------*/
