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
#ifndef __DHT22_H
#define __DHT22_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "error.h"

/* Macros and defines --------------------------------------------------------*/

#define DHT22_ERR_OK        0
#define DHT22_ERR_TIMEOUT  -1
#define DHT22_ERR_CHECKSUM -2

/* Typedefs ------------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Global function prototypes ------------------------------------------------*/
int8_t DHT22_Init();
int8_t DHT22_Read(int16_t* temperature, uint16_t* humidity);



#ifdef __cplusplus
}
#endif

#endif
