/**
 ********************************************************************************
 * @file    ${file_name}
 * @author  ${user}
 * @date    ${date}
 * @brief   
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

/* Typedefs ------------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Global function prototypes ------------------------------------------------*/
Error_t DHT22_Init();
Error_t DHT22_Read(float* temperature, float* humidity);

#ifdef __cplusplus
}
#endif

#endif
