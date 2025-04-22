
/**
 ********************************************************************************
 * @file    ${file_name}
 * @author  ${user}
 * @date    ${date}
 * @brief   
 ********************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ERROR_H
#define __ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Macros and defines --------------------------------------------------------*/

typedef enum {
    ERR_OK = 0,
    ERR_TIMEOUT = 1,
    ERR_COMM_FAIL = 2,
    ERR_INVALID_PARAM = 3,
    ERR_UNKNOWN = 4,
    ERR_CHECKSUM = 5,
    ERR_INIT = 6,
} Error_t;

/* Typedefs ------------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Global function prototypes ------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
