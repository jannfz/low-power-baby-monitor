/**
 ********************************************************************************
 * @file    ${file_name}
 * @author  ${user}
 * @date    ${date}
 * @brief   
 ********************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SSD1306_H
#define __SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include "error.h"

/* Macros and defines --------------------------------------------------------*/


/* Typedefs ------------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/


/* Global function prototypes ------------------------------------------------*/
Error_t ssd1306_init();
Error_t ssd1306_send_command(uint8_t cmd);
Error_t ssd1306_send_data(uint8_t *data, size_t len);
Error_t ssd1306_clear();
Error_t ssd1306_update();
Error_t ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t on);
Error_t ssd1306_print(uint8_t x, uint8_t y, const char* str);


#ifdef __cplusplus
}
#endif

#endif
