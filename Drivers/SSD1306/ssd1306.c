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
#include "ssd1306.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "font5x7.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

/* Macros and defines --------------------------------------------------------*/
#define SSD1306_I2C_ADDR    0x78

#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64

#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8)
#define SSD1306_MUX_RATIO   SSD1306_HEIGHT - 1


/* Fundamental Commands */
#define SSD1306_SET_CONTRAST                0x81  // Set contrast level (0x00–0xFF). Higher = brighter. Default = 0x7F.
#define SSD1306_DISPLAY_RESUME              0xA4  // Resume display from RAM content (normal display mode)
#define SSD1306_DISPLAY_ALL_ON              0xA5  // Force all pixels ON (ignores RAM content)
#define SSD1306_DISPLAY_NORMAL              0xA6  // Set display to non-inverted mode (RESET)
#define SSD1306_DISPLAY_INVERSE             0xA7  // Set display to inverted mode
#define SSD1306_DISPLAY_OFF                 0xAE  // Turn display OFF (enter sleep mode)
#define SSD1306_DISPLAY_ON                  0xAF  // Turn display ON

/* Scrolling Commands */
#define SSD1306_SCROLL_RIGHT                0x26  // Continuous horizontal scroll to the right (7-byte command)
#define SSD1306_SCROLL_LEFT                 0x27  // Continuous horizontal scroll to the left (7-byte command)
#define SSD1306_SCROLL_VERTICAL_RIGHT       0x29  // Vertical and right horizontal scroll (6-byte command)
#define SSD1306_SCROLL_VERTICAL_LEFT        0x2A  // Vertical and left horizontal scroll (6-byte command)
#define SSD1306_DEACTIVATE_SCROLL           0x2E  // Stop scrolling
#define SSD1306_ACTIVATE_SCROLL             0x2F  // Start scrolling
#define SSD1306_SET_VERTICAL_SCROLL_AREA    0xA3  // Set vertical scroll area

/* Addressing Commands */
#define SSD1306_SET_LOW_COLUMN              0x00  // Set lower nibble of column start address (Page mode)
#define SSD1306_SET_HIGH_COLUMN             0x10  // Set upper nibble of column start address (Page mode)
#define SSD1306_SET_MEMORY_ADDR_MODE        0x20  // Set memory addressing mode: 0x00=Horizontal, 0x01=Vertical, 0x02=Page
#define SSD1306_HORIZONTAL_MODE             0x00  // Horizontal addressing mode
#define SSD1306_VERTICAL_MODE               0x01  // Vertical addressing mode
#define SSD1306_PAGE_MODE                   0x02  // Page addressing mode (RESET)
#define SSD1306_SET_COLUMN_ADDR             0x21  // Set column address range (2-byte: start, end)
#define SSD1306_SET_PAGE_ADDR               0x22  // Set page address range (2-byte: start, end)
#define SSD1306_SET_PAGE_START_ADDR         0xB0  // Set page start address (Page mode only)

/* Hardware Configuration Commands */
#define SSD1306_SET_START_LINE              0x40  // Set display RAM start line (0–63). Default = 0x40.
#define SSD1306_SEG_REMAP_0                 0xA0  // Column address 0 maps to SEG0 (normal mapping)
#define SSD1306_SEG_REMAP_1                 0xA1  // Column address 127 maps to SEG0 (remapped)
#define SSD1306_SET_MUX_RATIO               0xA8  // Set MUX ratio (height - 1). e.g., 0x3F for 64px, 0x1F for 32px
#define SSD1306_COM_SCAN_DIR_INC            0xC0  // Scan COM outputs from COM0 to COM[N-1] (normal mode)
#define SSD1306_COM_SCAN_DIR_DEC            0xC8  // Scan COM outputs from COM[N-1] to COM0 (remapped)
#define SSD1306_SET_DISPLAY_OFFSET          0xD3  // Set vertical display offset (0–63)
#define SSD1306_SET_COM_PINS                0xDA  // Set COM pins hardware config. Bit 5: L/R remap, Bit 4: Alt/Seq

/* Timing & Driving Scheme Commands */
#define SSD1306_SET_OSC                     0xD5  // Set clock divide ratio and oscillator frequency
#define SSD1306_SET_PRECHARGE               0xD9  // Set pre-charge period (phase 1 & 2)
#define SSD1306_SET_VCOM_DETECT             0xDB  // Set VCOMH deselect level: 0.65xVcc, 0.77xVcc (RESET), or 0.83xVcc
#define SSD1306_NOP                         0xE3  // No operation

/* Charge Pump Commands */
#define SSD1306_CHARGE_PUMP                 0x8D  // Enable/disable internal charge pump
#define SSD1306_CHARGE_PUMP_OFF             0x10  // Disable charge pump (A[2]=0)
#define SSD1306_CHARGE_PUMP_ON              0x14  // Enable charge pump (A[2]=1)


/* Private typedef -----------------------------------------------------------*/

/* Static variables ----------------------------------------------------------*/

static uint8_t framebuffer[SSD1306_BUFFER_SIZE];

static const uint8_t SSD1306_INIT_CMDS[] = {
    SSD1306_DISPLAY_OFF,
    SSD1306_SET_MUX_RATIO,              SSD1306_MUX_RATIO,
    SSD1306_SET_DISPLAY_OFFSET,         0x00,
    SSD1306_SET_START_LINE | 0x00,
    SSD1306_SEG_REMAP_1,
    SSD1306_COM_SCAN_DIR_DEC,
    SSD1306_SET_COM_PINS,               0x12,
    SSD1306_SET_CONTRAST,               0x8F,
    SSD1306_DISPLAY_RESUME,
    SSD1306_SET_OSC,                    0x80,
    SSD1306_CHARGE_PUMP,                SSD1306_CHARGE_PUMP_ON,
    SSD1306_DISPLAY_ON,
};


/* Global variables ----------------------------------------------------------*/

/* Static function prototypes-------------------------------------------------*/


/* Global functions ----------------------------------------------------------*/


Error_t ssd1306_init() {

    HAL_Delay(100);

    for (size_t i = 0; i < sizeof(SSD1306_INIT_CMDS); i++) {
        Error_t status = ssd1306_send_command(SSD1306_INIT_CMDS[i]);
        if (status != ERR_OK) {
            return status;
        }
    }

    return ERR_OK;
}

Error_t ssd1306_send_command(uint8_t cmd) {

    uint8_t buf[2] = {0x80, cmd};

    if (HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, buf, 2, 100) != HAL_OK) {
        return ERR_COMM_FAIL;
    }

    return ERR_OK;
}

// SSD1306_Status_t ssd1306_send_data(uint8_t *data, size_t len) {
//     printf("ssd1306_send_data\n");
//     uint8_t *buf = malloc(len + 1);
//     buf[0] = 0x40; // Control byte 0x40 = data
//     memcpy(&buf[1], data, len);
//     HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, buf, len + 1, HAL_MAX_DELAY);
//     free(buf);
// }

// void ssd1306_draw_rectancle(uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
// 
//     uint8_t page = y / 8;
//     uint16_t index = x + page * SSD1306_WIDTH;
//     uint8_t bit = y % 8;
//     
//     for (uint8_t i = 0; i < width; i ++) {
// 
// 
// 
//     }
// 
// }

Error_t ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t on) {

    if (x > SSD1306_WIDTH || y > SSD1306_HEIGHT) {
        return ERR_INVALID_PARAM;
    }

    uint8_t page = y / 8;
    uint16_t index = x + page * SSD1306_WIDTH;
    uint8_t bit = y % 8;

    if (on) {
        framebuffer[index] |= (1 << bit);
    } else {
        framebuffer[index] &= ~(1 << bit);
    }

    return ERR_OK;
}

Error_t ssd1306_draw_char(uint8_t x, uint8_t y, char c) {

    if (c < 0x20 || c > 0x7F) {
        return ERR_INVALID_PARAM;  // unsupported char
    }

    if (x > SSD1306_WIDTH - 6 || y > SSD1306_HEIGHT - 8) {
        return ERR_INVALID_PARAM;      // out of bounds
    }

    uint8_t page = y / 8;
    uint16_t pos = x + page * SSD1306_WIDTH;

    for (uint8_t i = 0; i < 5; i++) {
        framebuffer[pos + i] = Font5x7[c - 0x20][i];
    }

    framebuffer[pos + 5] = 0x00; // 1-pixel spacing

    return ERR_OK;
}


Error_t ssd1306_print(uint8_t x, uint8_t y, const char* str) {

    if (str == NULL) {
        return ERR_INVALID_PARAM;
    }

    while (*str && x < SSD1306_WIDTH - 6) {
        Error_t status = ssd1306_draw_char(x, y, *str++);
        if (status != ERR_OK) {
            return status;
        }
        x += 6;  // 5 pixels + 1 spacing
    }

    return ERR_OK;
}


Error_t ssd1306_clear() {

    memset(framebuffer, 0x00, SSD1306_BUFFER_SIZE);

    return ssd1306_update();  // Push the cleared buffer to OLED
}

Error_t ssd1306_update(void) {

    for (uint8_t page = 0; page < 8; page++) {
        // Set page address
        if (ssd1306_send_command(SSD1306_SET_PAGE_ADDR) != ERR_OK) return ERR_COMM_FAIL;
        if (ssd1306_send_command(page) != ERR_OK) return ERR_COMM_FAIL;
        if (ssd1306_send_command(7) != ERR_OK) return ERR_COMM_FAIL;

        // Set column address
        if (ssd1306_send_command(SSD1306_SET_COLUMN_ADDR) != ERR_OK) return ERR_COMM_FAIL;
        if (ssd1306_send_command(0) != ERR_OK) return ERR_COMM_FAIL;
        if (ssd1306_send_command(SSD1306_WIDTH - 1) != ERR_OK) return ERR_COMM_FAIL;

        // Write data
        uint8_t control = 0x40; // Co = 0, D/C# = 1 (data)
        HAL_StatusTypeDef result = HAL_I2C_Mem_Write(
            &hi2c1,
            SSD1306_I2C_ADDR,
            control,
            I2C_MEMADD_SIZE_8BIT,
            &framebuffer[page * SSD1306_WIDTH],
            SSD1306_WIDTH,
            100
        );

        if (result != HAL_OK) {
            return ERR_COMM_FAIL;
        }
    }

    return ERR_OK;
}


/* Static functions ----------------------------------------------------------*/


/*
 * SSD1306 has internal command registers that are used to configure the
 * operations of the driver IC. After reset, the registers should be set with
 * appropriate values in order to function well. The registers can be accessed
 * by MPU interface in either 6801, 8080, SPI type with D/C# pin pull low or
 * using I2C interface. Below is an example of initialization flow of SSD1307.
 * The values of registers depend on different condition and application.
 *
 * Set MUX Ratio                        A8h,    3Fh
 * Set Display Offset                   D3h,    00h
 * Set Display Start Line               40h
 * Set Segment re-map                   A0h/A1h
 * Set COM Output Scan Direction        C0h/C8h
 * Set COM Pins hardware configuration  DAh,    02
 * Set Contrast Control                 81h,    7Fh
 * Disable Entire Display On            A4h
 * Set Normal Display                   A6h
 * Set Osc Frequency                    D5h,    80h
 * Enable charge pump regulator         8Dh,    14h
 * Display On                           AFh
*/
