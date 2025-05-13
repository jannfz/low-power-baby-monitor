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
#include "DHT22.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"

extern void delay_us(uint32_t us);

/* Macros and defines --------------------------------------------------------*/



#define DHT22_Pin               GPIO_PIN_4
#define DHT22_GPIO_Port         GPIOA

/* Private typedef -----------------------------------------------------------*/

/* Static variables ----------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Static function prototypes-------------------------------------------------*/
static uint8_t ReadBit();
static uint8_t ReadByte();
static void SetPinOutput();
static void SetPinInput();

/* Global functions ----------------------------------------------------------*/


int8_t DHT22_Init() {
    uint8_t bytes[5] = {0};

    SetPinOutput();
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    delay_us(10000);

    // Start signal
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    delay_us(30); // DHT pulls low in 20–40 µs
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);
    SetPinInput();

    // Wait for response
    if (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET) return DHT22_ERR_TIMEOUT;
    while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_RESET);
    while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET);

    // Read 5 bytes
    for (int i = 0; i < 5; i++) {
        bytes[i] = ReadByte();
    }

    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    SetPinOutput();


    // Checksum
    if (((bytes[0] + bytes[1] + bytes[2] + bytes[3]) & 0xFF) != bytes[4]) return DHT22_ERR_CHECKSUM;


    return DHT22_ERR_OK;
}

int8_t DHT22_Read(int16_t* temperature, uint16_t* humidity) {

    uint8_t bytes[5] = {0};

    // Start signal
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    delay_us(30); // DHT pulls low in 20–40 µs
    SetPinInput();

    // Wait for response
    if (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET) return DHT22_ERR_TIMEOUT;
    while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_RESET);
    while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET);

    // Read 5 bytes
    for (int i = 0; i < 5; i++) {
        bytes[i] = ReadByte();
    }

    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    SetPinOutput();


    // Checksum
    if (((bytes[0] + bytes[1] + bytes[2] + bytes[3]) & 0xFF) != bytes[4]) return DHT22_ERR_CHECKSUM;


    // Convert data
    *humidity = (bytes[0] << 8 | bytes[1]);
    int16_t temp = (bytes[2] << 8 | bytes[3]);
    if (temp & 0x8000) temp = -((temp & 0x7FFF));
    *temperature = temp;

    return DHT22_ERR_OK;
}

/* Static functions ----------------------------------------------------------*/

static uint8_t ReadBit() {
    while(HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_RESET);  // Wait for HIGH
    delay_us(35);
    return (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET) ? 1 : 0;
}

static uint8_t ReadByte() {
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        result <<= 1;
        result |= ReadBit();
        while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET);  // Wait for LOW
    }
    return result;
}

static void SetPinOutput() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);
}

static void SetPinInput() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);
}
