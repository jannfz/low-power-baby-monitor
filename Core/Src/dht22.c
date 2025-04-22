/*
 *******************************************************************************
 * @file    ${file_name}
 * @author  ${user}
 * @date    ${date}
 * @brief   
 *******************************************************************************
 */

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include "dht22.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"

extern void delay_us(uint32_t us);

/* Macros and defines --------------------------------------------------------*/

#define DHT22_TIMEOUT   100

#define DHT22_Pin       GPIO_PIN_4
#define DHT22_GPIO_Port GPIOA

/* Private typedef -----------------------------------------------------------*/

/* Static variables ----------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Static function prototypes-------------------------------------------------*/

/* Static functions ----------------------------------------------------------*/


static uint8_t DHT22_ReadBit() {
    while(HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_RESET);  // Wait for HIGH
    delay_us(35);
    return (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET) ? 1 : 0;
}

static uint8_t DHT22_ReadByte() {
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        result <<= 1;
        result |= DHT22_ReadBit(DHT22_GPIO_Port, DHT22_Pin);
        while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET);  // Wait for LOW
    }
    return result;
}

static void DHT22_SetPinOutput() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);
}

static void DHT22_SetPinInput() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);
}

/* Global functions ----------------------------------------------------------*/


Error_t DHT22_Init() {
    uint8_t bits[5] = {0};

    DHT22_SetPinOutput();
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    delay_us(10000);

    // Start signal
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    delay_us(30); // DHT pulls low in 20–40 µs
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);
    DHT22_SetPinInput();

    // Wait for response
    if (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET) return ERR_TIMEOUT;
    while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_RESET);
    while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET);

    // Read 5 bytes
    for (int i = 0; i < 5; i++) {
        bits[i] = DHT22_ReadByte();
    }

    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    DHT22_SetPinOutput();


    // Checksum
    if (((bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF) != bits[4]) return ERR_CHECKSUM;


    return ERR_OK;
}

Error_t DHT22_Read(float* temperature, float* humidity) {

    uint8_t bytes[5] = {0};

    // Start signal
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    delay_us(30); // DHT pulls low in 20–40 µs
    DHT22_SetPinInput();

    // Wait for response
    if (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET) return ERR_TIMEOUT;
    while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_RESET);
    while (HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET);

    // Read 5 bytes
    for (int i = 0; i < 5; i++) {
        bytes[i] = DHT22_ReadByte();
    }

    HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
    DHT22_SetPinOutput();

    uint8_t checksum = (bytes[0] + bytes[1] + bytes[2] + bytes[3]);

    // Checksum
    if (((bytes[0] + bytes[1] + bytes[2] + bytes[3]) & 0xFF) != bytes[4]) return ERR_CHECKSUM;

    printf("\r\n%02x %02x %02x %02x %02x %02x\r\n", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], checksum);

    // Convert data
    *humidity = (bytes[0] << 8 | bytes[1]) * 0.1f;
    int16_t temp = (bytes[2] << 8 | bytes[3]);
    if (temp & 0x8000) temp = -((temp & 0x7FFF));
    *temperature = temp * 0.1f;

    return ERR_OK;
}
