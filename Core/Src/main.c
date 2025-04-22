/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "error.h"
#include "i2c.h"
#include "stm32l4xx_hal.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdatomic.h>
#include <stdio.h>
#include "dht22.h"
#include "ssd1306.h"
#include "mpu6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}


void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // Enable tracing
    DWT->CYCCNT = 0;                                  // Reset the counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // Start the counter
}

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < ticks);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  switch (ssd1306_init()) {
      case ERR_OK:
          printf("ERR_OK");
          break;
      case ERR_TIMEOUT:
          printf("ERR_TIMEOUT");
          break;
      case ERR_COMM_FAIL:
          printf("ERR_COMM_FAIL");
          break;
      default:
          printf("default");
          break;
  }
  ssd1306_clear();
  ssd1306_update();
  mpu6050_init();
  switch (DHT22_Init()) {
      case ERR_OK:
          printf("ERR_OK");
          break;
      case ERR_TIMEOUT:
          printf("ERR_TIMEOUT");
          break;
      case ERR_CHECKSUM:
          printf("ERR_CHECKSUM");
          break;
      default:
          printf("default");
          break;
  }

  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    Mpu6050_Sample_t sample;
    Mpu6050_Data_t data;

    float temperature;
    float humidity;
    char buffer[28] = "";

    DHT22_Read(&temperature, &humidity);
    mpu6050_readAverage(&sample, 20);
    mpu6050_convert(&sample, &data);

    printf(" -------------------------\r\n");
    printf("| T:  |  %7.1f | %6.1f |\r\n", temperature, humidity);
    printf(" -------------------------\r\n");
    printf("| ax: |  %7.3f | %6d |\r\n", data.ax, sample.ax);
    printf("| ay: |  %7.3f | %6d |\r\n", data.ay, sample.ay);
    printf("| az: |  %7.3f | %6d |\r\n", data.az, sample.az);
    printf("| gx: |  %7.3f | %6d |\r\n", data.gx, sample.gx);
    printf("| gy: |  %7.3f | %6d |\r\n", data.gy, sample.gy);
    printf("| gz: |  %7.3f | %6d |\r\n", data.gz, sample.gz);

    sprintf(buffer, "T: %4.1f H: %4.1f", temperature, humidity);
    ssd1306_print(0, 0, buffer);

    ssd1306_print(0, 16, "       x     y    z  ");
    sprintf(buffer, "a %6.2f %6.2f %6.2f", data.ax, data.ay, data.az);
    ssd1306_print(0, 32, buffer);
    sprintf(buffer, "g %6.2f %6.2f %6.2f", data.gx, data.gy, data.gz);
    ssd1306_print(0, 48, buffer);

    // ssd1306_print(0,0, "000000000000000000000");
    // ssd1306_print(0,8, "111111111111111111111");
    // ssd1306_print(0,16,"222222222222222222222");
    // ssd1306_print(0,24,"333333333333333333333");
    // ssd1306_print(0,32,"444444444444444444444");
    // ssd1306_print(0,40,"555555555555555555555");
    // ssd1306_print(0,48,"666666666666666666666");
    // ssd1306_print(0,56,"777777777777777777777");



    // for (uint8_t y = 0; y < 64; y++) {
    //     for (uint8_t x = 0; x < 128; x++) {
    //         ssd1306_draw_pixel(x, y, 1);
    //         ssd1306_update();
    //         HAL_Delay(1);
    //     }
    // }
    // for (uint8_t y = 0; y < 64; y++) {
    //     for (uint8_t x = 0; x < 128; x++) {
    //         ssd1306_draw_pixel(x, y, 0);
    //         ssd1306_update();
    //         HAL_Delay(1);
    //     }
    // }
    ssd1306_update();


    HAL_Delay(100);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
      printf("Error_Handler");
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
