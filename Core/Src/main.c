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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
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
volatile uint16_t ADC_data[15];
volatile uint8_t cursor = 0;
volatile float temperatures[13];
volatile uint8_t EEPROM_data[256] = {0};
volatile uint8_t ADC_update_flag = 0; // [NA|NA|NA|NA|NA|NA|ADC2|ADC1]
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float V2T(float voltage, float B);
void I2C_reset();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** Disable the Internal Voltage Reference buffer
  */
  LL_VREFBUF_Disable();

  /** Configure the internal voltage reference buffer high impedance mode
  */
  LL_VREFBUF_EnableHIZ();

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  LL_PWR_DisableUCPDDeadBattery();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  // Enable I2C
  LL_I2C_Enable(I2C3);
  // Enable all I²C-slave interrupts we rely on
  LL_I2C_EnableIT_ADDR(I2C3);
  LL_I2C_EnableIT_STOP(I2C3);
  LL_I2C_EnableIT_NACK(I2C3);
  LL_I2C_EnableIT_ERR(I2C3);
  // Flush flags that CubeMX leaves uncleared
  LL_I2C_ClearFlag_ADDR(I2C3);
  LL_I2C_ClearFlag_STOP(I2C3);
  LL_I2C_ClearFlag_NACK(I2C3);

  // Enable ADC1 with DMA
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC_data[0]);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 7);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_ADC_REG_StartConversion(ADC1);

  // Enable ADC2 with DMA
  LL_ADC_Enable(ADC2);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC2));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&ADC2->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&ADC_data[7]);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 6);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_ADC_REG_StartConversion(ADC2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    if (ADC_update_flag & 1) {
      // ADC1 finished conversion sequence
      for (uint8_t i = 0; i < 7; i++) {
        EEPROM_data[i] = ADC_data[i] >> 4;
        temperatures[i] = V2T((float)ADC_data[i] / 4096.0f, 3950.0f);
        EEPROM_data[i + 128] = (uint8_t)(temperatures[i] + 40.0f);
      }
    }
    if (ADC_update_flag & 2) {
      // ADC1 finished conversion sequence
      for (uint8_t i = 7; i < 13; i++) {
        EEPROM_data[i] = ADC_data[i] >> 4;
        temperatures[i] = V2T((float)ADC_data[i] / 4096.0f, 3950.0f);
        EEPROM_data[i + 128] = (uint8_t)(temperatures[i] + 40.0f);
      }
    }
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(16000000);

  LL_SetSystemCoreClock(16000000);
}

/* USER CODE BEGIN 4 */
float V2T(float ratio, float B) {
  float R = ratio / (1.0f - ratio) * 47.0f;
  float T = 1.0f / ((logf(R / 100.0f) / B) + (1.0f / 298.15f));
  return T - 273.15f;
}

void I2C_reset() {
  cursor = 0;
  LL_I2C_Disable(I2C3);
  LL_I2C_ClearFlag_ADDR(I2C3);
  LL_I2C_ClearFlag_ARLO(I2C3);
  LL_I2C_ClearFlag_BERR(I2C3);
  LL_I2C_ClearFlag_NACK(I2C3);
  LL_I2C_ClearFlag_OVR(I2C3);
  LL_I2C_ClearFlag_STOP(I2C3);
  LL_I2C_ClearFlag_TXE(I2C3);
  LL_mDelay(1);
  LL_I2C_Enable(I2C3);
}
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
