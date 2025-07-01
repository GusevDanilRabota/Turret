/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* Структуры STM32F407VGT6 */
typedef struct
{
  GPIO_TypeDef *DIR_port, *ENA_port;
  uint32_t      DIR_pin,   ENA_pin;
} Config_GPIO;

typedef struct
{
  TIM_HandleTypeDef *Timer;

  uint32_t Maximum_frequency, Minimum_frequency;

  uint32_t Increment_frequency;
} Config_TIM;

typedef struct
{
  ADC_HandleTypeDef *Convertor;

  uint32_t Maximum_discrete_level, Middle_discrete_level, Minimum_discrete_level;

  float Inv_discrete_range;
} Config_ADC;

typedef struct
{
  float Maximum_angular, Minimum_angular;

  float Guidance_accuracy; // Точность_прицелывания
  float Deviation; // Отступ_от_границ
} Config_Angular;

typedef struct
{
  Config_GPIO GPIO;
  Config_TIM PWM;
  Config_ADC Convertor;
  Config_Angular Angular;

  float Alfa;
} Config;

typedef struct
{
  uint32_t Frequency;
  uint32_t Last_freq_update_time;

  uint8_t Moving;

  float Discrete_level, filtered_Discrete_level;
  float Angular,        filter_Angular;
} Status;

typedef struct
{
  Status Status;
  Config Config;
} Motor;

extern Motor Motor_AZ, Motor_EL;
/* Структуры Raspberry Pi 5 */
typedef struct
{
  char Rx_data[20], Tx_data[13];

  float Azimuth_difference, Elevation_difference;

  char Fire_mode;
} Target_data;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define CLAMP(x, min, max) (MAX(min, MIN(x, max)))
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
