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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Left GPIO_PIN_SET
#define Right GPIO_PIN_RESET

#define Up GPIO_PIN_SET
#define Down GPIO_PIN_RESET

#define Work GPIO_PIN_RESET
#define Sleep GPIO_PIN_SET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char rx_data[23], tx_data[14];

float def_az, def_el;
float angullar_az, angullar_el;

unsigned int raw_az, raw_el;
unsigned int Frequency_az, Frequency_el;

unsigned char Moving_az, Moving_el;

Motor Motor_AZ, Motor_EL;

Target_data Target;

size_t Size_Rx_UART;
size_t Size_Tx_UART;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // 0.595 мкс
{
    if (Target.Rx_data[19] != '\0')
    {
        HAL_UART_Receive_IT(&huart2, (uint8_t*)Target.Rx_data, Size_Rx_UART);
        return;
    };

    Motor_AZ.Status.Angular =
        (Target.Rx_data[3] - '0') * 100 +  // сотни
        (Target.Rx_data[4] - '0') * 10 +   // десятки
        (Target.Rx_data[5] - '0') +        // единицы
        (Target.Rx_data[6] - '0') * 0.1f;  // десятые
    if (Target.Rx_data[2] == '-') Motor_AZ.Status.Angular *= -1;

    Motor_EL.Status.Angular =
        (Target.Rx_data[10] - '0') * 100 +
        (Target.Rx_data[11] - '0') * 10 +
        (Target.Rx_data[12] - '0') +
        (Target.Rx_data[14] - '0') * 0.1f;
    if (Target.Rx_data[9] == '-') Motor_EL.Status.Angular *= -1;

    HAL_UART_Receive_IT(&huart2, (uint8_t*)Target.Rx_data, Size_Rx_UART);
}

void Read_AD_Conversion(Motor *Motor_xx) // 2.02 мкс
{
	ADC_HandleTypeDef* hadc = Motor_xx->Config.Convertor.Convertor;
	Status* status = &Motor_xx->Status;
	const Config* config = &Motor_xx->Config;
	const float alfa = config->Alfa;
	const float one_minus_alfa = 1.0f - alfa;

	HAL_ADC_Start(hadc);
	if (HAL_ADC_PollForConversion(hadc, 100) != HAL_OK)
	{
		HAL_ADC_Stop(hadc);
		return;
	};

	const uint32_t raw_value = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop(hadc);

	const int32_t min_discrete = config->Convertor.Minimum_discrete_level;
	const int32_t max_discrete = config->Convertor.Maximum_discrete_level;
	const float min_angular = config->Angular.Minimum_angular;
	const float max_angular = config->Angular.Maximum_angular;

	const float filtered_discrete = alfa * (float)raw_value + one_minus_alfa * status->filtered_Discrete_level;
	status->filtered_Discrete_level = filtered_discrete;
	status->Discrete_level = filtered_discrete;

	float norma = 0.0f;
	const int32_t discrete_range = max_discrete - min_discrete;

	if (discrete_range > 0)
	{
		norma = (float)((int32_t)filtered_discrete - min_discrete) * config->Convertor.Inv_discrete_range;
		norma = (norma < 0.0f) ? 0.0f : (norma > 1.0f) ? 1.0f : norma;
	}

	const float angular = min_angular + norma * (max_angular - min_angular);
	const float filtered_angular = alfa * angular + one_minus_alfa * status->filter_Angular;

	status->filter_Angular = filtered_angular;
	status->Angular = filtered_angular;
};

__attribute__((always_inline)) inline char Working_area(Motor *Motor_xx) // 0.31 мкс
{
    register const float a = Motor_xx->Status.Angular;
    register const float* c = &Motor_xx->Config.Angular.Minimum_angular;
    register const float lower = c[0] + c[3];
    register const float upper = c[1] - c[3];
    return (a > lower) & (a < upper);
};

void Transmet_computer(void)
{
    const int32_t az_tenths = (int32_t)(Motor_AZ.Status.Angular * 10.0f);
    const uint16_t az_abs = (az_tenths < 0) ? -az_tenths : az_tenths;

    Target.Tx_data[1] = '1' - (az_tenths >> 31);
    Target.Tx_data[2] = '0' + (az_abs / 1000);
    Target.Tx_data[3] = '0' + (az_abs % 1000 / 100);
    Target.Tx_data[4] = '0' + (az_abs % 100 / 10);
    Target.Tx_data[5] = '0' + (az_abs % 10);

    const int32_t el_tenths = (int32_t)(Motor_EL.Status.Angular * 10.0f);
    const uint16_t el_abs = (el_tenths < 0) ? -el_tenths : el_tenths;

    Target.Tx_data[6] = '1' - (el_tenths >> 31);
    Target.Tx_data[7] = '0' + (el_abs / 1000);
    Target.Tx_data[8] = '0' + (el_abs % 1000 / 100);
    Target.Tx_data[9] = '0' + (el_abs % 100 / 10);
    Target.Tx_data[10] = '0' + (el_abs % 10);

    HAL_UART_Transmit(&huart2, (uint8_t*)Target.Tx_data, 13, 100);
}

void Set_PWM_Frequency(Motor *Motor_xx, uint32_t freq)
{
	if(Motor_xx == NULL || Motor_xx->Config.PWM.Timer == NULL)
	{
		return;
	}
	else
	{
		freq = CLAMP(freq, Motor_xx->Config.PWM.Minimum_frequency, Motor_xx->Config.PWM.Maximum_frequency);

		uint32_t prescaler = 0;
		uint32_t period = (HAL_RCC_GetPCLK1Freq() * 2 / freq) - 1;

		while (period > 0xFFFF)
		{
			prescaler++;
			period = (HAL_RCC_GetPCLK1Freq() * 2 / (freq * (prescaler + 1))) - 1;
		}

		HAL_TIM_PWM_Stop(Motor_xx->Config.PWM.Timer, TIM_CHANNEL_1);
		Motor_xx->Config.PWM.Timer->Instance->PSC = prescaler;
		Motor_xx->Config.PWM.Timer->Instance->ARR = period;
		Motor_xx->Config.PWM.Timer->Instance->CCR1 = period / 2;
		HAL_TIM_PWM_Start(Motor_xx->Config.PWM.Timer, TIM_CHANNEL_1);
		Motor_xx->Status.Frequency = freq;
	};
};

void Start_motor(Target_data *Target, Motor *Motor_xx)
{
	Motor_xx->Status.Frequency = Motor_xx->Config.PWM.Minimum_frequency;
	Set_PWM_Frequency(Motor_xx, Motor_xx->Status.Frequency);
	HAL_TIM_PWM_Start(Motor_xx->Config.PWM.Timer, TIM_CHANNEL_1);

	GPIO_PinState roter;
	if (Motor_xx->Config.GPIO.DIR_port == Motor_AZ.Config.GPIO.DIR_port && Motor_xx->Config.GPIO.DIR_pin == Motor_AZ.Config.GPIO.DIR_pin)
	{
		roter = (Target->Azimuth_difference > 0) ? Left : Right;
	}
	else
	{
		roter = (Target->Elevation_difference > 0) ? Up : Down;
	};
	HAL_GPIO_WritePin(Motor_xx->Config.GPIO.DIR_port, Motor_xx->Config.GPIO.DIR_pin, roter);

	HAL_GPIO_WritePin(Motor_xx->Config.GPIO.ENA_port, Motor_xx->Config.GPIO.ENA_pin, Work);
	Motor_xx->Status.Moving = 1;
};

void Up_fequency(Motor *Motor_xx)
{
	if (HAL_GetTick() - Motor_xx->Status.Last_freq_update_time >= 100)
	{
		if (Motor_xx->Status.Frequency < Motor_xx->Config.PWM.Maximum_frequency)
		{
			Motor_xx->Status.Frequency += Motor_xx->Config.PWM.Increment_frequency;
			Motor_xx->Status.Frequency = MIN(Motor_xx->Status.Frequency, Motor_xx->Config.PWM.Maximum_frequency);
			Set_PWM_Frequency(Motor_xx, Motor_xx->Status.Frequency);
			Motor_xx->Status.Last_freq_update_time = HAL_GetTick();
		};
	};
};

void Stop_motor(Motor *Motor_xx)
{
	HAL_TIM_PWM_Stop(Motor_xx->Config.PWM.Timer, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(Motor_xx->Config.GPIO.ENA_port, Motor_xx->Config.GPIO.ENA_pin, 1);
	Motor_xx->Status.Moving = 0;
};

void Moving_away_from_borders(Motor *Motor_xx, unsigned int TimeOut)
{
	Motor_xx->Status.Frequency = Motor_xx->Config.PWM.Minimum_frequency;
	Set_PWM_Frequency(Motor_xx, Motor_xx->Status.Frequency);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	GPIO_PinState roter;
	if (Motor_xx->Config.GPIO.DIR_port == Motor_AZ.Config.GPIO.DIR_port && Motor_xx->Config.GPIO.DIR_pin == Motor_AZ.Config.GPIO.DIR_pin)
	{
		roter = (Motor_xx->Status.Angular > 0) ? Right : Left;
	}
	else
	{
		roter = (Motor_xx->Status.Angular > 0) ? Down : Up;
	};
	HAL_GPIO_WritePin(Motor_xx->Config.GPIO.DIR_port, Motor_xx->Config.GPIO.DIR_pin, roter);

	HAL_GPIO_WritePin(Motor_xx->Config.GPIO.ENA_port, Motor_xx->Config.GPIO.ENA_pin, Work);
	HAL_Delay(TimeOut);

	HAL_GPIO_WritePin(Motor_xx->Config.GPIO.ENA_port, Motor_xx->Config.GPIO.ENA_pin, Sleep);
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	/* Turret */
		/* Config */
			/* GPIO */
				Motor_AZ.Config.GPIO.DIR_port = GPIOE; Motor_AZ.Config.GPIO.DIR_pin = GPIO_PIN_8;
				Motor_EL.Config.GPIO.DIR_port = GPIOE; Motor_EL.Config.GPIO.DIR_pin = GPIO_PIN_9;

				Motor_AZ.Config.GPIO.ENA_port = GPIOE; Motor_AZ.Config.GPIO.ENA_pin = GPIO_PIN_11;
				Motor_EL.Config.GPIO.ENA_port = GPIOE; Motor_EL.Config.GPIO.ENA_pin = GPIO_PIN_12;

			/* PWM */
				Motor_AZ.Config.PWM.Timer = &htim3;
				Motor_EL.Config.PWM.Timer = &htim2;

				Motor_AZ.Config.PWM.Maximum_frequency = 200000; Motor_AZ.Config.PWM.Minimum_frequency = 20000; Motor_AZ.Config.PWM.Increment_frequency = 10000;
				Motor_EL.Config.PWM.Maximum_frequency = 200000; Motor_EL.Config.PWM.Minimum_frequency = 20000; Motor_EL.Config.PWM.Increment_frequency = 10000;

			/* Convertor */
				Motor_AZ.Config.Convertor.Convertor = &hadc1;
				Motor_EL.Config.Convertor.Convertor = &hadc2;

				Motor_AZ.Config.Convertor.Maximum_discrete_level = 2150; Motor_AZ.Config.Convertor.Minimum_discrete_level = 1550; Motor_AZ.Config.Convertor.Middle_discrete_level = 1850;
				Motor_EL.Config.Convertor.Maximum_discrete_level = 2150; Motor_EL.Config.Convertor.Minimum_discrete_level = 2050; Motor_EL.Config.Convertor.Middle_discrete_level = 2100;

				Motor_AZ.Config.Convertor.Inv_discrete_range = 1.0f / (Motor_AZ.Config.Convertor.Maximum_discrete_level - Motor_AZ.Config.Convertor.Minimum_discrete_level);
				Motor_EL.Config.Convertor.Inv_discrete_range = 1.0f / (Motor_EL.Config.Convertor.Maximum_discrete_level - Motor_EL.Config.Convertor.Minimum_discrete_level);
			/* Angular */
				Motor_AZ.Config.Angular.Maximum_angular = 270.0f; Motor_AZ.Config.Angular.Minimum_angular = -270.0f;
				Motor_EL.Config.Angular.Maximum_angular = 90.0f; Motor_EL.Config.Angular.Minimum_angular = -20.0f;

				Motor_AZ.Config.Angular.Deviation = 50.0f; Motor_AZ.Config.Angular.Guidance_accuracy = 1.0f;
				Motor_EL.Config.Angular.Deviation = 5.0f; Motor_EL.Config.Angular.Guidance_accuracy = 1.0f;

			/* Alfa */
				Motor_AZ.Config.Alfa = 0.2f;
				Motor_EL.Config.Alfa = 0.2f;

		/* Status */
			Motor_AZ.Status.Angular = 0.0f; Motor_AZ.Status.filter_Angular = 0.0f;
			Motor_EL.Status.Angular = 0.0f; Motor_EL.Status.filter_Angular = 0.0f;

			Motor_AZ.Status.Discrete_level = 1550; Motor_AZ.Status.filtered_Discrete_level = 1550;
			Motor_EL.Status.Discrete_level = 1550; Motor_EL.Status.filtered_Discrete_level = 1550;

			Motor_AZ.Status.Frequency = 20000;
			Motor_EL.Status.Frequency = 20000;

			Motor_AZ.Status.Moving = 0;
			Motor_EL.Status.Moving = 0;

	/* Raspberry Pi */
		Target.Rx_data[0] = 'A'; Target.Rx_data[1] = 'z';
		Target.Rx_data[2] = '+'; 	Target.Rx_data[3] = '0';  Target.Rx_data[4] = '0';  Target.Rx_data[5] = '0';  /* , */ ; Target.Rx_data[6] = '0';

		Target.Rx_data[7] = 'E'; Target.Rx_data[8] = 'l';
		Target.Rx_data[9] = '-'; 	Target.Rx_data[10] = '0'; Target.Rx_data[11] = '0'; Target.Rx_data[12] = '0'; /* , */ ; Target.Rx_data[13] = '0';

		Target.Rx_data[14] = 'F'; Target.Rx_data[15] = 'm';
		Target.Rx_data[16] = '9';

		Target.Rx_data[17] = '\r'; Target.Rx_data[18] = '\n'; Target.Rx_data[19] = '\0';

		Target.Tx_data[0] = 'T';
		Target.Tx_data[11] = '\r';
		Target.Tx_data[12] = '\n';

	Size_Rx_UART = sizeof(Target.Rx_data);
	Size_Tx_UART = sizeof(Target.Tx_data);
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t*)Target.Rx_data, Size_Rx_UART);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Read_AD_Conversion(&Motor_AZ);
	  Read_AD_Conversion(&Motor_EL);

	  if (Working_area(&Motor_AZ))
	  {
		  if (fabsf(Target.Azimuth_difference) > Motor_AZ.Config.Angular.Guidance_accuracy)
		  {
			  if (!Motor_AZ.Status.Moving)
			  {
				  Start_motor(&Target, &Motor_AZ);
			  }
			  else
			  {
				  Up_fequency(&Motor_AZ);
			  }
		  }
		  else if (fabsf(Target.Azimuth_difference) > 0)
		  {
			  // Сопровождение_цели
		  }
		  else
		  {
			  Stop_motor(&Motor_AZ);
			  // Поведение_не_определено
		  };

	  }
	  else
	  {
		  Stop_motor(&Motor_AZ);
		  Moving_away_from_borders(&Motor_AZ, 1000);
	  };

	  if (Working_area(&Motor_EL))
	  {
		  if (fabsf(Target.Elevation_difference) > Motor_EL.Config.Angular.Guidance_accuracy)
		  {
			  if (!Motor_EL.Status.Moving)
			  {
				  Start_motor(&Target, &Motor_EL);
			  }
			  else
			  {
				  Up_fequency(&Motor_EL);
			  };
		  }
		  else if (fabsf(Target.Elevation_difference) > 0)
		  {

			  // Сопровождение_цели
		  }
		  else
		  {
			  Stop_motor(&Motor_EL);
			  // Поведение_не_определено
		  };
	  }
	  else
	  {
		  Stop_motor(&Motor_EL);
		  Moving_away_from_borders(&Motor_EL, 100);
	  };

	  Transmet_computer();
    /* USER CODE END WHILE */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pins : PE8 PE9 PE11 PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	//HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
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
