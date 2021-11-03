/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	GREEN, ORANGE, RED, BLUE
} Leds;

typedef enum
{
	NORMAL, EMERGENCY_SIT
} state;

state channel_1_state = NORMAL;
state channel_2_state = NORMAL;
state channel_3_state = NORMAL;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CRITICAL_INT_TEMP 65  //critical internal temperature (°C) at which emergency mode is activated
#define HYSTERESIS_INT_TEMP 5 //hysteresis with decreasing internal temperature
#define CRITICAL_EXT_TEMP 45  //critical external temperature (°C) at which emergency mode is activated
#define HYSTERESIS_EXT_TEMP 5     //hysteresis with decreasing external temperature
#define CRITICAL_POT_VALUE 90 //critical value from the potentiometer, at which the emergency mode is activated
#define HYSTERESIS_POT 10     //hysteresis with decreasing value from the potentiometer
#define DEFAULT_PRESCALLER 100 //set default blinking frequency 200 Hz
#define EMERGENCY_PRESCALLER_1 20000 //set emergency blinking frequency 1 Hz
#define EMERGENCY_PRESCALLER_2 8000 //set emergency blinking frequency 2.5 Hz
#define EMERGENCY_PRESCALLER_3 4000 //set emergency blinking frequency 5 Hz
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim4;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t MAP(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint32_t MAP_Invert(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	return out_max - (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

void IndicateADC(Leds led, uint32_t duty_cycle)
{
	uint8_t emergency_tot = channel_1_state + channel_2_state + channel_3_state; //total number of active emergencies
	if (emergency_tot == 0)
	{
		TIM4->CCR3 = 0;
		TIM4->PSC = DEFAULT_PRESCALLER;
		switch (led)
		{
		case GREEN:
		{
			TIM4->CCR1 = duty_cycle-1;
			break;
		}
		case ORANGE:
		{
			TIM4->CCR2 = duty_cycle-1;
			break;
		}
		case RED:
		{
			break;
		}
		case BLUE:
		{
			TIM4->CCR4 = duty_cycle-1;
			break;
		}
		}
	}
	else
	{
		TIM4->CCR1 = 0;
		TIM4->CCR2 = 0;
		TIM4->CCR4 = 0;
		TIM4->CCR3 = 50;
		switch (emergency_tot)
		{
		case 1:
		{
			TIM4->PSC = EMERGENCY_PRESCALLER_1;
			break;
		}
		case 2:
		{
			TIM4->PSC = EMERGENCY_PRESCALLER_2;
			break;
		}
		case 3:
		{
			TIM4->PSC = EMERGENCY_PRESCALLER_3;
			break;
		}
		}
	}
}

void GetPotentiometerVoltage(void)
{
	volatile HAL_StatusTypeDef adcPoolResult;
	uint32_t valuePot = 0;
	HAL_ADC_Start(&hadc1);
	adcPoolResult = HAL_ADC_PollForConversion(&hadc1, 1);
	if (adcPoolResult == HAL_OK)
	{
		valuePot = MAP(HAL_ADC_GetValue(&hadc1), 0, 4095, 1, 100);
		if ((valuePot) > CRITICAL_POT_VALUE)
		{
			channel_1_state = EMERGENCY_SIT;
		}
		else if ((valuePot + HYSTERESIS_POT) < CRITICAL_POT_VALUE)
		{
			channel_1_state = NORMAL;
		}
		IndicateADC(BLUE, valuePot);
	}
	HAL_ADC_Stop(&hadc1);
}

void GetInternalTemperature(void)
{
	volatile HAL_StatusTypeDef adcPoolResult;
	uint32_t internalTemp = 0, valueInTemp = 0;
	double voltageInTemp = 0;
	HAL_ADCEx_InjectedStart(&hadc1);
	adcPoolResult = HAL_ADCEx_InjectedPollForConversion(&hadc1, 100);
	if (adcPoolResult == HAL_OK)
	{
		valueInTemp = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1); // read value from internal temperature sensor
		voltageInTemp = (double) valueInTemp * (3.3 / 4096); //convert valueInTemp to voltage
		internalTemp = (voltageInTemp - 0.76) / 0.0025 + 25; //convert voltage to temperature in °C
		if (internalTemp > CRITICAL_INT_TEMP)
		{
			channel_2_state = EMERGENCY_SIT;
		}
		else if ((internalTemp + HYSTERESIS_INT_TEMP) < CRITICAL_INT_TEMP)
		{
			channel_2_state = NORMAL;
		}
		IndicateADC(ORANGE, internalTemp);
	}
	HAL_ADCEx_InjectedStop(&hadc1);
}

void GetExternalTemperature(void)
{
	volatile HAL_StatusTypeDef adcPoolResult;
	uint32_t externalTemp = 0, valueExTemp = 0;
	double voltageExTemp = 0;
	HAL_ADCEx_InjectedStart(&hadc1);
	adcPoolResult = HAL_ADCEx_InjectedPollForConversion(&hadc1, 100);
	if (adcPoolResult == HAL_OK)
	{
		valueExTemp = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2); // read value from external temperature sensor
		voltageExTemp = (double) valueExTemp * (3.3 / 4096) * 100; //convert valueExTemp to voltage (volts x100)
		externalTemp = MAP_Invert(voltageExTemp, 2, 202, 1, 100); //convert voltage to temperature in the range from 1 to 100°C. 2 - voltage x100 at a temperature of 100°C, 202  - voltage x100 at a temperature of 0°C
		if (externalTemp > CRITICAL_EXT_TEMP)
		{
			channel_3_state = EMERGENCY_SIT;
		}
		else if ((externalTemp + HYSTERESIS_EXT_TEMP) < CRITICAL_EXT_TEMP)
		{
			channel_3_state = NORMAL;
		}
		IndicateADC(GREEN, externalTemp);
	}
	HAL_ADCEx_InjectedStop(&hadc1);
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
	MX_ADC1_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	TIM4->ARR = 100-1; //set TIM4 counter period
	TIM4->PSC = DEFAULT_PRESCALLER; //set start prescaller for all led channel
	//set start duty cycle for all led channel
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1)
	{
		GetPotentiometerVoltage();
		GetInternalTemperature();
		GetExternalTemperature();
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };
	ADC_InjectionConfTypeDef sConfigInjected =
	{ 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
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
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_TEMPSENSOR;
	sConfigInjected.InjectedRank = 1;
	sConfigInjected.InjectedNbrOfConversion = 2;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_112CYCLES;
	sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
	sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
	sConfigInjected.InjectedRank = 2;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 16;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
