/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */

int var = 0;
int brilho = 1;		//valor sempre positivo que não pode passar de 45

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
//void DigitExtract(int);
//void Display(int,int);
int AnalogHandler(int);
void LEDHandler(int);
void Set_LED (int, int , int , int );
void Set_Brightness(int);
void WS2512_Send (void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MAX_LED 20			//numero máximo de leds para acender em sequencia
#define MAX_Brightness 45	// brilho máximo entre 0 e 45
#define USE_BRIGHTNESS 1

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];	//para o brilho

uint16_t readValue;

int datasentflag = 0;

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  WS2512_Send();

  HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		HAL_ADC_PollForConversion(&hadc1, 1000);
		readValue = HAL_ADC_GetValue(&hadc1);

		var = AnalogHandler(readValue);
		LEDHandler(var);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/**
 * @brief  Essa manipula a quantidade de leds que serão acionados mediante a entrada analógica
 * @param	Value 	inteiro de 0 a 100.
 */
void LEDHandler(int Value) {

	int brilho = 0;

	brilho = Value * MAX_Brightness / 100;

	//lógica para acionamento sequencial

	if (Value >= 0 && Value < 10) {
		Set_LED(0, 255, 0, 0);
		Set_LED(1, 0, 0, 0);
		Set_LED(2, 0, 0, 0);
		Set_LED(3, 0, 0, 0);
		Set_LED(4, 0, 0, 0);
		Set_LED(5, 0, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
	} else if (Value >= 11 && Value < 20) {
		Set_LED(0, 255, 0, 0);
		Set_LED(1, 255, 0, 0);
		Set_LED(2, 0, 0, 0);
		Set_LED(3, 0, 0, 0);
		Set_LED(4, 0, 0, 0);
		Set_LED(5, 0, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
	} else if (Value >= 21 && Value < 30) {
		Set_LED(0, 255, 0, 0);
		Set_LED(1, 255, 0, 0);
		Set_LED(2, 255, 0, 0);
		Set_LED(3, 0, 0, 0);
		Set_LED(4, 0, 0, 0);
		Set_LED(5, 0, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
	} else if (Value >= 31 && Value < 40) {
		Set_LED(0, 255, 0, 0);
		Set_LED(1, 255, 0, 0);
		Set_LED(2, 255, 0, 0);
		Set_LED(3, 255, 0, 0);
		Set_LED(4, 0, 0, 0);
		Set_LED(5, 0, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
	} else if (Value >= 41 && Value < 50) {
		Set_LED(0, 255, 0, 0);
		Set_LED(1, 255, 0, 0);
		Set_LED(2, 255, 0, 0);
		Set_LED(3, 255, 0, 0);
		Set_LED(4, 255, 0, 0);
		Set_LED(5, 0, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
	} else if (Value >= 51 && Value < 60) {
		Set_LED(0, 255, 0, 0);
		Set_LED(1, 255, 0, 0);
		Set_LED(2, 255, 0, 0);
		Set_LED(3, 255, 0, 0);
		Set_LED(4, 255, 0, 0);
		Set_LED(5, 255, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
	} else if (Value >= 61 && Value < 70) {
		Set_LED(0, 255, 0, 0);
		Set_LED(1, 255, 0, 0);
		Set_LED(2, 255, 0, 0);
		Set_LED(3, 255, 0, 0);
		Set_LED(4, 255, 0, 0);
		Set_LED(5, 255, 0, 0);
		Set_LED(6, 255, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
	} else if (Value >= 71 && Value < 80) {
		Set_LED(0, 255, 0, 0);
		Set_LED(1, 255, 0, 0);
		Set_LED(2, 255, 0, 0);
		Set_LED(3, 255, 0, 0);
		Set_LED(4, 255, 0, 0);
		Set_LED(5, 255, 0, 0);
		Set_LED(6, 255, 0, 0);
		Set_LED(7, 255, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
	} else if (Value >= 81 && Value < 90) {
		Set_LED(0, 255, 0, 0);
		Set_LED(1, 255, 0, 0);
		Set_LED(2, 255, 0, 0);
		Set_LED(3, 255, 0, 0);
		Set_LED(4, 255, 0, 0);
		Set_LED(5, 255, 0, 0);
		Set_LED(6, 255, 0, 0);
		Set_LED(7, 255, 0, 0);
		Set_LED(8, 255, 0, 0);
		Set_LED(9, 0, 0, 0);
	} else if (Value >= 91 && Value < 100) {
		Set_LED(0, 255, 0, 0);
		Set_LED(1, 255, 0, 0);
		Set_LED(2, 255, 0, 0);
		Set_LED(3, 255, 0, 0);
		Set_LED(4, 255, 0, 0);
		Set_LED(5, 255, 0, 0);
		Set_LED(6, 255, 0, 0);
		Set_LED(7, 255, 0, 0);
		Set_LED(8, 255, 0, 0);
		Set_LED(9, 255, 0, 0);
	}

	Set_Brightness(brilho);
	WS2512_Send();
	HAL_Delay(50);
}

/**
 * @brief  Essa função executa a conversão do valor em bits da leitura analógica 1 para um range de 0 a 100%.
 * @param	Value 	inteiro de 0 a 4095 representa o valor em bits da leitura analógica do ADC1.
 * @retval Value	inteiro de 0 a 100
 */
int AnalogHandler(int Value) {

	Value = Value * 100 / 4095;

	if (Value > 100) {
		Value = 100;
	} else if (Value < 0) {
		Value = 0;
	}
	return Value;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag = 1;
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

#define PI 3.14159265

void Set_Brightness (int brightness) // 0~45 linearização do brilho
{
#if USE_BRIGHTNESS

	if(brightness > 45) brightness = 45;
	for(int i=0; i<MAX_LED;i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1;j<4;j++)
		{
			float angle = 90-brightness; // em graus
			angle = angle*PI / 180; //em radianos
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}

	}
}

#endif

uint16_t pwmData[(24*MAX_LED+50)];

void WS2512_Send (void)
{
	uint32_t indx=0;
	uint32_t color;

//	for (int i=0; i<50; i++)
//	{
//		pwmData[indx] = 0;
//		indx++;
//	}

	for (int i=0;i<MAX_LED;i++)
	{
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 60;		// pulso alto, 2/3 de 90, aprox 68%
			}

			else pwmData[indx] = 30;	// pulso baixo, 1/3 de 90, aprox 32%

			indx++;
		}
	}

	for(int i=0;i<50;i++)				//intervalor de tempo de 50us antes da próxima msg
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while(!datasentflag){};
	datasentflag = 0;
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
