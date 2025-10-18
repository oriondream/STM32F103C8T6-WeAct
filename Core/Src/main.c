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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANSI_GREEN 	"\033[1;1H\x1b[32m"
#define ANSI_END 	"\x1b[0m"
#define ANSI_YELLOW "\x1b[33m"

#define ADC_MODE_POLLING 0
#define ADC_MODE_INTERRUPT 1
#define ADC_MODE ADC_MODE_INTERRUPT

#define FAST_TOGGLE_MS 40
#define SLOW_TOGGLE_MS 1000
#define FAST_TOGGLE_DURATION_MS 2000
#define RX_BUFFER_SIZE 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Variable to track if a full message has been received */
volatile uint8_t RxCpltFlag = 0;

/* Variable to track if a full message has been received */
volatile uint8_t ADCcpltFlag = 0;

/* LED toggle delay - volatile as it's modified in interrupt context */
volatile uint32_t nextToggleDelay = 1000;

/* Counter for button presses */
volatile uint32_t btnPressTime = 0;

/* Analog-Digital Conversion result*/
volatile uint16_t AD_RES = 0;

/* Buffer to store the received I2C data */
uint8_t RxDataBuffer[RX_BUFFER_SIZE];

CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;

uint8_t RxData[8];
uint8_t TxData[8];

int datacheck = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Set fast toggle mode for 2 seconds after button press */
	btnPressTime = HAL_GetTick();
	nextToggleDelay = FAST_TOGGLE_MS;
}

/**
  * @brief  Slave Receive complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  * the configuration information for I2C module.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* Check if the callback is from the correct I2C peripheral */
    if (hi2c->Instance == I2C2)
    {
        /* Set the flag to notify the main loop that data is ready */
        RxCpltFlag = 1;

        /* Restart I2C slave reception for next message */
        /* Note: This is done in callback to ensure continuous reception */
        HAL_I2C_Slave_Receive_IT(hi2c, (uint8_t*)hi2c->pBuffPtr - RX_BUFFER_SIZE, RX_BUFFER_SIZE);
    }
    nextToggleDelay = FAST_TOGGLE_MS;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	nextToggleDelay = FAST_TOGGLE_MS;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Read & Update The ADC Result
    AD_RES = HAL_ADC_GetValue(&hadc1);
    ADCcpltFlag = 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if (RxHeader.DLC == 8
		&& RxData[0] == 0
		&& RxData[1] == 0xFF
		&& RxData[2] == 0
		&& RxData[3] == 0xFF
		&& RxData[4] == 0
		&& RxData[5] == 0xFF
		&& RxData[6] == 0
		&& RxData[7] == 0xFF
		)
	{
		datacheck = 1;
	}
}

void clear_screen(UART_HandleTypeDef* huart)
{
	const char* CLEAR = "\033[2J\0";
	HAL_UART_Transmit(&huart, (uint8_t*)CLEAR, strlen(CLEAR), 100);
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
  MX_CAN_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /*##-2- Configure the CAN Filter ###########################################*/
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int count = 0;
  char cbuff[400]= {0};

  uint32_t last_receive_ms = HAL_GetTick();
  uint32_t last_update_ms = 0;

  /* * START THE ASYNCHRONOUS RECEPTION
   * * Parameters:
   * 1. &hi2c1: Pointer to the I2C handle structure (e.g., I2C1)
   * 2. RxDataBuffer: Pointer to the data buffer
   * 3. RX_BUFFER_SIZE: The number of bytes to receive
   */
  if (HAL_I2C_Slave_Receive_IT(&hi2c2, RxDataBuffer, RX_BUFFER_SIZE) != HAL_OK)
  {
      /* Error handling */
      Error_Handler();
  }

  	HAL_ADC_Stop(&hadc1);

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
    	// Calibration failed
    	Error_Handler(); // Define your error handler
    }

#if ADC_MODE==ADC_MODE_POLLING
    if (HAL_ADC_Start(&hadc1))
    {
        /* Error handling */
        Error_Handler();
    }
#elif ADC_MODE==ADC_MODE_INTERRUPT
    HAL_TIM_Base_Start(&htim3); // Start Timer3 (Trigger Source For ADC1)
    if (HAL_ADC_Start_IT(&hadc1))
    {
        /* Error handling */
        Error_Handler();
    }
#endif
    //TODO:
    clear_screen(&huart1);

    while (1)
    {
	/* Check if we should return to slow toggle mode */
	uint32_t now_ms = HAL_GetTick();
	if (btnPressTime > 0 && (now_ms - btnPressTime) > FAST_TOGGLE_DURATION_MS) {
		nextToggleDelay = SLOW_TOGGLE_MS;
		btnPressTime = 0; /* Reset timer */
	}

	/* Toggle LED at current rate */
	uint16_t LEDS = GPIO_PIN_2 | GPIO_PIN_6;
	if (datacheck == 1)
	{
		LEDS |= GPIO_PIN_3;
	}

	HAL_GPIO_TogglePin(GPIOB, LEDS);
	HAL_Delay(nextToggleDelay);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* Check if I2C data has been received */
	if (RxCpltFlag == 1)
	{
		++count;
		/* Only print the 3 bytes we actually received (RX_BUFFER_SIZE = 3) */
		sprintf(cbuff, ANSI_GREEN"[%5ld:%3ld]"ANSI_END" I2C2 "
				       ANSI_YELLOW"["ANSI_END"%3d %3d %3d"ANSI_YELLOW "]"ANSI_END,
			now_ms/1000,
			now_ms%1000,
			RxDataBuffer[0],
			RxDataBuffer[1],
			RxDataBuffer[2]
		);

		HAL_UART_Transmit(&huart1, (uint8_t*)cbuff, strlen(cbuff), 100);

		last_receive_ms = HAL_GetTick();
		
		/* Set fast toggle mode for 2 seconds after I2C receive */
		btnPressTime = HAL_GetTick();
		nextToggleDelay = FAST_TOGGLE_MS;
		
		RxCpltFlag = 0;
	}
	else {
		/* Periodically report no data status */
		if (now_ms - last_receive_ms >= 1000 && now_ms - last_update_ms >= 1000)
		{
			sprintf(cbuff, "No data for %lds\r\n", (now_ms - last_receive_ms) / 1000);
			last_update_ms = now_ms;
			HAL_UART_Transmit(&huart1, (uint8_t*)cbuff, strlen(cbuff), 100);
		}
	}

#if ADC_MODE==ADC_MODE_INTERRUPT
	if (ADCcpltFlag)
	{
		sprintf(cbuff, "\033[2;12H"
				       " ADC: %6d",
			AD_RES
		);

		HAL_UART_Transmit(&huart1, (uint8_t*)cbuff, strlen(cbuff), 100);
		ADCcpltFlag = 0;
	}
#else
	  HAL_ADC_PollForConversion(&hadc1, 1);
	  AD_RES = HAL_ADC_GetValue(&hadc1);
	  sprintf(cbuff, "\x1b[32m" "[%5d:%3d]"    "\x1b[0m"
			                  " ADC: %6d\r\n",
          now_ms/1000,
		  now_ms%1000,
		  AD_RES
	  );
	  HAL_UART_Transmit(&huart1, (uint8_t*)cbuff, strlen(cbuff), 100);
#endif
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

// STM32CubeMX performs bit shift to the assigned address
// The value of OwnAddress1 does NOT reflect the true address assigned
// in the Cube MX. It is double the value.

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 110;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 23;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_PB2_Pin|LED_CAN_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_KEY_Pin */
  GPIO_InitStruct.Pin = BTN_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_PB2_Pin LED_CAN_Pin PB6 */
  GPIO_InitStruct.Pin = LED_PB2_Pin|LED_CAN_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
  __disable_irq();
  nextToggleDelay = FAST_TOGGLE_MS;
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
