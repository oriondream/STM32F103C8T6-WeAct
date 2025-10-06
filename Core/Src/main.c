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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Variable to track if a full message has been received */
volatile uint8_t RxCpltFlag = 0;

/* LED toggle delay - volatile as it's modified in interrupt context */
volatile uint32_t nextToggleDelay = 1000;

/* Counter for button presses */
volatile uint32_t btnPressTime = 0;

#define FAST_TOGGLE_MS 40
#define SLOW_TOGGLE_MS 1000
#define FAST_TOGGLE_DURATION_MS 2000

/* Buffer to store the received data */
#define RX_BUFFER_SIZE 3
uint8_t RxDataBuffer[RX_BUFFER_SIZE];

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  //TODO:

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int count = 0;
  char cbuff[100]= {0};

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

  while (1)
  {
	/* Check if we should return to slow toggle mode */
	uint32_t now_ms = HAL_GetTick();
	if (btnPressTime > 0 && (now_ms - btnPressTime) > FAST_TOGGLE_DURATION_MS) {
		nextToggleDelay = SLOW_TOGGLE_MS;
		btnPressTime = 0; /* Reset timer */
	}

	/* Toggle LED at current rate */
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2 | GPIO_PIN_6);
	HAL_Delay(nextToggleDelay);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
	/* Check if I2C data has been received */
	if (RxCpltFlag == 1)
	{
		++count;
		/* Only print the 3 bytes we actually received (RX_BUFFER_SIZE = 3) */
		sprintf(cbuff, "\x1b[32m" "[%5d:%3d]"    "\x1b[0m"
				                  " received "
				       "\x1b[33m" "["            "\x1b[0m"
				                  "%3d %3d %3d"
				       "\x1b[33m" "]\r\n"        "\x1b[0m", now_ms/1000, now_ms%1000,
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
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_PB2_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_KEY_Pin */
  GPIO_InitStruct.Pin = BTN_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_PB2_Pin PB6 */
  GPIO_InitStruct.Pin = LED_PB2_Pin|GPIO_PIN_6;
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
