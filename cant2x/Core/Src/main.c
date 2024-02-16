/* /* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int flame_Flag = 0, mq2_Flag = 0, lm35_Flag = 0;
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailBox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void MX_GPIO_Init(void);
void SystemClock_Config(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);
void MX_CAN1_Init(void);
void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_Delay(100);
  HAL_Delay(100);
  TxHeader.DLC=8;
  TxHeader.RTR=CAN_RTR_DATA;
  TxHeader.IDE=CAN_ID_STD;
  TxHeader.ExtId=0x0;
  TxHeader.StdId=0x0AA;
  TxHeader.TransmitGlobalTime=DISABLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	   HAL_ADC_Start_IT(&hadc1); // Start ADC conversion for flame sensor
	    HAL_ADC_Start_IT(&hadc2); // Start ADC conversion for MQ2 sensor
	    HAL_ADC_Start_IT(&hadc3); // Start ADC conversion for LM35 sensor

	    if(flame_Flag == 1){
	        TxData[0] = 'F' ;
	        if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox) != HAL_OK)
	        {
	            HAL_UART_Transmit(&huart2,(uint8_t*)"Fire detected\r\n", 15, HAL_MAX_DELAY);
	            Error_Handler();
	        }
	        HAL_Delay(50);
	        flame_Flag = 0;
	    }
	    else
	    {
	        TxData[0] = 'N';
	        if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox) != HAL_OK)
	        {
	            HAL_UART_Transmit(&huart2,(uint8_t*)"No fire\r\n", 10, HAL_MAX_DELAY);
	            Error_Handler();
	        }
	        HAL_Delay(50);
	    }

	    if(mq2_Flag == 1){
	        TxData[1] = 'G' ;
	        if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox) != HAL_OK)
	        {
	            HAL_UART_Transmit(&huart2,(uint8_t*)"Gas detected\r\n", 14, HAL_MAX_DELAY);
	            Error_Handler();
	        }
	        HAL_Delay(50);
	        mq2_Flag = 0;
	    }
	    else
	    {
	        TxData[1] = 'N';
	        if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox) != HAL_OK)
	        {
	            HAL_UART_Transmit(&huart2,(uint8_t*)"No gas\r\n", 9, HAL_MAX_DELAY);
	            Error_Handler();
	        }
	        HAL_Delay(50);
	    }

	    if(lm35_Flag == 1){
	        TxData[2] = 'T' ;
	        if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox) != HAL_OK)
	        {
	            HAL_UART_Transmit(&huart2,(uint8_t*)"Temperature detected\r\n", 22, HAL_MAX_DELAY);
	            Error_Handler();
	        }
	        HAL_Delay(50);
	        lm35_Flag = 0;
	    }
	    else
	    {
	        TxData[2] = 'N';
	        if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox) != HAL_OK)
	        {
	            HAL_UART_Transmit(&huart2,(uint8_t*)"No temperature\r\n", 16, HAL_MAX_DELAY);
	            Error_Handler();
	        }
	        HAL_Delay(50);
	    }
	}


    // Similar logic for other sensors goes here...

    /* USER CODE END 3 */
  }
  /* USER CODE BEGIN 4 */

  /* USER CODE END 4 */



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{
	 ADC_ChannelConfTypeDef sConfig = {0};

	  /* USER CODE BEGIN ADC1_Init 1 */

	  /* USER CODE END ADC1_Init 1 */

	  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	  */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
  /* ADC1 initialization code goes here... */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC2_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	hadc2.Instance = ADC2;
	  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
  /* ADC2 initialization code goes here... */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC3_Init(void)
{
	  ADC_ChannelConfTypeDef sConfig = {0};

	  /* USER CODE BEGIN ADC3_Init 1 */

	  /* USER CODE END ADC3_Init 1 */

	  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	  */
	  hadc3.Instance = ADC3;
	  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc3.Init.ScanConvMode = DISABLE;
	  hadc3.Init.ContinuousConvMode = DISABLE;
	  hadc3.Init.DiscontinuousConvMode = DISABLE;
	  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc3.Init.NbrOfConversion = 1;
	  hadc3.Init.DMAContinuousRequests = DISABLE;
	  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	  if (HAL_ADC_Init(&hadc3) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_3;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
  /* ADC3 initialization code goes here... */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN1_Init(void)
{
	hcan1.Instance = CAN1;
	  hcan1.Init.Prescaler = 18;
	  hcan1.Init.Mode = CAN_MODE_NORMAL;
	  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
	  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
	  hcan1.Init.TimeTriggeredMode = DISABLE;
	  hcan1.Init.AutoBusOff = DISABLE;
	  hcan1.Init.AutoWakeUp = DISABLE;
	  hcan1.Init.AutoRetransmission = DISABLE;
	  hcan1.Init.ReceiveFifoLocked = DISABLE;
	  hcan1.Init.TransmitFifoPriority = DISABLE;
	  if (HAL_CAN_Init(&hcan1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN CAN1_Init 2 */
	   CAN_FilterTypeDef canFilterConfig;
	   canFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
	   canFilterConfig.SlaveStartFilterBank=14;
	   canFilterConfig.FilterBank=2;
	   canFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
	   canFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	   canFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	   canFilterConfig.FilterMaskIdLow=0x0000;
	   canFilterConfig.FilterMaskIdHigh=0xFF00;
	   canFilterConfig.FilterIdLow=0x0000;
	   canFilterConfig.FilterIdHigh=0x1500;
	   HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
  /* CAN1 initialization code goes here... */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{  huart2.Instance = USART2;
huart2.Init.BaudRate = 9600;
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

  /* USART2 initialization code goes here... */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /* GPIO initialization code goes here... */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        uint32_t flame = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        if(flame < 500){
            flame_Flag = 1;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);    // Yellow LED
        }
        else{
            flame_Flag = 0;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        }
    }
    else if(hadc->Instance == ADC2){
        uint32_t mq2 = HAL_ADC_GetValue(&hadc2);
        HAL_ADC_Stop(&hadc2);

        if(mq2 > 200){ // Adjust the condition based on your sensor readings
            mq2_Flag = 1;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);    // Orange LED
        }
        else{
            mq2_Flag = 0;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
        }
    }
    else{
        uint32_t lm35 = HAL_ADC_GetValue(&hadc3);
        float temperature = lm35*0.086*100;

        HAL_ADC_Stop(&hadc3);

        if(temperature > 60){ // Adjust the condition based on your sensor readings
            lm35_Flag = 1;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);    // Red LED
        }
        else{
            lm35_Flag = 0;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        }
    }
}

/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */

  /*Configure GPIO pin Output Level */

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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


