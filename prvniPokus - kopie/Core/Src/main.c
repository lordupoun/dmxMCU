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
#include "ili9341.h"
#include "ili9341_gfx.h"
//#include "ili9341_font.h"

//#include "snow_tiger.h"

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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ili9341_t *_lcd;
//ina260_t *_pow;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t uartBuff1[513];
ili9341_t *_screen;

ili9341_t *screen(void)
{
  return _screen;
}
int lcdStatus=0; //TODO: přepsat na uint
uint16_t x1;
uint16_t y1;
void screenTouchBegin(ili9341_t *lcd, uint16_t x, uint16_t y)
{


		ili9341_touch_coordinate(lcd, &x1, &y1);
		lcdStatus=1;
		//x1=x;
		//y1=y;


}

void screenTouchEnd(ili9341_t *lcd, uint16_t x, uint16_t y)
{
	//ili9341_draw_string(_screen, textAttr, "Cus picus!!!!!");
}
/*ili9341_t *screen(void)
{
  return _screen;
}*/

/*ina260_t *power(void)
{
  return _pow;
}*/



//uint8_t uartBuff2[513];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//uartBuff1[0]=127;
	uartBuff1[1]=255;
	uartBuff1[2]=255;
	uartBuff1[3]=255;
	/*for(int i=0;i<514;i++)
	{
		uartBuff1[i]='1';
		uartBuff2[i]='0';
	}*/
	//uartBuff1[513]='\n';
	  /* Enable I-Cache-------------------------------------------------------------*/
	  //SCB_EnableICache();

	  /* Enable D-Cache-------------------------------------------------------------*/
	  //SCB_EnableDCache();
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
 /* _pow = ina260_new(
        &hi2c3,
        INA260_SLAVE_ADDRESS);
*/

  _screen = ili9341_new(
      &hspi1,
      TFT_RESET_GPIO_Port, TFT_RESET_Pin,
      TFT_CS_GPIO_Port,    TFT_CS_Pin,
      TFT_DC_GPIO_Port,    TFT_DC_Pin,
      isoLandscape,
      TOUCH_CS_GPIO_Port,  TOUCH_CS_Pin,
      TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin,
      itsSupported,
      itnNormalized);
  ili9341_text_attr_t textAttr = (ili9341_text_attr_t){
     .font = &ili9341_font_11x18,
     .fg_color = ILI9341_WHITE,
     .bg_color = ILI9341_RED,
     .origin_x = 10,
     .origin_y = 10
   };
  ili9341_set_touch_pressed_begin(_screen, screenTouchBegin);
  ili9341_set_touch_pressed_end(_screen, screenTouchEnd);
  HAL_TIM_Base_Start_IT(&htim2);
  //ili9341_draw_line(_lcd, ILI9341_BLACK, 0,0, 100,100);
  	  //ili9341_fill_rect(_screen, ILI9341_YELLOW, 11,1, 22,8);
  	  ili9341_fill_screen(_screen, ILI9341_BLUE);
  	  HAL_Delay(1000);
  	ili9341_fill_screen(_screen, ILI9341_PINK);
  		  ili9341_draw_string(_screen, textAttr, "Cus picus!");
  		    	  //HAL_Delay(10000);
  		    	  //ili9341_fill_rect(_screen, ILI9341_YELLOW, 11,1, 22,8);
  		    	  //HAL_Delay(10000);
  	//ili9341_calibrate_3point(_screen,320,240);
  //HAL_UARTEx_ReceiveToIdle_IT(&huart1, uartBuff1, 1); //uartBuff1_SIZE; v gitu TEST si to bralo data a ukládalo je ve špatný chvíle (byly tam vícekrát a psalo je to na špatný pozice...); v YATu to bude vždycky rozhozený - záleží kdy ho spustím
  //HAL_UARTEx_ReceiveToIdle_IT(&huart1, uartBuff1, 1);
  //HAL_UART_Receive(&huart1, uartBuff1, 513,100); //Stačí dodělat automatický čtení a automatický odeslání -> Je jedno kdy se to odešle buffer se stejně mění nárazově... ne?
  //HAL_UART_Transmit(&huart1, uartBuff1, 513,100);
  //uartBuff
  //HAL_UART_Receive_IT(&huart1, uartBuff1, 513);
  //HAL_UART_Receive_IT(&huart1, uartBuff, 513);
  //-------------------------------------------------Globalni promenna jako flag -> volatile - standardne by nebyla dost rychla
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  if(lcdStatus==1)
	  {
		  ili9341_fill_screen(_screen, ILI9341_BLUE);
		  char int_str[20];
		  uint16_t test=5;
		  sprintf(int_str, "%d", x1);
		  ili9341_draw_string(_screen, textAttr, int_str);
		  //ili9341_draw_string(_screen, textAttr, "         "+y1);
		  lcdStatus=0;
	  }


	  //ILI9341_Fill_Screen(YELLOW);
	  //uartBuff1[0]=0;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_UARTEx_ReceiveToIdle_IT(&huart1, uartBuff1, 513);
	//HAL_UART_Receive(&huart1, uartBuff1, 513,100); //no jo ale když vypadne komunikace tak jsem v... -> timout musí být jako délka jednoho paketu
    //HAL_UART_Transmit(&huart1, uartBuff1, 513,100);
	  //HAL_UART_Receive_IT(&huart1, uartBuff1, 513);
	 //HAL_UARTEx_ReceiveToIdle_IT(&huart1, uartBuff1, 513);
	 //HAL_UART_Transmit(&huart1, "zadek", 513);
	 //HAL_Delay(100);
	   //dokud nedostane všech 513, nekončí -> to samé v interruptu -> v interruptu aktivovat nový, který bude čekat na načtení ale zárove�? odeslání nechat až při úspěšném odeslání -> (odelsání aktivuje čtení jednej a odesílání druhej, čtení aktivuje odeslání jednej a čtení druhej)
	  //HAL_Delay(100);

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1656;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 250000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
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
  huart2.Init.BaudRate = 250000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_2;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_DC_Pin|TFT_RESET_Pin|TFT_CS_Pin|GPIO_PIN_13
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_CS_Pin */
  GPIO_InitStruct.Pin = TOUCH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOUCH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_IRQ_Pin */
  GPIO_InitStruct.Pin = TOUCH_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_DC_Pin TFT_RESET_Pin TFT_CS_Pin PB13
                           PB8 */
  GPIO_InitStruct.Pin = TFT_DC_Pin|TFT_RESET_Pin|TFT_CS_Pin|GPIO_PIN_13
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case TOUCH_IRQ_Pin:
      ili9341_touch_interrupt(_screen);
      //ili9341_fill_screen(_screen, ILI9341_RED);
      break;

    default:
      // unhandled interrupt pin
      break;
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Transmit_IT(&huart1, uartBuff1, sizeof (uartBuff1));
	//HAL_UART_Receive_IT(&huart1, uartBuff2, 513);
	//HAL_UART_Transmit_IT(&huart1, "haha", sizeof ("haha"));
	//HAL_Delay(100);
	//HAL_UART_Receive_IT(&huart1, uartBuff1, 513);
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
/*void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit_IT(&huart1, uartBuff2, sizeof (uartBuff2));
	HAL_UART_Receive_IT(&huart1, uartBuff1 , 513);
	//HAL_UART_Transmit_IT(&huart1, uartBuff, sizeof (uartBuff));
	//HAL_Delay(1);
}*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) //nevykonává se, proč?
{
	//HAL_UARTEx_ReceiveToIdle_IT(&huart1, uartBuff1, 513);
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	//HAL_Delay(100);
	//HAL_UART_Transmit_IT(&huart1, uartBuff1, 513);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	//HAL_Delay(1);
	//if (htim->Instance == htim2.Instance)
	  //{
	HAL_UART_Transmit_IT(&huart1, uartBuff1, 513); //začne pozdě číst?
	  //}
	//HAL_UART_Transmit(&huart1, 255, 1,100); //Proč
	//HAL_UART_Transmit(&huart1, uartBuff1, 513,100);
    //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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
