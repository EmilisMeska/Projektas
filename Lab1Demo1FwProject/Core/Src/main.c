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
#include "math.h"
#include "sc_types.h"
#include "Statechart.h"
#include "Statechart_required.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h> 
#include <string.h>
#include "EventRecorder.h" 

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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BUFFERLEN 5
#define NUM_ADC_CHANNELS 2
#define ADC_RESOLUTION 4095
#define VREF 3.3    
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  32) // Size of array aADCxConvertedData[]
#define V_IN 3.3f
#define RC_SAMPLE_TIME_SEC 0.05f // 50 ms
#define R1_OHMS 10000.0f         // 10kO
#define R2_OHMS 100000.0f        // 100kO

/* Definition of ADCx conversions data table size */
/* Variable containing ADC conversions data */
uint8_t TxBuffer[100];   // TX data buffer
uint8_t RxBuffer[100];   // RX data buffer
uint16_t DataBufferADC[BUFFERLEN][NUM_ADC_CHANNELS];
uint16_t RawData[NUM_ADC_CHANNELS];
static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; // Variable containing ADC conversions data 
float AverageSensorsADC[NUM_ADC_CHANNELS];
float avg_current;
float Capacitance[2]; // Talpos [F]
float Capacitance_Avg; // Talpumo vidurkis
float Capacitance_Diff; // Talpumo skirtumas
Statechart sc_handle; // Statechart pointer
typedef enum {UART_READY,UART_BUSY,UART_FINISHED} UART_Status;
UART_Status Status=UART_READY;
#define ENABLE_EVENT_RECORDER
#define ENABLE_SLEEP


/* DEBUGGING AND PERFORMANCE DECLARATIONS (START) */
#define EVENT_ENTER_ID 0x1
#define EVENT_GOSLEEP_ID 0x2
#define EVENT_TIMER_ID 0x3
#define EVENT_ADC_ID 0x4
#define EVENT_DISPLAY_END_ID 0x5
#ifdef ENABLE_EVENT_RECORDER
#define DEBUG_EVENT_START() EventRecord2 (EVENT_ENTER_ID+EventLevelAPI, 0, 0)
#define DEBUG_EVENT_GOSLEEP() EventRecord2 (EVENT_GOSLEEP_ID+EventLevelAPI, 0, 0)
#define DEBUG_EVENT_TIMER() EventRecord2 (EVENT_TIMER_ID+EventLevelAPI, 0, 0)
#define DEBUG_EVENT_ADC() EventRecord2 (EVENT_ADC_ID+EventLevelAPI, 0, 0)
#define DEBUG_EVENT_DISPLAY() EventRecord2 (EVENT_DISPLAY_END_ID+EventLevelAPI, 0, 0)
#else
#define DEBUG_EVENT_START() __asm("NOP")
#define DEBUG_EVENT_GOSLEEP() __asm("NOP")
#define DEBUG_EVENT_TIMER() __asm("NOP")
#define DEBUG_EVENT_ADC() __asm("NOP")
#define DEBUG_EVENT_DISPLAY() __asm("NOP")
#endif
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
if (Status==UART_BUSY)
	Status=UART_READY; // transmition finished, can send more if needed
}
void HandleError()
{
	uint32_t uart_err;
	uart_err=HAL_UART_GetError(&huart2);
}
void MyErrorHandlerADC()
{
}
/* Start ADC conversion in channal whose number is passed as parameter "channel"*/
sc_integer statechart_startConvADC( Statechart* handle, const sc_integer channel)
{
	if (HAL_ADC_Start_DMA(&hadc,(uint32_t *)aADCxConvertedData, NUM_ADC_CHANNELS) != HAL_OK)
	{
		MyErrorHandlerADC();// handle error};
		return 0;
	}
	return 1;
}
/* Read ADC sample from register and store in DataBUffer */
sc_integer statechart_readADCSample( Statechart* handle, const sc_integer channel)
{	
	RawData[0]=aADCxConvertedData[0];
	RawData[1]=aADCxConvertedData[1];
	return 1;
}
sc_integer statechart_saveADCSample( Statechart* handle, const sc_integer channel, const sc_integer sample)
{
	DataBufferADC[sample][0]=RawData[0];
	DataBufferADC[sample][1]=RawData[1];
	return 1;
}
/* Calculate average values of acquired sensor readings */
void statechart_processData(Statechart* handle)
{
     uint32_t sum[NUM_ADC_CHANNELS] = {0};
    float voltages[NUM_ADC_CHANNELS];

    for (int i = 0; i < BUFFERLEN; i++)
    {
        for (int j = 0; j < NUM_ADC_CHANNELS; j++)
        {
            sum[j] += DataBufferADC[i][j];
        }
    }

    for (int j = 0; j < NUM_ADC_CHANNELS; j++)
    {
        uint32_t avg_adc = sum[j] / BUFFERLEN;
        voltages[j] = ((float)avg_adc / ADC_RESOLUTION) * VREF;
    }

    // Kanalas 0: R1 = 10k
    // Kanalas 1: R2 = 100k

    float ln_term_0 = 1.0f - (voltages[0] / V_IN);
    float ln_term_1 = 1.0f - (voltages[1] / V_IN);

    if (ln_term_0 > 0 && ln_term_1 > 0) {
        Capacitance[0] = -RC_SAMPLE_TIME_SEC / (R1_OHMS * logf(ln_term_0));
        Capacitance[1] = -RC_SAMPLE_TIME_SEC / (R2_OHMS * logf(ln_term_1));
    } else {
        Capacitance[0] = 0;
        Capacitance[1] = 0;
    }

    Capacitance_Diff = fabsf(Capacitance[0] - Capacitance[1]);
    Capacitance_Avg = (Capacitance[0] + Capacitance[1]) / 2.0f;
}
void statechart_displayInfo( Statechart* handle)
{
		#ifndef DISABLE_DISPLAY
    static char string_display[30];

    ssd1306_Fill(0);

    // Atvaizduoti vidurki
    ssd1306_SetCursor(0, 20);
    sprintf(string_display, "Avg: %.2fuF", Capacitance_Avg * 1e6);
    ssd1306_WriteString(string_display, Font_11x18, 1);

    // Atvaizduoti skirtuma
    ssd1306_SetCursor(0, 40);
    sprintf(string_display, "Dif: %.2fuF", Capacitance_Diff * 1e6);
    ssd1306_WriteString(string_display, Font_11x18, 1);

    ssd1306_UpdateScreen(&hi2c3);
		#endif
    DEBUG_EVENT_DISPLAY();
    __asm("NOP");
}
void statechart_sendInfo(void)
{
    if (Status != UART_READY) return;  // Don't transmit if UART is busy

    // Format the message with capacitance values in microFarads
    snprintf((char*)TxBuffer, sizeof(TxBuffer), "AVG: %.2f uF\r\nDIF: %.2f uF\r\n\r\n", 
             Capacitance_Avg * 1e6, Capacitance_Diff * 1e6);

    // Mark UART as busy
    Status = UART_BUSY;

    // Start transmission in interrupt mode
    if (HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer)) != HAL_OK) {
        HandleError();
        Status = UART_READY; // Recover if needed
    }
}
void statechart_readI2CSensor( Statechart* handle)
{
}
sc_integer statechart_saveI2CSample( Statechart* handle, const sc_integer sample_no)
{
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	DEBUG_EVENT_TIMER();
	if (htim==&htim6)
	{
		HAL_ResumeTick(); // Enable SysTick after wake-up
		statechart_raise_ev_GetSample(&sc_handle); //raise event TimerIntr in statechart
		
		__NOP(); 
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	DEBUG_EVENT_ADC();
	HAL_ResumeTick(); // Enable SysTick after wake-up
	statechart_raise_ev_ADCSampleReady(&sc_handle); //raise event TimerIntr in statechart
}


//static void SYSCLKConfig_STOP(void)
//{
//	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	uint32_t pFLatency = 0;
///* Enable Power Control clock */
//	__HAL_RCC_PWR_CLK_ENABLE();
///* Get the Oscillators configuration according to the internal RCC registers */
//	HAL_RCC_GetOscConfig(&RCC_OscInitStruct);
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//	{
//		Error_Handler();
//	}
///* Get the Clocks configuration according to the internal RCC registers */
//	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);
//	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
//	clocks dividers */
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
//	{
//		Error_Handler();
//	}
//}

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
  MX_TIM6_Init();
  MX_ADC_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	#ifdef ENABLE_EVENT_RECORDER
		EventRecorderInitialize (EventRecordAll, 1); // initialize and start Event Recorder
	#endif
	ssd1306_Init(&hi2c3);
	
	#ifndef DISABLE_DISPLAY
	HAL_GPIO_WritePin(OLED_VCC_GPIO_Port,OLED_VCC_Pin,GPIO_PIN_SET); // apply Vcc to OLED display
// Init lcd using one of the stm32HAL i2c typedefs
 ssd1306_Init(&hi2c3);
 // Write data to local screenbuffer
// Write data to local screenbuffer
	ssd1306_SetCursor(0,0);
	ssd1306_WriteString("Haha Benis", Font_11x18, 1);
	ssd1306_SetCursor(0, 26);
	ssd1306_WriteString("Semestro projektas", Font_11x18, 1);
	ssd1306_SetCursor(0, 44);
	ssd1306_WriteString("2025", Font_11x18, 1); 
		#endif
// Copy all data from local screenbuffer to the screen
  ssd1306_UpdateScreen(&hi2c3);

	#ifndef DISABLE_PROCESSING
  statechart_init(&sc_handle); // initialize state machine
  statechart_enter(&sc_handle); // run state machine
	
	if(HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) //run TIM6 timer
   {
   }			
		#endif  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    #ifdef ENABLE_SLEEP
        HAL_SuspendTick(); // Disable SysTick
    #endif
    
    DEBUG_EVENT_GOSLEEP();
    
    #ifdef ENABLE_SLEEP
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    #endif
    
    statechart_sendInfo();  // Send data to computer
    HAL_Delay(1000);       // 1 second discretization period
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
	
  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00506682;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00506682;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Seg_B_Pin|Seg_F_Pin|Seg_A_Pin|Decimal_Point_Pin
                          |COM4_Pin|COM3_Pin|COM2_Pin|COM1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|OLED_VCC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RGB_LD1_GPIO_Port, RGB_LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Seg_B_Pin Seg_F_Pin Seg_A_Pin Decimal_Point_Pin
                           COM4_Pin COM3_Pin COM2_Pin COM1_Pin */
  GPIO_InitStruct.Pin = Seg_B_Pin|Seg_F_Pin|Seg_A_Pin|Decimal_Point_Pin
                          |COM4_Pin|COM3_Pin|COM2_Pin|COM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_1_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin OLED_VCC_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|OLED_VCC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SWITCH_S1_Pin */
  GPIO_InitStruct.Pin = SWITCH_S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWITCH_S1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_LD1_Pin */
  GPIO_InitStruct.Pin = RGB_LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RGB_LD1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

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
