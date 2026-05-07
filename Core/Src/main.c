/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "Statechart.h"
#include "Statechart_required.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "OLED_SSD1306.h"
#include "GFX_BW.h"
#include "fonts.h"

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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t lcd_timer = 0;
uint32_t uart_timer = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Statechart sc_handle;

// Konstantos
#define SAMPLE_COUNT 200
#define VREF         3.3f
#define ADC_MAX      4095.0f
#define SHUNT_OHM    10.0f    // shunt reiksme ohmais
#define GAIN          1.53f// (R3+R4)/R2   15,04/9,82

//Kintamieji
float V_BIAS =      1.66f; 
float    RMS_Value;
float    Amplitude;
float v_out;
float v_shunt;

// ADC
uint16_t DataBufferADC[SAMPLE_COUNT];  
uint16_t ADC_VAL  = 0; 
uint16_t ADC_Sample = 0;


uint8_t TxBuffer[30];
int i;
typedef enum {UART_READY,UART_BUSY,UART_FINISHED} UART_Status;
UART_Status Status=UART_READY;

//Mygtukai
//B1 Kalibravimas
volatile uint32_t cal_msg_until = 0;
volatile uint8_t  cal_done      = 0;

//B2 paskutinės reikšmės OLED ekrane rodymas
volatile uint8_t hold_active  = 0;  
float RMS_Hold    = 0.0f;
float Amplitude_Hold = 0.0f;

// Debug
volatile int debug_flag_uart = 0;
volatile int debug_flag_lcd  = 0;


void HandleError_Uart()
{
uint32_t uart_err;
uart_err=HAL_UART_GetError(&huart2);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
if (Status==UART_BUSY)
 Status=UART_READY; // transmition finished, can send more if needed
}

void statechart_startConvADC( Statechart* handle)
{
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)&ADC_VAL, 1);
}



sc_integer statechart_readADCSample( Statechart* handle, const sc_integer channel)
{
	ADC_Sample=ADC_VAL;
	
	return 1;
}

sc_integer statechart_saveADCSample( Statechart* handle, const sc_integer channel, const sc_integer sample)
{
	 if ((uint32_t)sample < SAMPLE_COUNT)
        DataBufferADC[sample] = ADC_VAL;
    return 1;
	
}

void Calibrate_Bias(void)
{
	// Paimam vidurkį iš jau surinkto buferio
    float sum = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++)
        sum += ((float)DataBufferADC[i] / ADC_MAX) * VREF;
    V_BIAS = sum / SAMPLE_COUNT;
}

void statechart_processData(Statechart* handle)
{
    float sum_sq = 0.0f;
    float peak   = 0.0f;

    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        // ADC reikšmė -> įtampa
					v_out = ((float)DataBufferADC[i] / ADC_MAX) * VREF;
        // Įtampos ant sunto skaiciavimas
         v_shunt = (v_out - V_BIAS) / GAIN;

        // Įtampa -> srovė (mA)
        float current_mA = (v_shunt / SHUNT_OHM) * 1000.0f;
			

        // RMS
        sum_sq += current_mA * current_mA;
		
        // Amplitude
        float abs_cur = fabsf(current_mA);
			
        if (abs_cur > peak) peak = abs_cur;
    }

    RMS_Value = sqrtf(sum_sq / SAMPLE_COUNT);
    Amplitude = peak;
		
		if (Amplitude > 100.0f)
        HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
}


void statechart_displayInfo(Statechart* handle)
{
    static char buf[32];

    SSD1306_Clear(BLACK);
    GFX_SetFont(font_8x5);
    GFX_SetFontSize(2);

    // Kalibravimo pranešimas 1.5s
    if (cal_done && HAL_GetTick() < cal_msg_until)
    {
        sprintf(buf, "%.3fV OK", V_BIAS);
        GFX_DrawString(0, 20, buf, WHITE, 0);
        SSD1306_Display();
        return;  
    }
    cal_done = 0;  // pranešimas baigėsi
	
		
    // Įprasti matavimai
    float show_rms = hold_active ? RMS_Hold       : RMS_Value;
    float show_amp = hold_active ? Amplitude_Hold : Amplitude;

    sprintf(buf, "RMS:%.2f mA", show_rms);
    GFX_DrawString(0, 0, buf, WHITE, 0);

    sprintf(buf, "AMP:%.2f mA", show_amp);
    GFX_DrawString(0, 22, buf, WHITE, 0);

    GFX_SetFontSize(1);
		sprintf(buf, "%s", hold_active ? "HOLD" : "    "),
		GFX_DrawString(0, 52, buf, WHITE, 0);
		
    SSD1306_Display();
    debug_flag_lcd++;
}


void statechart_sendUART( Statechart* handle)
{
	debug_flag_uart++;
	
if (Status == UART_READY)
    {
        snprintf((char*)TxBuffer, sizeof(TxBuffer), "RMS=%.2f mA, AMP=%.2f mA\r\n", RMS_Value, Amplitude);

        if (HAL_UART_Transmit_IT(&huart2, (uint8_t*)TxBuffer, strlen((char*)TxBuffer)) == HAL_OK)
        {
            Status = UART_BUSY;
        }
    }
	
}



void statechart_checkButtons(Statechart* handle)
{
    static uint32_t last_B1 = 0, last_B2 = 0;
    static uint32_t B1_press_start = 0;
    uint32_t now = HAL_GetTick();

    // Button_1 - V_BIAS kalibracija
    if (HAL_GPIO_ReadPin(Button_1_GPIO_Port, Button_1_Pin) == GPIO_PIN_RESET)
    {
        if (B1_press_start == 0)
            B1_press_start = now;

        if ((now - B1_press_start) >= 2000)
        {
            B1_press_start = 0;
            last_B1 = now;

            Calibrate_Bias();

            // Nustatom laiką kiek rodyti pranešimą
            cal_msg_until = now + 1500;
            cal_done = 1;
        }
    }
    else
    {
        B1_press_start = 0;
    }

    // Button_2 - Hold on/off
    if (HAL_GPIO_ReadPin(Button_2_GPIO_Port, Button_2_Pin) == GPIO_PIN_RESET)
    {
        if ((now - last_B2) >= 300)
        {
            last_B2 = now;
            hold_active = !hold_active;
            if (hold_active)
            {
                RMS_Hold       = RMS_Value;
                Amplitude_Hold = Amplitude;
            }
        }
    }

    // Button_3 
//    if (HAL_GPIO_ReadPin(Button_3_GPIO_Port, Button_3_Pin) == GPIO_PIN_RESET)
//    {

//    }
		
}



// Laikmaciu pertraukciu callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        // ADC GetSample
        statechart_raise_ev_GetSample(&sc_handle);
    }
		
		 else if (htim->Instance == TIM7)
    {
				static uint8_t tim_counter = 0;
        tim_counter++;

        // Kas 0.2s 
        statechart_raise_ev_SendUart(&sc_handle);
				statechart_raise_ev_Button(&sc_handle);
        // Kas 1s
        if (tim_counter >= 5)
        {
            tim_counter = 0;
            statechart_raise_ev_DisplayLCD(&sc_handle);
        }

    }
		
		
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	statechart_raise_ev_ADCSampleReady(&sc_handle); //raise event TimerIntr in statechart	
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED); // ADC kalibracija
	
statechart_init(&sc_handle);
statechart_enter(&sc_handle);


HAL_TIM_Base_Start_IT(&htim6);  //ADC laikmatis
HAL_TIM_Base_Start_IT(&htim7);  //OLED ir UART laikmatis

SSD1306_SpiInit(&hspi1);
SSD1306_Clear(BLACK);
SSD1306_Display();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  hadc.Init.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = ENABLE;
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
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim6.Init.Prescaler = 31;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 31999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 199;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SSD1306_RESET_GPIO_Port, SSD1306_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Button_1_Pin Button_2_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin|Button_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_4_Pin */
  GPIO_InitStruct.Pin = Button_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_3_Pin */
  GPIO_InitStruct.Pin = Button_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SSD1306_DC_Pin */
  GPIO_InitStruct.Pin = SSD1306_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SSD1306_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin SSD1306_RESET_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|SSD1306_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
