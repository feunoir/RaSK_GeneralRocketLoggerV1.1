/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */


#include <stdlib.h>
#include "leastsquare.h"
#include "../uart_wrapper/uart_wrapper.h"

#include "LPS22HB.h"
#include "../MPU9250/MPU9250.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS SDFatFs;
FIL file;
char SDPath[4];
LPS22HB_t lps22hb;
MPU9250_t mpu9250;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_RTC_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
typedef enum {
	Rocket_NoStatus,
	Rocket_OnGround,
	Rocket_Launched,
	Rocket_Cutoff,
	Rocket_Falling,
	Rocket_Landed
} Rocket_Status_t;

typedef struct {
	Rocket_Status_t status;
	uint8_t d_pos;
	uint32_t d_num;
	uint32_t time[256];
	uint32_t pressraw[256];
	float press[256];
	float accel[256];
	float slope[256];

	//	temporary counter
	int counter;
}  Flight_Info_t;

/* Private function prototypes -----------------------------------------------*/
void Init_LoggerSystem(void);
void Config_OnePulseDelayTime(float delay);
void Config_Logdir(void);
int Check_Bootcount(void);
void Estimate_Status(Flight_Info_t* finfo);
void Trigger_ReleaseSystem(void);


char dir_syslog[20] = "SYSLOG.TXT";
char dir_datlog[20] = "DATLOG.CSV";

Flight_Info_t FlightInfo;

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  FIL file;     /* File object */
  char wbuff[256] = {}; /* File write buffer */
  FRESULT fres;
  uint32_t tmeasure_a, tmeasure_b;

#if UART_DEBUG
  char ubuff[256];
#endif

  //initialise_monitor_handles();



  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_RTC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  Init_LoggerSystem();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

	  tmeasure_a = TIM2->CNT;

	  fres = open_append(&file, dir_datlog);
	  if ( fres != FR_OK ) {
#if UART_DEBUG
		  xprintf("file open failure!\n");
#endif
		  HAL_Delay(100);
		  continue;
	  }
	  MPU9250_update(&mpu9250, UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
	  LPS22HB_GetData(&lps22hb);
	  FlightInfo.time[FlightInfo.d_pos] = TIM2->CNT;
	  FlightInfo.press[FlightInfo.d_pos] = LPS22HB_Pressure(&lps22hb);
	  FlightInfo.pressraw[FlightInfo.d_pos] = LPS22HB_PressureRaw(&lps22hb);

	  sprintf(wbuff, "%d,%d,%lu,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g",
			  (int)FlightInfo.status,
			  FlightInfo.d_pos,
			  FlightInfo.time[FlightInfo.d_pos],
			  LPS22HB_Pressure(&lps22hb),
			  LPS22HB_Temperature(&lps22hb),
			  MPU9250_calcAccel(&mpu9250,mpu9250.ax),
			  MPU9250_calcAccel(&mpu9250,mpu9250.ay),
			  MPU9250_calcAccel(&mpu9250,mpu9250.az),
			  MPU9250_calcGyro(&mpu9250,mpu9250.gx),
			  MPU9250_calcGyro(&mpu9250,mpu9250.gy),
			  MPU9250_calcGyro(&mpu9250,mpu9250.gz),
			  MPU9250_calcMag(&mpu9250,mpu9250.mx),
			  MPU9250_calcMag(&mpu9250,mpu9250.my),
			  MPU9250_calcMag(&mpu9250,mpu9250.mz)
			  );

	  f_puts((TCHAR *)wbuff, &file);

	  f_close(&file);
	  puts(wbuff);

	  //Estimate_Status(&FlightInfo);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	  tmeasure_b = TIM2->CNT;

#if UART_DEBUG
	  //xprintf("[T:%lu] PressRAW = %lu, TempRAW = %d", time, LPS22HB_PressureRaw(&lps22hb), LPS22HB_TemperatureRaw(&lps22hb));

	  sprintf(ubuff, " %d[t:%lu Pt:%lu] P:%f T:%f A:%f\n",
			  FlightInfo.d_pos,
			  FlightInfo.time[FlightInfo.d_pos],
			  tmeasure_b-tmeasure_a,
			  FlightInfo.press[FlightInfo.d_pos],
			  LPS22HB_Temperature(&lps22hb),
			  FlightInfo.accel[FlightInfo.d_pos]
			  );
	 xputs(ubuff);
#endif
	  FlightInfo.d_num++;
	  FlightInfo.d_pos++;

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter
     *
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 35999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OnePulse_Init(&htim16, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 1999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PB3   ------> USART2_TX
     PB4   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BUZZ_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin 
                           PB7 */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FPIN_Pin */
  GPIO_InitStruct.Pin = FPIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FPIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZ_Pin TRIG_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/**
 * 	System initialization
 */
void Init_LoggerSystem(void)
{
	LPS22HB_Result_t lpsres;
	int mpures;



	/**
	 * 	TImer Start
	 */
	HAL_TIM_Base_Start(&htim2);

	Config_OnePulseDelayTime(TRIG_TIME);


	/**
	 *	File directory configuration
	 */
	Config_Logdir();

	FlightInfo.d_pos = 0;
	FlightInfo.d_num = 0;

	/**
	 * LPS22HB initialization
	 */
	LPS22HB_GetHandle(&lps22hb, &hi2c1);
	LPS22HB_SetAddress(&lps22hb, LPS22HB_Address_L);

	lpsres = LPS22HB_Init(&lps22hb, LPS22HB_ODR_75HZ);
#if UART_DEBUG
	xprintf("[lps22hb] status %d\n",  (int)lpsres );
#endif
	lpsres = LPS22HB_SetLPF(&lps22hb, LPS22HB_LPF_BW9);
#if UART_DEBUG
	xprintf("[lps22hb] set lpf bw9 (stat:%d)\n",  (int)lpsres );
#endif


	MPU9250_GetHandle(&mpu9250, &hi2c1);
	MPU9250_SetAddress(&mpu9250, MPU9250_Address_H);

	mpures = MPU9250_begin(&mpu9250);
#if UART_DEBUG
	xprintf("[mpu9250] status 0x%x\n",  mpures );
#endif
	MPU9250_setSensors(&mpu9250, INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS); // Enable all sensors
	MPU9250_setGyroFSR(&mpu9250, 2000); // Set gyro to 2000 dps
	MPU9250_setAccelFSR(&mpu9250, 16); // Set accel to +/-16g
	  // Note: the MPU-9250's magnetometer FSR is set at
	  // +/- 4912 uT (micro-tesla's)
	MPU9250_setLPF(&mpu9250, 98); // Set LPF corner frequency to 5Hz
	MPU9250_setSampleRate(&mpu9250, 100); // Set sample rate to 10Hz
	MPU9250_setCompassSampleRate(&mpu9250, 100); // Set mag rate to 100Hz
}

void Config_OnePulseDelayTime(float delay)
{
	TIM_OC_InitTypeDef sConfigOC_TIM16;
	uint16_t timeout;

	timeout = delay * SystemCoreClock / (float)(htim16.Init.Prescaler + 1) ;

	htim16.Init.Period = timeout;

	HAL_TIM_Base_Init(&htim16);

	sConfigOC_TIM16.OCMode = TIM_OCMODE_ACTIVE;
	sConfigOC_TIM16.Pulse = timeout;
	sConfigOC_TIM16.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC_TIM16.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC_TIM16.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC_TIM16.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC_TIM16.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC_TIM16,TIM_CHANNEL_1);

	HAL_TIM_OnePulse_Start_IT(&htim16, TIM_CHANNEL_1);
}

/**
 *  Log file directory configuration
 */
void Config_Logdir(void)
{
	int bootcount;
	char logdir[8];

	bootcount = Check_Bootcount();
	if ( bootcount < 0 ) {
		xprintf("bootcount check failure!\n");

		sprintf(logdir, "LOG");
		xprintf("log directory was set to \"LOG\" \n");
	} else {
		xprintf("bootcount is %d\n", bootcount);

		sprintf(logdir, "LOG_%03d", bootcount);
		xprintf("log directory was set to \"LOG_%03d\"\n", bootcount);
	}

	f_mkdir(logdir);

	sprintf(dir_syslog, "LOG_%03d/SYSLOG.TXT", bootcount);
	sprintf(dir_datlog, "LOG_%03d/DATLOG.CSV", bootcount);
}

/**
 * 	Check system boot count from config file in sdcard
 */
int Check_Bootcount(void)
{
    FIL filer;
    FIL filew;
    FRESULT res;

    char buff[3] = {};
    int bootcount = 0;


    res = f_open(&filer, "CONFIG/BOOTCNT.TXT", FA_READ );
    if ( FR_OK != res ) {
        res = f_mkdir("CONFIG");
        if ( FR_OK == res ) {
        	res = f_open(&filew, "CONFIG/BOOTCNT.TXT", FA_CREATE_ALWAYS | FA_WRITE );
        	if ( FR_OK != res ) {	//	failed to make file
        		xprintf("filesystem error!\n");

            	return -1;

        	} else {	//	succeed to make file
        		f_printf(&filew, "%d", bootcount);  //  bootcount = 0
        	}
        }
    } else {	//	succeed to open file
        f_gets((TCHAR *)buff, 4	, &filer);
        bootcount = atoi(buff);
        bootcount++;

        f_close(&filer);

        res = f_open(&filew, "CONFIG/BOOTCNT.TXT", FA_CREATE_ALWAYS | FA_WRITE );
    }

    f_printf(&filew, "%d", bootcount);

    f_close(&filew);

    return bootcount;
}


/**
 * 	Estimate rocket status from sensor data calculation
 */
void Estimate_Status(Flight_Info_t* finfo)
{
	FIL fsys;
    char outbuff[256];

    //  Get slope of pressure from latest 0.2[s] data
    //  (1 / 75[Hz]) * 15 = 0.2[s]
    //finfo->slope[finfo->d_pos] = Calc_LeastSquare_Int(finfo->time, finfo->pressraw, finfo->d_pos, 15);

    //  status switcher
    switch(finfo->status) {

    	//	it is status when the program is booted
        case Rocket_NoStatus: {
            //  count up when slope of pressure is less than }}0.0000005
            if( finfo->slope[finfo->d_pos] <= 0.002048f && finfo->slope[finfo->d_pos] >= -0.002048f )
                finfo->counter++;
            //	or else counter will reset
            else
                finfo->counter = 0;

            // estimate status that static when counter exeeds 60
            if( finfo->counter >= 60 ) {
                //  record system information log
                open_append(&fsys, dir_syslog);
                sprintf(outbuff, "%lu STAT->ONGR %e\n", finfo->time[finfo->d_pos], finfo->slope[finfo->d_pos]);
                f_puts(outbuff, &fsys);
#if UART_DEBUG
                uart_puts(outbuff);
#endif
                f_close(&fsys);

                //  switch status
                finfo->status = Rocket_OnGround;

                finfo->counter = 0;
            }
            break;
        }

        //  rocket is in a state of rest
        case Rocket_OnGround: {
            //
            if( finfo->slope[finfo->d_pos] <= -0.004096 || finfo->accel[finfo->d_pos] > 3.0 ) {
                finfo->counter++;
            } else {
                finfo->counter = 0;
            }

            //
            if( finfo->counter >= 10 ) {
                //	Activate release timer

            	//  record log
            	open_append(&fsys, dir_syslog);
                sprintf(outbuff, "%lu STAT->LNCH %e\n", finfo->time[finfo->d_pos], finfo->slope[finfo->d_pos]);
                f_puts(outbuff, &fsys);
#if UART_DEBUG
                uart_puts(outbuff);
#endif
                f_close(&fsys);

                //  change status
                finfo->status = Rocket_Launched;

                finfo->counter = 0;
            }
            break;
        }

        //  rocket is under acceleration
        case Rocket_Launched: {
            //
            if( calc_leastsquare_floatin(finfo->time, finfo->slope, finfo->d_pos, 15) >= 0.0 || finfo->accel[finfo->d_pos] <= 0.8f ) {

                //  record log
                open_append(&fsys, dir_syslog);
                sprintf(outbuff, "%lu STAT->COFF\n", TIM2->CNT);
                f_puts(outbuff, &fsys);
#if UART_DEBUG
                uart_puts(outbuff);
#endif
				f_close(&fsys);

                //  change status
                finfo->status = Rocket_Cutoff;
            }
            break;
        }

        //  rocket is free falling
        case Rocket_Cutoff: {
        //case Rocket_Launched: {
            if( finfo->slope[finfo->d_pos] > 0.00004096f || finfo->accel[finfo->d_pos] < 0.1f ) {
                //  record log
            	open_append(&fsys, dir_syslog);
                sprintf(outbuff, "%lu STAT->FALL %e\n", finfo->time[finfo->d_pos], finfo->slope[finfo->d_pos]);
                f_puts(outbuff, &fsys);
#if UART_DEBUG
                uart_puts(outbuff);
#endif
                f_close(&fsys);

                //  change status
                finfo->status = Rocket_Falling;

                //open();
            }
            break;
        }

        //
        case Rocket_Falling: {
            if( finfo->slope[finfo->d_pos] <= 0.5f && finfo->slope[finfo->d_pos] >= 0.0f ) {
                //  record log
            	open_append(&fsys, dir_syslog);
                sprintf(outbuff, "%lu STAT->LAND %e\n", finfo->time[finfo->d_pos], finfo->slope[finfo->d_pos]);
                f_puts(outbuff, &fsys);
#if UART_DEBUG
                uart_puts(outbuff);
#endif
                f_close(&fsys);

                // change status
                finfo->status = Rocket_Landed;
            }
            break;
        }

        case Rocket_Landed: {
        	break;
        }
    };
}


/**
 *  Trigger pin interrupt
 *
 *  It will works when positive edge is detected in GPIO pin PC_7
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if( GPIO_Pin == GPIO_PIN_7 ) {
		if( FlightInfo.status <= Rocket_OnGround ) {
			// 	Activate Release Timer
			HAL_TIM_Base_Start_IT(&htim16);

			//	Set the rocket status
			FlightInfo.status = Rocket_Launched;
			FlightInfo.counter = 0;

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

#if UART_DEBUG
			xprintf("Trigger pin was separated!\n");
			xprintf("Release timer count down start\n");
#endif
		}
	}
}

/**
 * 	One shot timer interrupt (TIM16)
 *
 * 	It works when few seconds after EXTI
 *
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint32_t time = TIM16->CNT;
	if( htim->Instance == TIM16) {
		if( time != 0) {
			Trigger_ReleaseSystem();
			xprintf("Initiate release system\n");
		}
	}
}

/**
 * 	Send trigger signal for initiate release system
 */
void Trigger_ReleaseSystem(void)
{
#if RELEASE_MODE == 0
	//	output trigger signal
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
#elif RELEASE_MODE == 1	//	servo
	//	output trigger signal
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
#endif

	//	indicate that trigger signal is active on LED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
#if UART_DEBUG
	xprintf("Release system triggered\n");
#endif
}

/**
 * FatFs file open with append mode
 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
