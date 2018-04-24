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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */

#include "Misc/Common.h"
#include "Sensors/IMU/imu.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId defaultTaskHandle;
osThreadId IMU_ifaceHandle;
osThreadId xBeeTelemetryHandle;
osThreadId fetchFBarometerHandle;
osThreadId centralizeDataHandle;
osThreadId xBee_RCHandle;
osMessageQId xBeeQueueHandle;
osSemaphoreId IMU_IntSemHandle;
osSemaphoreId xBeeTxBufferSemHandle;
osSemaphoreId pressureSensorI2cSemHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

IMU_data IMU_buffer[CIRC_BUFFER_SIZE] CCMRAM;
IMU_data BARO_buffer[CIRC_BUFFER_SIZE] CCMRAM;

UART_HandleTypeDef* xBee_huart;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config (void);
static void MX_GPIO_Init (void);
static void MX_DMA_Init (void);
static void MX_SPI1_Init (void);
static void MX_USART1_UART_Init (void);
static void MX_TIM7_Init (void);
static void MX_USART3_UART_Init (void);
static void MX_I2C2_Init (void);
static void MX_SDIO_SD_Init (void);
static void MX_UART4_Init (void);
void StartDefaultTask (void const * argument);
extern void TK_IMU (void const * argument);
extern void TK_xBeeTelemetry (void const * argument);
extern void TK_fetchBarometer (void const * argument);
extern void TK_data (void const * argument);
extern void TK_xBee_receive (void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void initPeripherals();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main (void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init ();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config ();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init ();
  MX_DMA_Init ();
  MX_SPI1_Init ();
  MX_USART1_UART_Init ();
  MX_TIM7_Init ();
  MX_USART3_UART_Init ();
  MX_I2C2_Init ();
  MX_SDIO_SD_Init ();
  MX_UART4_Init ();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT (&htim7);
  HAL_GPIO_WritePin (GPS_ENn_GPIO_Port, GPS_ENn_Pin, GPIO_PIN_RESET);

  initBarometer ();
  initPeripherals ();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of IMU_IntSem */
  osSemaphoreDef(IMU_IntSem);
  IMU_IntSemHandle = osSemaphoreCreate (osSemaphore(IMU_IntSem), 1);

  /* definition and creation of xBeeTxBufferSem */
  osSemaphoreDef(xBeeTxBufferSem);
  xBeeTxBufferSemHandle = osSemaphoreCreate (osSemaphore(xBeeTxBufferSem), 1);

  /* definition and creation of pressureSensorI2cSem */
  osSemaphoreDef(pressureSensorI2cSem);
  pressureSensorI2cSemHandle = osSemaphoreCreate (osSemaphore(pressureSensorI2cSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate (osThread(defaultTask), NULL);

  /* definition and creation of IMU_iface */
  osThreadDef(IMU_iface, TK_IMU, osPriorityAboveNormal, 0, 128);
  IMU_ifaceHandle = osThreadCreate (osThread(IMU_iface), NULL);

  /* definition and creation of xBeeTelemetry */
  osThreadDef(xBeeTelemetry, TK_xBeeTelemetry, osPriorityBelowNormal, 0, 128);
  xBeeTelemetryHandle = osThreadCreate (osThread(xBeeTelemetry), NULL);

  /* definition and creation of fetchFBarometer */
  osThreadDef(fetchFBarometer, TK_fetchBarometer, osPriorityAboveNormal, 0, 128);
  fetchFBarometerHandle = osThreadCreate (osThread(fetchFBarometer), NULL);

  /* definition and creation of centralizeData */
  osThreadDef(centralizeData, TK_data, osPriorityNormal, 0, 128);
  centralizeDataHandle = osThreadCreate (osThread(centralizeData), NULL);

  /* definition and creation of xBee_RC */
  osThreadDef(xBee_RC, TK_xBee_receive, osPriorityNormal, 0, 128);
  xBee_RCHandle = osThreadCreate (osThread(xBee_RC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of xBeeQueue */
  osMessageQDef(xBeeQueue, 16, Telemetry_Message);
  xBeeQueueHandle = osMessageCreate (osMessageQ(xBeeQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Start scheduler */
  osKernelStart ();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

    }
  /* USER CODE END 3 */

}

/* USER CODE BEGIN 10 */

void initPeripherals ()
{
  xBee_huart = &huart3;
  HAL_UART_Init(xBee_huart);
}
/* USER CODE END 10 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config (void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE()
  ;

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
    {
      _Error_Handler (__FILE__, __LINE__);
    }

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
      _Error_Handler (__FILE__, __LINE__);
    }

  /**Configure the Systick interrupt time
   */
  HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq () / 1000);

  /**Configure the Systick
   */
  HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (SysTick_IRQn, 15, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init (void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init (&hi2c2) != HAL_OK)
    {
      _Error_Handler (__FILE__, __LINE__);
    }

}

/* SDIO init function */
static void MX_SDIO_SD_Init (void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

}

/* SPI1 init function */
static void MX_SPI1_Init (void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init (&hspi1) != HAL_OK)
    {
      _Error_Handler (__FILE__, __LINE__);
    }

}

/* TIM7 init function */
static void MX_TIM7_Init (void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 168;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 50;
  if (HAL_TIM_Base_Init (&htim7) != HAL_OK)
    {
      _Error_Handler (__FILE__, __LINE__);
    }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&htim7, &sMasterConfig) != HAL_OK)
    {
      _Error_Handler (__FILE__, __LINE__);
    }

}

/* UART4 init function */
static void MX_UART4_Init (void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init (&huart4) != HAL_OK)
    {
      _Error_Handler (__FILE__, __LINE__);
    }

}

/* USART1 init function */
static void MX_USART1_UART_Init (void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init (&huart1) != HAL_OK)
    {
      _Error_Handler (__FILE__, __LINE__);
    }

}

/* USART3 init function */
static void MX_USART3_UART_Init (void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init (&huart3) != HAL_OK)
    {
      _Error_Handler (__FILE__, __LINE__);
    }

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init (void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE()
  ;
  __HAL_RCC_DMA2_CLK_ENABLE()
  ;

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ (DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ (DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ (DMA1_Stream3_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ (DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ (DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ (DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ (DMA2_Stream6_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init (void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE()
  ;
  __HAL_RCC_GPIOC_CLK_ENABLE()
  ;
  __HAL_RCC_GPIOB_CLK_ENABLE()
  ;
  __HAL_RCC_GPIOE_CLK_ENABLE()
  ;
  __HAL_RCC_GPIOD_CLK_ENABLE()
  ;

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPS_ENn_GPIO_Port, GPS_ENn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPIOB, ABS_P_SENS_ENn_Pin | DIF_P_SENS_ENn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPIOE, SD_ENn_Pin | GPS_RESETn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPS_SWITCH_GPIO_Port, GPS_SWITCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (IMU_ENn_GPIO_Port, IMU_ENn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPIOA, AUX_IO_10_Pin | AUX_IO_12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPS_ENn_Pin */
  GPIO_InitStruct.Pin = GPS_ENn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPS_ENn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ABS_P_SENS_ENn_Pin DIF_P_SENS_ENn_Pin */
  GPIO_InitStruct.Pin = ABS_P_SENS_ENn_Pin | DIF_P_SENS_ENn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_ENn_Pin GPS_RESETn_Pin GPS_SWITCH_Pin */
  GPIO_InitStruct.Pin = SD_ENn_Pin | GPS_RESETn_Pin | GPS_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_ENn_Pin */
  GPIO_InitStruct.Pin = IMU_ENn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (IMU_ENn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AUX_IO_10_Pin AUX_IO_12_Pin */
  GPIO_InitStruct.Pin = AUX_IO_10_Pin | AUX_IO_12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT0_Pin */
  GPIO_InitStruct.Pin = IMU_INT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (IMU_INT0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority (EXTI9_5_IRQn, 12, 0);
  HAL_NVIC_EnableIRQ (EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/*
 * IMU interrupt handler
 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  getIMUdataDMA ();
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask (void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init ();

  /* USER CODE BEGIN 5 */

  uint32_t wbytes;
  /* File write counts */
  uint8_t wtext[] = "text to write logical disk";

  /* File write buffer */
  /*
   if (FATFS_LinkDriver (&SD_Driver, mynewdiskPath) == 0)
   {
   if (f_mount (&mynewdiskFatFs, (TCHAR const*) mynewdiskPath, 0) == FR_OK)
   {
   if (f_open (&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
   {
   if (f_write (&MyFile, wtext, sizeof(wtext), (void *) &wbytes) == FR_OK)
   ;
   {
   f_close (&MyFile);
   }
   }
   }
   }
   FATFS_UnLinkDriver (mynewdiskPath);
   */

  HAL_GPIO_WritePin (GPS_ENn_GPIO_Port, GPS_ENn_Pin, GPIO_PIN_RESET);
  osDelay (500);
  HAL_GPIO_WritePin (GPS_RESETn_GPIO_Port, GPS_RESETn_Pin, GPIO_PIN_SET);

  /* Infinite loop */
  for (;;)
    {
      /*
       IMU_data* current_IMU_data = &IMU_buffer[currentImuSeqNumber % CIRC_BUFFER_SIZE];

       void* ptr = pvPortMalloc (sizeof(IMU_data));
       IMU_data* d = ptr;
       *d = *current_IMU_data;
       Telemetry_Message m =
       { .ptr = ptr, .size = sizeof(IMU_data) };
       osMessagePut (xBeeQueueHandle, &m, 50);
       */
      osDelay (10);
    }
  /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
    {
      HAL_IncTick ();
    }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler (char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
