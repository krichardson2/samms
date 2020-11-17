/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "dataout.h"
#include <string.h>
#include <stdio.h>
#include "arm_math.h"
#include "nlms_filter.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Timer Parameters
#define TIMER_FREQUENCY			  ((uint32_t) 22000) /* Timer frequency (unit: Hz). With a timer 16-bits and time base freq min 1Hz, range is (1Hz, 32kHz) */
#define TIMER_FREQUENCY_RANGE_MIN ((uint32_t) 1)   /* Timer minimum frequency (unit: Hz), used to calculate frequency range. With a timer 16 bits, maximum frequency will be 32000 times this value.*/
#define TIMER_PRESCALER_MAX_VALUE (0xFFFF-1)	   /* Timer prescaler maximum value (0xFFFF for a timer 16-bits) */

#define ADC_BUF_LENGTH 2048 // 2048 - at 20kHz roughly 1/10th of a second
#define ADC_BUF_HALF 1024   // 1024 - 1/20th of a second
#define ADC_BUF_QUARTER 512 // 512 - downsampled output of filtering
#define ADC_BUF_EIGHTH 256
#define FILTER_TAP_NUM 1024 // Have filter number he same length as the data being processed
#define DECIMATE_TAP_NUM 21

//Definitions for changing Code
#define AUDIO_OUTPUT_ENABLE 1// 1 to turn on I2S output, #undef AUDIO_OUTPUT_ENABLE to turn off
#undef NLMS_FILTER_ENABLE // 1 to turn on nLMS filter, #undef NLMS_FILTER_ENABLE to turn off
#define DECIMATION_ENABLE  1// 1 to turn on Decimator after nLMS filter

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_spi3_tx;
TIM_HandleTypeDef htim2;
SemaphoreHandle_t SDCard_Mutex;
TickType_t MAX_LOCK_DELAY = 10;
FATFS FatFs; // object
FIL fil; // file object
FRESULT fres;
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
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

/* Definitions for initialize */
osThreadId_t initializeHandle;
const osThreadAttr_t initialize_attributes = {
  .name = "initialize",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for accelerometer */
osThreadId_t accelerometerHandle;
const osThreadAttr_t accelerometer_attributes = {
  .name = "accelerometer",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for adcDMA */
osThreadId_t adcDMAHandle;
const osThreadAttr_t adcDMA_attributes = {
  .name = "adcDMA",
  .priority = (osPriority_t) osPriorityBelowNormal1,
  .stack_size = 128 * 4
};
/* Definitions for filtering */
osThreadId_t filteringHandle;
const osThreadAttr_t filtering_attributes = {
  .name = "filtering",
  .priority = (osPriority_t) osPriorityBelowNormal2,
  .stack_size = 128 * 4
};
/* Definitions for feedback */
osThreadId_t feedbackHandle;
const osThreadAttr_t feedback_attributes = {
  .name = "feedback",
  .priority = (osPriority_t) osPriorityBelowNormal3,
  .stack_size = 128 * 4
};
/* Definitions for dataout */
osThreadId_t dataoutHandle;
const osThreadAttr_t dataout_attributes = {
  .name = "dataout",
  .priority = (osPriority_t) osPriorityBelowNormal5,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
RTC_TimeTypeDef currTime = {0};
RTC_DateTypeDef currDate = {0};

uint8_t buffer[24]; // holds date time and SPL
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);

/* USER CODE BEGIN PFP */
void getTime(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t adc_buf [ADC_BUF_LENGTH];

float mic_sum [ADC_BUF_HALF];
float mic_diff [ADC_BUF_HALF];

#ifdef AUDIO_OUTPUT_ENABLE
uint16_t L_mic [ADC_BUF_HALF];
uint16_t R_mic [ADC_BUF_HALF];
uint16_t txBuf [2*ADC_BUF_LENGTH];
#endif

#ifdef NLMS_FILTER_ENABLE
float nlmsCoeffs [FILTER_TAP_NUM];
float nlmsState [FILTER_TAP_NUM + ADC_BUF_HALF - 1]; // Output of nLMS filter is len(x) + len(h) - 1
float mu = 0.05;
float nlmsOutput [ADC_BUF_HALF];
float nlmsError [ADC_BUF_HALF];
arm_lms_norm_instance_f32 nlms_struct;
#endif

#ifdef DECIMATION_ENABLE
/*
 Filter designed in http://t-filter.engineerjs.com/

 Low-pass designed for decimator filter:

 Sampling Rate = 20000 Hz
from	to		gain		ripple/att.		act. rpl
0 Hz 	5000 Hz 	1 		   5 dB 	  	  4.05 dB
6000 Hz	10000 Hz 	0   	 -40 dB 		-40.25 dB
*/
arm_fir_decimate_instance_f32 decimate_struct;
static float decimateCoeffs [DECIMATE_TAP_NUM] = {
		0.0285798399416968,
		0.07328836181028257,
		0.04512928732568178,
		-0.03422632401030227,
		-0.03472426238662957,
		0.053430907613764095,
		0.032914528649623485,
		-0.0988081824627219,
		-0.03413542207884341,
		0.3160339484471911,
		0.5341936566511764,
		0.3160339484471911,
		-0.03413542207884341,
		-0.0988081824627219,
		0.032914528649623485,
		0.053430907613764095,
		-0.03472426238662957,
		-0.03422632401030227,
		0.04512928732568178,
		0.07328836181028257,
		0.0285798399416968};

float decimateState [ADC_BUF_HALF + DECIMATE_TAP_NUM - 1];
uint8_t mFactor = 2; // Downsample from 20kHz to 10kHz, giving us 5kHz bandwidth still
float downsampleOutput [ADC_BUF_QUARTER];
#endif

float rmsState [ADC_BUF_EIGHTH];

int callback_state = 0;
int SPL_FLAG = 0;
int rms_count = 0;
float rms_val; //output of RMS calculation for each sliding block of data
float rms_sum;
double spl = 0; // pointer for result of SPL calculations

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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  mountDrive(&FatFs, &fil, &fres); // mount SD card
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  SDCard_Mutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of initialize */
  initializeHandle = osThreadNew(StartDefaultTask, NULL, &initialize_attributes);

  /* creation of accelerometer */
  accelerometerHandle = osThreadNew(StartTask02, NULL, &accelerometer_attributes);

  /* creation of adcDMA */
  adcDMAHandle = osThreadNew(StartTask03, NULL, &adcDMA_attributes);

  /* creation of filtering */
  filteringHandle = osThreadNew(StartTask04, NULL, &filtering_attributes);

  /* creation of feedback */
  feedbackHandle = osThreadNew(StartTask05, NULL, &feedback_attributes);

  /* creation of dataout */
  dataoutHandle = osThreadNew(StartTask06, NULL, &dataout_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
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
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
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
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
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
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
	  RCC_ClkInitTypeDef clk_init_struct = {0};       /* Temporary variable to retrieve RCC clock configuration */
	  uint32_t latency;                               /* Temporary variable to retrieve Flash Latency */

	  uint32_t timer_clock_frequency = 0;

	  // Retrieve timer clock source frequency //
	  HAL_RCC_GetClockConfig(&clk_init_struct, &latency);
	  /* If APB1 prescaler is different of 1, timers have a factor x2 on their    */
	    /* clock source.                                                            */
	  if (clk_init_struct.APB1CLKDivider == RCC_HCLK_DIV1)
	  {
	    timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
	  }
	  else
	  {
	    timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;
	  }

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;//(timer_prescaler - 1);
  htim2.Init.Period = (timer_clock_frequency / (TIMER_FREQUENCY * 2));//((timer_clock_frequency / (timer_prescaler * TIMER_FREQUENCY)) - 1);
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

  /* USER CODE END TIM2_Init 1 */
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SD_CS_Pin|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : SD_CS_Pin PD12 PD13 PD14
                           PD15 */
  GPIO_InitStruct.Pin = SD_CS_Pin|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Called when first half of buffer is filled
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  //Toggle LEDS
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  //Set flag high
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
  callback_state = 1;
}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  //Toggle LEDS
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
  //Pull flag low
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  callback_state = 2;
}

// get time function for RTC
void getTime()
{
	//must be called together in this order
	HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BCD);

	buffer[0] = (currDate.Date / 16) + 48;
	buffer[1] = (currDate.Date % 16) + 48;
	buffer[2] = '.';
	buffer[3] = (currDate.Month / 16) + 48;
	buffer[4] = (currDate.Month % 16) + 48;
	buffer[5] = '.';
	buffer[6] = '2';
	buffer[7] = '0';
	buffer[8] = (currDate.Year / 16) + 48;
	buffer[9] = (currDate.Year % 16) + 48;
	buffer[10] =',';
	buffer[11] = (currTime.Hours / 16) + 48;
	buffer[12] = (currTime.Hours % 16) + 48;
	buffer[13] = ':';
	buffer[14] = (currTime.Minutes / 16) + 48;
	buffer[15] = (currTime.Minutes % 16) + 48;
	buffer[16] = ':';
	buffer[17] = (currTime.Seconds / 16) + 48;
	buffer[18] = (currTime.Seconds % 16) + 48;
	buffer[19] = ',';
	buffer[20] = '1';
	buffer[21] = '1';
	buffer[22] = '0';
	buffer[23] = '\n';
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the initialize thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	int userdata[2];
	int spl_read = 0;

	//xSemaphoreTake(SDCard_Mutex, MAX_LOCK_DELAY); // lock mutex
	readSDcard(&FatFs, &fil, &fres, userdata);
	unmountDrive(&FatFs, &fil, &fres); // mount SD card
	//xSemaphoreGive(SDCard_Mutex); // unlock mutex
	spl_read = userdata[1];
  for(;;)
  {
	if (spl_read == 50)
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	  osDelay(500);
  }
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the accelerometer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	if (HAL_I2C_IsDeviceReady(&hi2c1, 0x30, 2, 10) == HAL_OK) // IT WORKS OMG. 0x30 = [0011000][0] AUX_DEV_ID in datasheet
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	if (SPL_FLAG != 1) {
		htim2.Instance->CR1 &= ~TIM_CR1_CEN; // pause timer
		htim2.Instance->CNT = 0;
		htim2.Instance->SR = (uint16_t)~TIM_FLAG_UPDATE;
	}
	else if (SPL_FLAG == 1) {
		// TURN TIMER ON
		htim2.Instance->CR1 |= TIM_CR1_CEN;
	}
	SPL_FLAG = 1 - SPL_FLAG;
    osDelay(5000);
  }
  osThreadTerminate(NULL);
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the adcDMA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	  // Calibrate ADC on power-up
	  HAL_ADC_Start(&hadc2);
	  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) adc_buf, (uint32_t) ADC_BUF_LENGTH) != HAL_OK)
	  {
		  Error_Handler();
	  }

	  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  //Initialize nLMS Filter Structure
	  //arm_lms_norm_init_f32(&nlms_struct, FILTER_TAP_NUM, &nlmsCoeffs[0], &nlmsState[0], mu, ADC_BUF_HALF);

	  //Initialize decimation Filter Structure
	  arm_fir_decimate_init_f32(&decimate_struct, (uint16_t) DECIMATE_TAP_NUM, mFactor, &decimateCoeffs[0], &decimateState[0], (uint32_t) ADC_BUF_HALF);

	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */

	  int offset_w_pointer, w_pointer;
	  float armResult; // pointer for result of arm_rms_f32 function call

  /* Infinite loop */
  for(;;)
  {
      if (callback_state != 0)
      {
          // Determine if half or full complete DMA callback
          if (callback_state == 1) // Half Complete callback
          {
              offset_w_pointer = 0; // pull data from first half of adc buffer
              w_pointer = 0;
          }

          else if (callback_state == 2) // Full Complete callback
          {
              offset_w_pointer = ADC_BUF_HALF; // pull data from second half of adc buffer
              w_pointer = 0;
          }

          audio_splitter(&adc_buf[0], &mic_sum[0], &mic_diff[0], w_pointer, offset_w_pointer, ADC_BUF_HALF);


		  #ifdef NLMS_FILTER_ENABLE
          // Add and subtract the data from each microphone to pass into nLMS
          // sum is d_n, diff is x_n
          // audio_splitter(&adc_buf[0], &mic_sum[0], &mic_diff[0], w_pointer, offset_w_pointer, ADC_BUF_HALF);

          // Pass pointers to nLMS Filter
          // Feed into d, x, output, error vectors into nLMS filter
          // arm_lms_norm_f32(&nlms_struct, &mic_sum[0], &mic_diff[0], &nlmsOutput[0], &nlmsError[0], (uint32_t) ADC_BUF_HALF);

		  #endif

		  #ifdef DECIMATION_ENABLE
          // Downsample by a factor of 2, and pass through Lowpass filter
          arm_fir_decimate_f32(&decimate_struct, &mic_sum[0], &downsampleOutput[0], ADC_BUF_HALF);

          /* Calculate RMS value for SPL calculations
                    Calculate across sliding 40ms windows of the speech data. Output from decimation filter is
                    is going to be (ADC_BUF_HALF / mFactor). For this project, (1024 / 2) = 512 samples
                    @ 10kHz the period is 100 microseconds, so lets go for 256 samples

                    pass the following into the RMS calculation function:

                    &downsampleOutput[0] : input data buffer address (data from downsampling)
                    &rmsState[0]		 : buffer for holding previous data when doing sliding block (2 * size of overlap block)
                    armResult			 : a pointer to where the arm_rms_f32 call returns its calculated value
					ADC_BUF_QUARTER		 : the size of the input data buffer
					OverlapBlock		 : half the length of rmsState buffer
		  */

          rms_count++;
          rms_val = calc_rms(&downsampleOutput[0], &rmsState[0], &armResult, ADC_BUF_QUARTER, ADC_BUF_EIGHTH);
          rms_sum += rms_val;
          if (SPL_FLAG)
          {
        	  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
        	  // Calculate SPL
        	  // This branch is simulating the end of data gathering from Accelerometer.
        	  spl = calc_SPL(rms_sum, rms_count);
        	  if (spl > 60)
        		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        	  if (spl < 50)
        		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        	  rms_count = 0;
        	  rms_val = 0;
        	  rms_sum = 0;
        	  SPL_FLAG = 0; // high when timer is OFF

          }

          #endif
          callback_state = 0;
      }
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the filtering thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the feedback thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the dataout thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
