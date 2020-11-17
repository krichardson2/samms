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
#include "bma456.h"

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
I2C_HandleTypeDef hi2c1;
#define bma456address 0x30 // SLAVE ADDRESS of BMA456
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
// this tutorial uses slave receive with the same board https://visualgdb.com/tutorials/arm/stm32/i2c/
void read_BMA456_bus (uint8_t addr, uint8_t *data, uint32_t len, void* intf_ptr) {
	if (len == 1) {
		HAL_I2C_Master_Transmit(&hi2c1, bma456address, addr, len, 10);
		HAL_I2C_Master_Receive(&hi2c1, bma456address, data, len, 10); // don't use &data because data is a pointer
	}
	else { // specifically for reading the x, y, z sense data
		uint8_t sense_addr = addr;
		uint8_t sense_data[6];
		uint8_t sense = 0;
		for (int i = 0; i < 6; i++) { // walk through all 6 sense registers and store in sense_data
			HAL_I2C_Mem_Read(&hi2c1, bma456address, sense_addr, 1, &sense_data[i], 1, 10);
			sense_addr = sense_addr + 0x01;
			data[i] = sense_data[i];
		}
	}
}

void write_BMA456_bus (uint8_t addr, uint8_t *data, uint32_t len, void* intf_ptr) {
	uint8_t addr_data[len+1]; // addr_data[0] = reg addr, addr_data[1+] = data to be written
	addr_data[0] = addr;
	for (int i = 1; i < len + 1; i++) { // concatenate addr and data
		addr_data[i] = data[i-1];
	}
	HAL_I2C_Master_Transmit(&hi2c1, bma456address, addr_data, len + 1, 100);
}

void delay_BMA456_us (uint32_t delay_us, void* intf_ptr) {
	float delay_ms = (float) delay_us / 1000; // convert from us to ms
	HAL_Delay(delay_ms);
}

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2
   * https://www.youtube.com/watch?v=A1CIOUZdeR4 */

//  bma4_read_fptr_t read_bus = 0x42;
//  bma4_write_fptr_t write_bus = 0x4c;
//  bma4_delay_us_fptr_t delay_us = 500000; // 0.5 s
  uint8_t i2cdata[2] = {100, 100};
  uint8_t x = -10;
  // look for slave address
//  for (uint8_t i = 0; i < 255; i++) {
//	  if (HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 10) == HAL_OK) { // IT WORKS OMG. 0x30 = [0011000][0] AUX_DEV_ID in datasheet
//		  x = 1;
//	  	  break;
//	  }
//  }
  x = 0;
  if (HAL_I2C_IsDeviceReady(&hi2c1, 0x30, 1, 10) == HAL_OK) // IT WORKS OMG. 0x30 = [0011000][0] AUX_DEV_ID in datasheet
	  x = 1;

  // reading i2c
  // tutorial: https://www.youtube.com/watch?v=1COFk1M2tak
  // step 1 transmit register address
  //   HAL_I2C_Master_Transmit(&hi2c, slave_address, register_to_read_from, byte_length, timeout_ms);
  HAL_I2C_Master_Transmit(&hi2c1, bma456address, 0x00, 1, 10);
  // step 2 send read request
  //   HAL_I2C_Master_Receive(&hi2c1, slave_address, &buffer_to_store_data, byte_length, timeout_ms);
  HAL_I2C_Master_Receive(&hi2c1, bma456address, &i2cdata[1], 1, 10);

// --------- INITALIZE BMA456 ----------------
  // based on https://community.bosch-sensortec.com/t5/Knowledge-base/BMA4xy-accelerometer-series-design-guide/ta-p/5837
  uint16_t rslt = BMA4_OK;
  struct bma4_dev dev;
  /* Modify the parameters */
  //fptr is function ptr
  //dev.chip_id	      = 0x16; // i2c slave address (chip id)
  dev.intf      	  = BMA4_I2C_INTF; // we are using i2c
  dev.variant 		  = BMA45X_VARIANT; // bma456
  dev.bus_read        = &read_BMA456_bus; // 0x42 = read addr
  dev.bus_write       = &write_BMA456_bus; // 0x4c = write addr
  dev.delay_us        = &delay_BMA456_us; // 500000 = 0.5s;
  // the 3 above variables cause an error because they're pointers set to variables (dangling pointer?)
  dev.read_write_len  = 8;
  dev.resolution      = 12;
  dev.feature_len     = BMA456_FEATURE_SIZE;
  rslt = bma456_init(&dev);
  if (rslt == BMA4_OK)
  { // SUCCESS! chip id should be 0x16 = 22
      x = 2;
  }
  else { // initialization failure
	  x = 3;
  }
  // --------- INITALIZE BMA456 OVER ----------------

  /* Initialize the device instance as per the initialization example */

  /* Enable the accelerometer */
  bma4_set_accel_enable(1, &dev);

  /* Declare an accelerometer configuration structure */
  struct bma4_accel_config accel_conf;

  /* Assign the desired settings */
  accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
  accel_conf.range = BMA4_ACCEL_RANGE_2G;
  accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
  accel_conf.perf_mode = BMA4_CONTINUOUS_MODE;

  /* Set the configuration */
  rslt |= bma4_set_accel_config(&accel_conf, &dev);
  //HAL_I2C_Master_Receive(&hi2c1, 0x40, &i2cdata[1], 1, 10); // if successful, '0x28' should be i2cdata[1]


  struct bma4_accel sens_data;
  // above does not work because certain items in dev are NULL. Use the BMA456 github repo to choose values for the items
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // test if sensor is working by triggering the self-test
  uint16_t commrslt;
  uint8_t	testrslt;

  commrslt = bma4_perform_accel_selftest(&testrslt, &dev);
  // if testrslt = 0, success!
  while (1)
  {
      /* Read the sensor data into the sensor data instance */
      rslt |= bma4_read_accel_xyz(&sens_data, &dev);

      /* Exit the program in case of a failure */
      if (rslt != BMA4_OK)
          return rslt;

      /* Use the data */
      //printf("X: %d, Y: %d, Z: %d\n", sens_data.x, sens_data.y, sens_data.z);
      HAL_Delay(10);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
