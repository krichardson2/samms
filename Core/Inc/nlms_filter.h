/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : nlms_filter.h
  * @brief          : Header for nlms_filter.c file.
  *                   This file contains the common defines of the application.
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

#include <stdint.h>
/*
 * Function to split 32-bit vector into two 16-bit vectors
 */
void vec_splitter(uint32_t *input, uint16_t *left, uint16_t *right, int start, int length);

void sidelobe_math(int *arr1, int *arr2, int *addition, int *subtraction, int arr_len);

void audio_splitter(uint32_t *adc_buf, float *sum, float *diff, int w_pointer, int offset_w_pointer, uint32_t ADC_BUF_LENGTH);
