/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : nlms_filter.c
  * @brief          : Normalized Least Mean Square Filter
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include <stdint.h>

void vec_splitter(uint32_t *input, uint16_t *left, uint16_t *right, int start, int length)
{
	for (int i=start; i < start + length; i++)
	{
		left[i] = (uint16_t) ((input[i] & 0xFFFF0000)>>16);
		right[i] = (uint16_t) (input[i] & 0x0000FFFF);
	}
}

void sidelobe_math(int *arr1, int *arr2, int *addition, int *subtraction, int arr_len)
{
	for (int i=0;i<arr_len;i++)
	{
		addition[i] = arr1[i] + arr2[i]; 	// Add each element to get Voice and Noise
		subtraction[i] = arr2[i] - arr1[i]; // Subtract each element to get sidelobe noise
	}

	return;
}

void audio_splitter(uint32_t *adc_buf, uint16_t *sum, uint16_t *diff, int w_pointer, int offset_w_pointer, uint32_t ADC_BUF_LENGTH)
{
	uint16_t adc1_val, adc2_val;
	for (int i = offset_w_pointer; i<(offset_w_pointer + ADC_BUF_LENGTH); i++)
	{
  	  adc1_val = (uint16_t) ((adc_buf[i] & 0xFFFF0000)>>16);
  	  adc2_val = (uint16_t) (adc_buf[i] & 0xFFFF);
  	  sum[w_pointer] = adc1_val + adc2_val;
  	  diff[w_pointer] = adc1_val - adc2_val;
  	  w_pointer++;
	}
}

