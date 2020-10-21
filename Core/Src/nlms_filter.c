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

void audio_splitter(uint32_t *adc_buf, float *sum, float *diff, int w_pointer, int offset_w_pointer, uint32_t ADC_BUF_LENGTH)
{
	uint16_t adc1_val, adc2_val;
	for (int i = offset_w_pointer; i<(offset_w_pointer + ADC_BUF_LENGTH); i++)
	{

  	  adc1_val = ((adc_buf[i] & 0xFFFF0000)>>16);
  	  adc2_val = (adc_buf[i] & 0xFFFF);

  	  // Going to scale values to fit within floating point value range
  	  // -3.4*1038 to 3.4*1038
  	  //   -3529.2 to 3529.2

  	  /* Subtract mean voltage of 1.65V and shift by 1 bit
			1.65V -> 12Bit -> 2048
  	   		>> 1 bit = -3 dB
  	   	    cast to float
  	  */

  	  adc1_val = (float) ((adc1_val - 2048)>>1);
  	  adc2_val = (float) ((adc2_val - 2048)>>1);

  	  sum[w_pointer] = (float) adc1_val + adc2_val;
  	  diff[w_pointer] = (float) adc1_val - adc2_val;
  	  w_pointer++;
	}
}


