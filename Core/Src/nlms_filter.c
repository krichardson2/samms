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

#include "arm_math.h"



void sidelobe_math(int *arr1, int *arr2, int *addition, int *subtraction, int arr_len)
{
	for (int i=0;i<arr_len;i++)
	{
		addition[i] = arr1[i] + arr2[i]; 	// Add each element to get Voice and Noise
		subtraction[i] = arr2[i] - arr1[i]; // Subtract each element to get sidelobe noise
	}

	return;
}

