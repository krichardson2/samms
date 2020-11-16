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
#include <math.h>


void vec_splitter(uint32_t *input, uint16_t *left, uint16_t *right, int start, int length)
{
	for (int i=start; i < start + length; i++)
	{
		left[i] = (uint16_t) ((input[i] & 0xFFFF0000)>>16);
		right[i] = (uint16_t) (input[i] & 0x0000FFFF);
	}
}

void audio_splitter(uint32_t *adc_buf, float *sum, float *diff, int w_pointer, int offset_w_pointer, uint32_t buf_half_length)
{
	float adc1_val, adc2_val;
	for (int i = offset_w_pointer; i<(offset_w_pointer + buf_half_length); i++)
	{

  	  adc1_val = ((float)((adc_buf[i] & 0xFFFF0000)>>16))-2260;
  	  adc2_val = ((float)(adc_buf[i] & 0xFFFF))-2260;

  	  // Going to scale values to fit within floating point value range
  	  // -3.4*1038 to 3.4*1038
  	  //   -3529.2 to 3529.2

  	  /* Subtract mean voltage of 1.65V and shift by 1 bit
  	        2.30V offset from short -> 12-Bit -> 2854
			1.65V -> 12Bit -> 2048
  	   		>> 1 bit = -3 dB
  	   	    cast to float
  	  */

  	  sum[w_pointer] = (float)  adc1_val;
  	  diff[w_pointer] = (float) adc2_val;
  	  w_pointer++;
	}
}

/*S calc_rms(float *wave, float* rmsState, uint32_t lengthVector)
 * 	rmsState should be a preallocated buffer of size lengthVector/2
 *	float rmsVal = 0.0;
 *
 *	return (rms >> 10)
*/
float calc_rms(float *wave, float *rmsState, float *armResult, uint32_t lengthVector, uint32_t overlapBlock)
{
	float rmsVal = 0.0;
	uint32_t halfOverlap = overlapBlock >> 1;
	int startInd = 0;


	// I need to add comments for clarity
	for (int m=0;m<(lengthVector/halfOverlap);m++)
	{
		startInd = halfOverlap - startInd; // alternates between half-overlap and zero
		int indJump = m*halfOverlap;
		for (int i=0;i<halfOverlap;i++)
		{
			rmsState[(startInd+i)] = wave[(indJump+i)];
		}
		arm_rms_f32(rmsState, overlapBlock, armResult);
		rmsVal += *armResult;
	}

	return (float) (rmsVal / 4);
}

/*
 * Function to calculate the SPL based on RMS voltage of the output speech signal.
 * This output should be scaled from a reference signal at a certain 30cm distance from a speaker
 * with a controllable volume output.
 *
 * Inputs:
 * 	float 	  RMS   	- the total summed rms value of the previous speech signals
 * 	uint32_t  count 	- the number of blocks used in RMS calculations. Used to find average RMS over all blocks
 *  double*   spl_val 	- pointer to where the calculated SPL value is stored in memory
 *
 * Outputs:
 *  None
 */
double calc_SPL(float RMS, uint32_t count)
{
	// SPL=20log(Vin/2)+100.45
	double speechRMS, spl_val;
	speechRMS = (double) ((RMS) / (count * 2));
	spl_val = 1.5866*(20*(log10(fabs(speechRMS))) + 22.17) - 18.556;
	// Linear regression from excel : y = 1.5866x - 18.556

	return spl_val;
}

