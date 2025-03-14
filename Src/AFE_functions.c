/*
 * AFE_functions.c
 *
 *  Created on: Nov 8, 2024
 *      Author: blondier94
 */

#include "AFE_functions.h"
#include <stm32f0xx_hal.h>
#include <math.h>

/* Driver */

inline float __attribute__ ((always_inline, optimize("-O3")))
get_voltage_for_SiPM (float T, float a, float T_0, float U_0, float U_cor)
{
  return a * (T - T_0) + U_0 + U_cor;
}

float __attribute__ ((optimize("-O3")))
get_voltage_for_SiPM_x (float T, s_regulatorSettings *regulatorSettings)
{
  return get_voltage_for_SiPM (T, regulatorSettings->a, regulatorSettings->T_0,
			       regulatorSettings->U_0, regulatorSettings->U_offset);
}

/* COMPUTE FUNCTIONS */

int
compare_measurements (const void *a, const void *b)
{
  float diff = ((s_ADC_Measurement*) a)->adc_value - ((s_ADC_Measurement*) b)->adc_value;
  return (diff > 0) - (diff < 0); // Returns -1, 0, or 1
}

static float __attribute__((deprecated))
calculate_average (s_ADC_Measurement *data, size_t N, e_average method, float alpha,
		   uint32_t timestamp_ms)
{
  if (N == 0) return 0.0f;
  float sum = 0.0f;
  float result = 0.0f;
  float weight_sum = 0.0f;

  switch (method)
    {

    case e_average_NONE:
      result = data[0].adc_value;
      break;
    case e_average_STANDARD:
      for (size_t i = 0; i < N; i++)
	{
	  sum += data[i].adc_value;
	}
      result = sum / N;
      break;

    case e_average_EXPONENTIAL:
      result = data[0].adc_value;
      for (size_t i = 1; i < N; i++)
	{
	  result = alpha * data[i].adc_value + (1 - alpha) * result;
	}
      break;

    case e_average_MEDIAN:
      qsort (data, N, sizeof(s_ADC_Measurement), compare_measurements);
      result =
	  (N % 2 == 0) ?
	      (data[N / 2 - 1].adc_value + data[N / 2].adc_value) / 2.0 : data[N / 2].adc_value;
      break;

    case e_average_RMS:
      for (size_t i = 0; i < N; i++)
	{
	  sum += data[i].adc_value * data[i].adc_value;
	}
      result = sqrt (sum / N);
      break;

    case e_average_HARMONIC:
      for (size_t i = 0; i < N; i++)
	{
	  if (data[i].adc_value != 0)
	    {
	      sum += 1.0f / data[i].adc_value;
	    }
	}
      result = (sum != 0) ? N / sum : 0.0f;
      break;

    case e_average_GEOMETRIC:
      result = 1.0f;
      for (size_t i = 0; i < N; i++)
	{
	  result *= data[i].adc_value;
	}
      result = pow (result, 1.0 / N);
      break;

    case e_average_TRIMMED:
      qsort (data, N, sizeof(s_ADC_Measurement), compare_measurements);
      size_t trim = N / 10;
      sum = 0.0f;
      for (size_t i = trim; i < N - trim; i++)
	{
	  sum += data[i].adc_value;
	}
      result = sum / (N - 2 * trim);
      break;

    case e_average_WEIGHTED_EXPONENTIAL:
      for (size_t i = 0; i < N; i++)
	{
	  float r = timestamp_ms - data[i].timestamp_ms;
	  float weight = expf (-fabsf (r / alpha));
	  sum += data[i].adc_value * weight;
	  weight_sum += weight;
	}
      result = (weight_sum != 0) ? sum / weight_sum : 0.0f;
      break;
    }

  return result;
}

static inline size_t __attribute__ ((optimize("-O3")))
get_average_from_buffer (s_BufferADC *cb, size_t N, uint32_t timestamp_ms, uint32_t max_dt_ms,
			 e_average method, float *average_result, float alpha, float multiplicator)
{
  if (cb->head == cb->tail)
    {
      *average_result = NAN;
      return 0;
    }

  size_t cb_count = CircularBuffer_GetItemCount (cb);
  if (N > cb_count)
    {
      N = cb_count;
    }

  size_t start_index = (cb->head == 0) ? (cb->buffer_size - 1) : (cb->head - 1);
  size_t count = 0;
  float sum = 0.0f;
  float weight_sum = 0.0f;

  for (size_t i0 = 0; i0 < N; i0++)
    {
      if (check_time_diff_is_more_than (cb->buffer[start_index].timestamp_ms, timestamp_ms,
					max_dt_ms, cb->dt_ms) && (max_dt_ms != 0))
	{
	  break;
	}
      float value = cb->buffer[start_index].adc_value;
      switch (method)
	{
	case e_average_STANDARD:
	  {
	    sum += value;
	    break;
	  }
	case e_average_EXPONENTIAL:
	  {
	    sum = alpha * value + (1 - alpha) * sum;
	    break;
	  }
	case e_average_RMS:
	  {
	    sum += value * value;
	    break;
	  }
	case e_average_HARMONIC:
	  {
	    if (value != 0)
	      {
		sum += 1.0f / value;
	      }
	    break;
	  }
	case e_average_GEOMETRIC:
	  {
	    sum = (i0 == 0) ? value : sum * value;
	    break;
	  }
	case e_average_WEIGHTED_EXPONENTIAL:
	  {
	    float r = timestamp_ms - cb->buffer[start_index].timestamp_ms;
	    float weight = expf (-fabsf (r / alpha));
	    sum += value * weight;
	    weight_sum += weight;
	  }
	  break;
	default:
	  break;
	}
      count++;
      start_index = (start_index == 0) ? (cb->buffer_size - 1) : (start_index - 1);
    }

  switch (method)
    {
    case e_average_STANDARD:
      {
	*average_result = (count != 0) ? sum / count : 0.0f;
	break;
      }
    case e_average_EXPONENTIAL:
      // don't break
    case e_average_WEIGHTED_EXPONENTIAL:
      {
	*average_result = (weight_sum != 0) ? sum / weight_sum : 0.0f;
	break;
      }
    case e_average_RMS:
      {
	*average_result = sqrt (sum / count);
	break;
      }
    case e_average_HARMONIC:
      {
	*average_result = (sum != 0) ? count / sum : 0.0f;
	break;
      }
    case e_average_GEOMETRIC:
      {
	*average_result = pow (sum, 1.0 / count);
	break;
      }
    default:
      {
	*average_result = 0.0f;
	break;
      }
    }
  if (count == 0)
    {
      *average_result = NAN;
    }
  *average_result = (*average_result) * multiplicator;
  return count;
}

inline size_t __attribute__ ((always_inline, optimize("-O3")))
get_average_atSettings (s_channelSettings *a, float *here)
{
  return get_average_from_buffer (a->buffer_ADC, a->max_N, HAL_GetTick (), a->max_dt_ms,
				  a->averaging_method, here, a->alpha, a->multiplicator);
}


