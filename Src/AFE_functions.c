/*
 * AFE_functions.c
 *
 *  Created on: Nov 8, 2024
 *      Author: blondier94
 */

#include "AFE_functions.h"
#include "arima.h"
#include <math.h>
#if defined STM32F0
#include <stm32f0xx_hal.h>
#else
#include "main.h"
#endif

/* Driver */

inline float __attribute__ ((always_inline, optimize("-O3")))
get_voltage_for_SiPM (float T, float a, float T_0, float U_0, float U_cor)
{
  return a * (T - T_0) + U_0 + U_cor;
}

inline float __attribute__ ((always_inline, optimize("-O3")))
get_voltage_for_SiPM_x (float T, s_regulatorSettings *regulatorSettings)
{
  return get_voltage_for_SiPM (T, regulatorSettings->dV_dT, regulatorSettings->T_opt,
			       regulatorSettings->V_opt, regulatorSettings->V_offset);
}

/* COMPUTE FUNCTIONS */

inline float __attribute__ ((always_inline, optimize("-O3")))
faxplusb (float value, float a, float b)
{
  return a * value + b;
}

inline float __attribute__ ((always_inline, optimize("-O3")))
faxplusbcs (float value, s_channelSettings *ch)
{
  return faxplusb (value, ch->a, ch->b);
}

int
compare_measurements (const void *a, const void *b)
{
  float diff = ((s_ADC_Measurement*) a)->adc_value - ((s_ADC_Measurement*) b)->adc_value;
  return (diff > 0) - (diff < 0); // Returns -1, 0, or 1
}

static float __attribute__((deprecated))
calculate_average_old (s_ADC_Measurement *data, size_t N, e_average method, float alpha,
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
	      (data[N / 2 - 1].adc_value + data[N / 2].adc_value) / 2.0f : data[N / 2].adc_value;
      break;

    case e_average_RMS:
      for (size_t i = 0; i < N; i++)
	{
	  sum += data[i].adc_value * data[i].adc_value;
	}
      result = sqrtf (sum / N);
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
      result = powf (result, 1.0f / N);
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

inline size_t __attribute__ ((always_inline, optimize("-O3")))
next_index (size_t i, size_t i_0, size_t N, size_t buffer_size, uint8_t fromTail)
{
  size_t offset;
  if (fromTail)
    {
      offset = i_0 + i;
      if (offset >= N)
	{
	  offset -= N;
	}
      else
	{
	  offset = buffer_size - (N - offset);
	}
    }
  else
    {
      if (i_0 >= i)
	{
	  offset = i_0 - i;
	}
      else
	{
	  offset = buffer_size - (i - i_0);
	}
    }

  return offset % buffer_size;
}


static inline void __attribute__ ((always_inline, optimize("-O3")))
calculate_sum (float *sum, float *weight_sum, e_average method, s_ADC_Measurement *ptr,
	       uint32_t timestamp_ms, float alpha, size_t cnt)
{
  switch (method)
    {
    case e_average_STANDARD:
      {
	*sum += ptr->adc_value;
	break;
      }
    case e_average_EXPONENTIAL:
      {
	*sum = (alpha * (float) ptr->adc_value) + ((1.0f - alpha) * (*sum));
	break;
      }
    case e_average_RMS:
      {
	*sum += ptr->adc_value * ptr->adc_value;
	break;
      }
    case e_average_HARMONIC:
      {
	if (ptr->adc_value != 0)
	  {
	    *sum += 1.0f / (float) ptr->adc_value;
	  }
	break;
      }
    case e_average_GEOMETRIC:
      {
	*sum = (cnt == 0) ? ptr->adc_value : (*sum) * ptr->adc_value;
	break;
      }
    case e_average_WEIGHTED_EXPONENTIAL:
      {
	uint32_t ru = timestamp_ms - ptr->timestamp_ms;
	float r = fabsf ((float) ru);
	float weight = expf (-r * alpha);
	*sum += (weight * ptr->adc_value);
	*weight_sum += weight;
	break;
      }
#if USE_ARIMA
	case e_average_ARIMA:
{
	    /* Update series for ARIMA */
	    if (arima_n < N)
{
	      arima_series_timestamp[arima_n] = ptr->timestamp_ms;
	      arima_series_value[arima_n] = faxplusbcs(ptr->adc_value, a); // Apply transformation here.
	      ++arima_n;
}
	    break;
}
#endif
    default:
      break;
    }
}

static inline float __attribute__ ((always_inline, optimize("-O3")))
calculate_average (float sum, float weight_sum, e_average method, size_t cnt,
		   s_ADC_Measurement *ptr0)
{
  switch (method)
    {
    case e_average_STANDARD:
      {
	return (cnt != 0) ? sum / (float) cnt : ptr0->adc_value;
      }
    case e_average_EXPONENTIAL:
      {
	return sum;
      }
    case e_average_WEIGHTED_EXPONENTIAL:
      {
	return (weight_sum != 0) ? sum / weight_sum : ptr0->adc_value;
      }
    case e_average_RMS:
      {
	return sqrtf (sum / (float) cnt);
      }
    case e_average_HARMONIC:
      {
	return (sum != 0) ? (float) cnt / sum : 0.0f;
      }
    case e_average_GEOMETRIC:
      {
	return powf (sum, 1.0f / (float) cnt);
      }
#if USE_ARIMA
	case e_average_ARIMA:
	  {
	    // ARIMA(1,1,1) setup: AR(1) and MA(1) coefficients
	    float ar_coeffs[] =
	      { 0.1 };  // AR(1)
	    float ma_coeffs[] =
	      { -0.5 };  // MA(1)
	    int d = 1;
	    int p = sizeof(ar_coeffs) / sizeof(ar_coeffs[0]);
	    int q = sizeof(ma_coeffs) / sizeof(ma_coeffs[0]);
	    arima_differencing (arima_series_timestamp, arima_series_value,
				arima_n, d, arima_diff_values,
				arima_time_diffs);
	    arima_apply_ARMA (arima_diff_values, arima_n, arima_time_diffs,
			      ar_coeffs, p, ma_coeffs, q, alpha,
			      arima_predicted); // predicting
	    arima_inverse_differencing (arima_predicted, arima_series_value, // TODO Check if this is correct
					arima_n, d, arima_smoothing); // smoothing
	    return arima_series_value[arima_n - 1]
		+ arima_smoothing[arima_n - 1]; // add current value to smoothing or predicting
	  }
#endif
    default:
      {
	return ptr0->adc_value;
      }
    }
}

float __attribute__ ((optimize("-O3")))
get_average_from_buffer (s_BufferADC *cb, size_t N, uint32_t timestamp_ms, uint32_t max_dt_ms,
			 e_average method, float alpha)
{
  if (cb->head == cb->tail)
    {
      return NAN;
    }
  size_t cb_count = CircularBuffer_GetItemCount (cb);
  size_t i_0 = ((cb->head - 1)) % cb->buffer_size;
  s_ADC_Measurement *ptr0 = &cb->buffer[i_0];
  if ((cb_count == 1) || (method == e_average_NONE))
    {
      return (float) ptr0->adc_value;
    }
  else if (N > cb_count)
    {
      N = cb_count;
    }

  float average_result = 0.0f;
  size_t count = 0;
  float sum = 0.0f;
  float weight_sum = 0.0f;

#if USE_ARIMA
  size_t arima_n = 0;
  uint32_t *arima_series_timestamp =
      (method == e_average_ARIMA) ? malloc (sizeof(uint32_t) * N) : malloc (0);
  uint32_t *arima_time_diffs =
      (method == e_average_ARIMA) ? malloc (sizeof(uint32_t) * (N-1)) : malloc (0);
  float *arima_series_value =
      (method == e_average_ARIMA) ? malloc (sizeof(float) * N) : malloc (0);
  float *arima_diff_values =
      (method == e_average_ARIMA) ? malloc (sizeof(float) * N) : malloc (0);
  float *arima_predicted =
      (method == e_average_ARIMA) ? malloc (sizeof(float) * N) : malloc (0);
  float *arima_smoothing =
      (method == e_average_ARIMA) ? malloc (sizeof(float) * N) : malloc (0);
#endif

  s_ADC_Measurement *ptr;
  size_t i = 0;
  size_t cnt = 0;
  /* Set default value for sum */
  if ((method == e_average_EXPONENTIAL) || (method == e_average_GEOMETRIC))
    {
      sum = NAN;
    }
  for (count = 0; count < N; ++count)
    {

      // Here is place for a moving averages
      i = next_index (count, i_0, N, cb->buffer_size, 1);
      ptr = &cb->buffer[i];
      if ((timestamp_ms - ptr->timestamp_ms) > max_dt_ms)
	{
	  continue;
	}

      ++cnt;
      calculate_sum (&sum, &weight_sum, method, ptr, timestamp_ms, alpha, cnt);
    }

  /* Check if we have any data */
  if (cnt == 0)
    {
      average_result = NAN;
    }
  else
    {
      average_result = calculate_average (sum, weight_sum, method, cnt, ptr0);
    }
#if USE_ARIMA
  if (method == e_average_ARIMA)
  {
    free (arima_diff_values);
    free (arima_predicted);
    free (arima_series_timestamp);
    free (arima_series_value);
    free (arima_time_diffs);
    free (arima_smoothing);
  }
#endif
  return average_result;
}

float __attribute__ ((optimize("-O3")))
get_average_atSettings (s_channelSettings *a, uint32_t timestamp)
{
  return faxplusbcs (
      get_average_from_buffer (a->buffer_ADC, a->max_N, timestamp, a->max_dt_ms,
			       a->averaging_method, a->alpha),
      a);

}

inline uint8_t __attribute__ ((always_inline, optimize("-O3")))
get_number_of_channels (uint8_t channels_mask)
{
  uint8_t number_of_channels = 0;
  for (uint8_t i0 = 0; i0 < AFE_NUMBER_OF_CHANNELS; ++i0)
    {
      if (0x01 & (channels_mask >> i0))
	{
	  ++number_of_channels;
	}
    }
  return number_of_channels;
}

/***
 * FIXME Create conversion from float to DAC uint16_t value
 */
inline uint16_t __attribute ((always_inline, optimize("-O3")))
machine_DAC_convert_V_to_DAC_value (float V, s_regulatorSettings *regulatorSettings)
{
  return (uint16_t) roundf (faxplusb (V, regulatorSettings->a_dac, regulatorSettings->b_dac));
}
