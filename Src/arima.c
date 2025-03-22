/*
 * arima.c
 *
 *  Created on: Mar 22, 2025
 *      Author: Wojciech Noga
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <stdint.h>

// Function to apply differencing (for integration part of ARIMA)
int
arima_differencing (uint32_t *series_timestamp, float *series_value, size_t n,
		    int d, float *diff_values, uint32_t *time_diffs)
{
  if (n >= d)
    {
      for (int diff = 0; diff < d; diff++)
	{
	  for (size_t i = 1; i < n - diff; i++)
	    {
	      diff_values[i - 1] = series_value[i] - series_value[i - 1];
	      time_diffs[i - 1] = series_timestamp[i] - series_timestamp[i - 1];
	    }
	}
      return 0;
    }
  else
    {
      return -1;
    }
}

// Function to apply inverse differencing (to reconstruct original scale)
float
arima_inverse_differencing (float *predicted, float *series_value, size_t n,
			    int d, float *output)
{
  if (predicted != output)
    {
      for (size_t _ = 0; _ < n; ++_)
	{
	  output[_] = predicted[_]; // Copy buffer
	}
    }
  for (int diff = 0; diff < d; diff++)
    {
      for (size_t i = 1; i < n; i++)
	{
	  output[i] = output[i - 1] + output[i];
	}
    }
  return output[n - 1]; // return last value
}

// Function to apply ARMA (ARIMA without differencing)
float
arima_apply_ARMA (float *diff_values, size_t n, uint32_t *time_diffs,
		  float *ar_coeffs, int p, float *ma_coeffs, int q, float alpha,
		  float *output)
{
  float error[n];
  for (size_t i = 0; i < n; ++i)
    {
      error[i] = 0.0;
    }
  float output_end = NAN;

  for (int t = 0; t < n; t++)
    {
      float ar_part = 0.0, ma_part = 0.0;

      for (int i = 1; i <= p; i++)
	{
	  if (t - i >= 0)
	    {

	      float weight =
		  alpha ?
		      expf (-time_diffs[t - i] * alpha) :
		      1.0 / (1.0 + time_diffs[t - i]); // Adjust AR effect by time gap
	      ar_part += ar_coeffs[i - 1] * diff_values[t - i] * weight;
	    }
	}

      for (int j = 1; j <= q; j++)
	{
	  if (t - j >= 0)
	    {
	      ma_part += ma_coeffs[j - 1] * error[t - j];
	    }
	}

      output[t] = ar_part + ma_part;
      error[t] = diff_values[t] - output[t];
      output_end = output[t];
    }
  return output_end;
}

int
_arima_example (int p, int d, int q)
{
  size_t MAX_DATA = 1000;
  uint32_t series_timestamp[MAX_DATA];
  float series_value[MAX_DATA];
  float diff_values[MAX_DATA];
  float predicted[MAX_DATA];
  float softened[MAX_DATA];
  uint32_t time_diffs[MAX_DATA];
  size_t n = 0;

  uint32_t timestamp = 0;
  for (n = 0; n < MAX_DATA; ++n)
    {
      series_timestamp[n] = timestamp;
      series_value[n] = 100.0
	  * (0.5 + ((sin (timestamp / 1000.0) + 1.0) / 2.0)
	      + 0.2 * (((float) rand () / RAND_MAX) - 0.5) * 2.0);
//      series_value[n] = 100 + (rand () % 10);
      timestamp += 100 + (rand () % 10);
    }

  float ar_coeffs[] =
    { 0.1 };
  float ma_coeffs[] =
    { -0.5 };

  arima_differencing (series_timestamp, series_value, n, d, diff_values,
		      time_diffs);
  arima_apply_ARMA (diff_values, n - d, time_diffs, ar_coeffs, p, ma_coeffs, q,
		    0, predicted);
  arima_inverse_differencing (predicted, series_value, n - d, d, softened);

  printf ("Unix Timestamp,Original,Predicted,Softened\n");
  for (int i = d; i < n; i++)
    {
      printf ("%ld,%.2f,%.2f,%.2f\n", series_timestamp[i], series_value[i],
	      series_value[i - 1] + predicted[i - d],
	      series_value[i - 1] + softened[i - d]);
    }

  return 0;
}

int
arima_example (void)
{
  return _arima_example (1, 1, 1);
}
