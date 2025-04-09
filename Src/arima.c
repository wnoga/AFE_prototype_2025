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

/**
 * @brief Applies differencing to a time series.
 *
 * This function performs differencing on a time series, which is a crucial step
 * in ARIMA modeling to make the series stationary. Differencing involves
 * subtracting the previous value from the current value in the series.
 *
 * @param series_timestamp Array of timestamps for the time series.
 * @param series_value Array of values for the time series.
 * @param n Number of data points in the series.
 * @param d Degree of differencing (number of times differencing is applied).
 * @param diff_values Output array to store the differenced values.
 * @param time_diffs Output array to store the time differences between data points.
 * @return 0 on success, -1 if the degree of differencing is greater than or equal to the number of data points.
 */
int
arima_differencing (uint32_t *series_timestamp, float *series_value, size_t n,
		    int d, float *diff_values, uint32_t *time_diffs)
{
  if (n > d)
    {
      // Apply differencing 'd' times
      for (int diff_order = 0; diff_order < d; diff_order++)
	{
	  // Iterate through the series, applying differencing
	  for (size_t i = 1; i < n - diff_order; i++)
	    {
	      // Calculate the difference between the current and previous value
	      diff_values[i - 1] = series_value[i] - series_value[i - 1]; // diff_values[i - 1]
	      // Calculate the time difference between the current and previous timestamp
	      time_diffs[i - 1] = series_timestamp[i] - series_timestamp[i - 1];
	    }
      #if ARIMA_UPDATE_SERIES_DURING_DIFFERENCING
	  // Update the series_value for the next iteration of differencing
	  for (size_t i = 0; i < n - diff_order - 1; i++)
	    {
	      series_value[i] = diff_values[i];
	    }
      #endif // ARIMA_UPDATE_SERIES_DURING_DIFFERENCING
	}
      return 0;
    }
  else
    {
      return -1;
    }

}

/**
 * @brief Applies inverse differencing to reconstruct the original scale of a time series.
 *
 * This function reverses the differencing process to reconstruct the original scale
 * of a time series after predictions have been made on the differenced data.
 *
 * @param predicted Array of predicted values on the differenced series.
 * @param series_value Array of original series values (used for initial values).
 * @param n Number of data points in the series.
 * @param d Degree of differencing that was applied.
 * @param output Output array to store the inverse differenced values.
 * @return The last value of the reconstructed series.
 */
float
arima_inverse_differencing (float *predicted, float *series_value, size_t n,
			    int d, float *output)
{
  if (predicted != output) // Check if the input and output buffers are the same
    {
      for (size_t _ = 0; _ < n; ++_) // Copy buffer
	{
	  output[_] = predicted[_];
	}
    }
  // Apply inverse differencing 'd' times
  for (int diff_order = 0; diff_order < d; diff_order++)
    {
      // Iterate through the series, applying inverse differencing
      for (size_t i = 1; i < n; i++) // n-diff_order
	{
	  // Reconstruct the original scale by adding the previous value
	  output[i] = output[i - 1] + output[i]; // output[i]
	  // output[i] = output[i - 1] + predicted[i]; // output[i]
	}
    }
  return output[n - 1]; // return last value
}

// Function to apply ARMA (ARIMA without differencing)
float
/**
 * @brief Applies the ARMA model to a time series.
 *
 * This function implements the Autoregressive Moving Average (ARMA) model, which
 * is a combination of Autoregressive (AR) and Moving Average (MA) models. It
 * predicts future values in a time series based on past values and past errors.
 *
 * @param diff_values Array of differenced values (or original values if no differencing).
 * @param n Number of data points in the series.
 * @param time_diffs Array of time differences between data points.
 * @param ar_coeffs Array of AR coefficients.
 * @param p Order of the AR model.
 * @param ma_coeffs Array of MA coefficients.
 * @param q Order of the MA model.
 * @param alpha Exponential decay factor for time-weighted AR component.
 * @param output Output array to store the ARMA predicted values.
 */
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

  for (int t = 0; t < n; t++) // Iterate through the time series
    {
      float ar_part = 0.0, ma_part = 0.0; // Initialize AR and MA components

      // Calculate the AR component
      for (int i = 1; i <= p; i++) // Iterate through AR coefficients
	{
	  if (t - i >= 0) // Check if the past value is within the series
	    {
	      // Calculate the weight based on time difference and alpha
	      float weight =
		  alpha ?
		      expf (-time_diffs[t - i] * alpha) :
		      1.0 / (1.0 + time_diffs[t - i]); // Adjust AR effect by time gap (if alpha is 0)
	      // Add the weighted past value to the AR component
	      ar_part += ar_coeffs[i - 1] * diff_values[t - i] * weight; // ar_coeffs[i - 1]
	    }
	}

      // Calculate the MA component
      for (int j = 1; j <= q; j++) // Iterate through MA coefficients
	{
	  if (t - j >= 0) // Check if the past error is within the series
	    {
	      // Add the weighted past error to the MA component
	      ma_part += ma_coeffs[j - 1] * error[t - j];
	    }
	}

      output[t] = ar_part + ma_part;
      error[t] = diff_values[t] - output[t];
      output_end = output[t];
    }
  return output_end;
}

/**
 * @brief Example function demonstrating the usage of ARIMA functions.
 *
 * This function generates a sample time series, applies ARIMA modeling, and
 * prints the results to the console.
 *
 * @return 0 on successful execution.
 */
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
	  * (0.5 + ((sin (timestamp / 1000.0) + 1.0) / 2.0) // sin(x)
	      + 0.2 * (((float) rand () / RAND_MAX) - 0.5) * 2.0); // noise
//      series_value[n] = 100 + (rand () % 10);
      timestamp += 100 + (rand () % 10); // add noise to timestamp
    }

  float ar_coeffs[] =
    { 0.1 };
  float ma_coeffs[] =
    { -0.5 };

  arima_differencing (series_timestamp, series_value, n, d, diff_values, // differencing
		      time_diffs);
  arima_apply_ARMA (diff_values, n - d, time_diffs, ar_coeffs, p, ma_coeffs, q, // ARMA
		    0, predicted);
  arima_inverse_differencing (predicted, series_value, n - d, d, softened); // inverse differencing

  printf ("Unix Timestamp,Original,Predicted,Softened\n");
  for (int i = d; i < n; i++)
    {
      printf ("%ld,%.2f,%.2f,%.2f\n", series_timestamp[i], series_value[i], // print values
	      series_value[i - 1] + predicted[i - d],
	      series_value[i - 1] + softened[i - d]);
    }

  return 0;
}

/**
 * @brief Example function to be called from outside.
 *
 * This function sets up the ARIMA model parameters and calls the internal
 * example function.
 */
int
arima_example (void)
{
  return _arima_example (1, 1, 1);
}
