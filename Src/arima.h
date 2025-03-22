/*
 * arima.h
 *
 *  Created on: Mar 22, 2025
 *      Author: Wojciech Noga
 */

#ifndef ARIMA_H_
#define ARIMA_H_

void arima_differencing (uint32_t *series_timestamp, float *series_value, size_t n, int d, float *diff_values, uint32_t *time_diffs);
float arima_inverse_differencing (float *predicted, float *series_value, size_t n, int d, float *output);
float arima_apply_ARMA (float *diff_values, size_t n, uint32_t *time_diffs, float *ar_coeffs, int p, float *ma_coeffs, int q, float alpha, float *output);
int arima_example();

#endif /* ARIMA_H_ */
