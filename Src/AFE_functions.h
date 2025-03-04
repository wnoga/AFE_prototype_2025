/*
 * AFE_functions.h
 *
 *  Created on: Nov 8, 2024
 *      Author: Wojciech Noga
 */

#ifndef AFE_FUNCTIONS_H_
#define AFE_FUNCTIONS_H_

#include "BufferADC.h"
//#include "parser_CAN.h"

typedef enum
{
  e_subdevice_master=0,
  e_subdevice_slave,
} e_subdevice;

typedef enum
{
  e_average_NONE,
  e_average_STANDARD,
  e_average_EXPONENTIAL,
  e_average_MEDIAN,
  e_average_RMS,
  e_average_HARMONIC,
  e_average_GEOMETRIC,
  e_average_TRIMMED,
  e_average_WEIGHTED_EXPONENTIAL
} e_average;

typedef struct
{
    uint8_t channel_nr; // ADC channel number
  // exponential
  e_subdevice subdevice; // master or slave
  s_BufferADC *buffer_ADC; // pointer to buffer for ADC structure
  /* buffer and averaging settings */
  size_t buffer_size;
  uint32_t dt_ms; // time between two measurements
  e_average averaging_method;
//  uint32_t tau; // parameter for averaging

  uint32_t max_dt_ms; // maximum time difference in averaging
  float multiplicator; // multiplicator for raw ADC value (convert to real value)
//  uint32_t bin_width_ms; // bin width for averaging
  float alpha; // weight modificator for averaging
  uint32_t max_N; // maximum number of data used for averaging

  uint32_t period_ms; // set how often data should be send periodically
  uint32_t period_ms_last;

} s_channelSettings;

typedef struct
{
  e_subdevice subdevice;
  s_channelSettings *temperature_channelSettings_ptr;
  /* Temperature loop parameters */
  float dU;
  float dT;
  float T_0;
  float U_0;
  float U_cor;
  /* Old temperature value */
  float T_old;
  uint32_t timestamp_ms_old;
  int8_t enabled;
} s_regulatorSettings;


size_t get_average_atSettings (s_channelSettings *a, float *here);
float get_voltage_for_SiPM_x (float T, s_regulatorSettings *regulatorSettings);

#endif /* AFE_FUNCTIONS_H_ */
