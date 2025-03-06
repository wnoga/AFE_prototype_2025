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
  AFECommand_getSerialNumber = 0x00,
  AFECommand_getVersion = 0x01,
  AFECommand_resetAll = 0x03,

  AFECommand_getSensorDataSi_last = 0x30,
  AFECommand_getSensorDataSi_average = 0x31,
  AFECommand_getSensorDataSi_all_last = 0x32,
  AFECommand_getSensorDataSi_all_average = 0x33,
  AFECommand_setSensorDataSi_all_periodic_average = 0x34,

  AFECommand_getSensorDataSiAndTimestamp_average = 0x3B,
  AFECommand_getSensorDataSi_all_periodic_average = 0x3F,

  AFECommand_setSensorDataSi_periodic_last = 0x40,
  AFECommand_setSensorDataSiAndTimestamp_periodic_last = 0x41,
  AFECommand_setSensorDataSi_periodic_average = 0x42,
  AFECommand_setSensorDataSiAndTimestamp_periodic_average = 0x43,

  AFECommand_transmitSPIData = 0xA0,
  AFECommand_writeGPIO = 0xA2,

  AFECommand_setTemperatureLoopForChannelState_bySubdevice = 0xC0,
  AFECommand_setTemperatureLoopForChannelState_byMask = 0xC1,
  AFECommand_setDACValueRaw_bySubdevice = 0xC2,
  AFECommand_setDACValueSi_bySubdevice = 0xC3,
  AFECommand_stopTemperatureLoopForAllChannels = 0xC4,
  AFECommand_setDAC_bySubdevice = 0xC5,

  AFECommand_setAveragingMode = 0xD0,
  AFECommand_setAveragingAlpha = 0xD1,
  AFECommand_setAveragingBufferSize = 0xD2,
  AFECommand_setChannel_dt_ms = 0xD3,
  AFECommand_setAveraging_max_dt_ms = 0xD4,
  AFECommand_setChannel_multiplicator = 0xD5,
  AFECommand_setAveragingSubdevice = 0xD6,

  AFECommand_debug_machine_control = 0xF1
} AFECommand;

typedef enum
{
  AFECommandChannel_0 = 0b00000001,
  AFECommandChannel_1 = 0b00000010,
  AFECommandChannel_2 = 0b00000100,
  AFECommandChannel_3 = 0b00001000,
  AFECommandChannel_4 = 0b00010000,
  AFECommandChannel_5 = 0b00100000,
  AFECommandChannel_6 = 0b01000000,
  AFECommandChannel_7 = 0b10000000,
} AFECommandChannel;

typedef enum
{
  AFECommandSubdevice_master 	= 0b00000001,
  AFECommandSubdevice_slave 	= 0b00000010,
  AFECommandSubdevice_both 	= 0b00000011,
} AFECommandSubdevice;

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
