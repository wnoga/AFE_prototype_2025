/*
 * AFE_functions.h
 *
 *  Created on: Nov 8, 2024
 *      Author: Wojciech Noga
 */

#ifndef AFE_FUNCTIONS_H_
#define AFE_FUNCTIONS_H_

#define USE_STACK_FOR_BUFFER 0
#define ADC_MEASUREMENT_RAW_SIZE_MAX 256
#define AFE_NUMBER_OF_CHANNELS 8
#define AFE_NUMBER_OF_SUBDEVICES 2
#define ADC_MEASUREMENT_RAW_DEFAULT_DT_MS 1000

#define AFE_DAC_MAX 0xFFF
#define AFE_DAC_START 0xFFF

#define AFE_REGULATOR_DEFAULT_a_dac (-259.602373913043)
#define AFE_REGULATOR_DEFAULT_b_dac (16530.5430434783)
#define AFE_REGULATOR_DEFAULT_dV_dT 0.05
#define AFE_REGULATOR_DEFAULT_T0 25.0
#define AFE_REGULATOR_DEFAULT_U0 55.0
#define AFE_REGULATOR_DEFAULT_U_offset 0.0
#define AFE_REGULATOR_DEFAULT_dT 1.0

#define NUMBER_OF_AD8402_CHANNELS 2

#define USE_ARIMA 0

#include "BufferADC.h"
#include <stdio.h>

typedef enum
{
  AFECommand_getSerialNumber = 0x00,
  AFECommand_getVersion = 0x01,
  AFECommand_resetAll = 0x03,
  AFECommand_startADC = 0x04,
  AFECommand_getTimestamp = 0x05,
  AFECommand_getSyncTimestamp = 0x06,

  AFECommand_getSensorDataSi_last_byMask = 0x30,
  AFECommand_getSensorDataSi_average_byMask = 0x31,

//  AFECommand_getSensorDataSiAndTimestamp_average_byMask = 0x3B,
  AFECommand_getSensorDataSi_periodic = 0x3F,

  AFECommand_setSensorDataSi_periodic_last = 0x40,
  AFECommand_setSensorDataSiAndTimestamp_periodic_last = 0x41,
  AFECommand_setSensorDataSi_periodic_average = 0x42,
  AFECommand_setSensorDataSiAndTimestamp_periodic_average = 0x43,

  AFECommand_transmitSPIData = 0xA0,
  AFECommand_setAD8402Value_byte_byMask = 0xA1,
  AFECommand_writeGPIO = 0xA2,
  AFECommand_setCanMsgBurstDelay_ms = 0xA3,
  AFECommand_setAfe_can_watchdog_timeout_ms = 0xA4,

  AFECommand_setTemperatureLoopForChannelState_byMask_asStatus = 0xC1,
  AFECommand_setDACValueRaw_bySubdeviceMask = 0xC2,
  AFECommand_setDACValueSi_bySubdeviceMask = 0xC3,
  AFECommand_stopTemperatureLoopForAllChannels = 0xC4,
  AFECommand_setDAC_bySubdeviceMask = 0xC5,
  AFECommand_setDACRampOneBytePerMillisecond_ms = 0xC6,

  AFECommand_setAveragingMode_byMask = 0xD0,
  AFECommand_setAveragingAlpha_byMask = 0xD1,
  AFECommand_setAveragingBufferSize_byMask = 0xD2,
  AFECommand_setChannel_dt_ms_byMask = 0xD3,
  AFECommand_setAveraging_max_dt_ms_byMask = 0xD4,
  AFECommand_setChannel_multiplicator_byMask = 0xD5,
  AFECommand_setAveragingSubdevice = 0xD6,
  AFECommand_setChannel_a_byMask = 0xD7,
  AFECommand_setChannel_b_byMask = 0xD8,
  AFECommand_setChannel_period_ms_byMask = 0xD9,

  AFECommand_setChannelBufferSize = 0xE0,

  AFECommand_setRegulator_a_dac_byMask = 0xE5,
  AFECommand_setRegulator_b_dac_byMask = 0xE6,
  AFECommand_setRegulator_dV_dT_byMask = 0xE7,
  AFECommand_setRegulator_V_opt_byMask = 0xE8,
  AFECommand_setRegulator_V_offset_byMask = 0xE9,

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
  e_ADC_CHANNEL_DC_LEVEL_MEAS0=0,
  e_ADC_CHANNEL_DC_LEVEL_MEAS1,
  e_ADC_CHANNEL_U_SIPM_MEAS0,
  e_ADC_CHANNEL_U_SIPM_MEAS1,
  e_ADC_CHANNEL_I_SIPM_MEAS0,
  e_ADC_CHANNEL_I_SIPM_MEAS1,
  e_ADC_CHANNEL_TEMP_EXT,
  e_ADC_CHANNEL_TEMP_LOCAL,
} e_ADC_CHANNEL;

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
  e_average_WEIGHTED_EXPONENTIAL,
  e_average_ARIMA
} e_average;

typedef struct __attribute__((packed))
{
  uint8_t channel_nr; // ADC channel number
  // exponential
  e_subdevice subdevice; // master or slave
  s_BufferADC *buffer_ADC; // pointer to buffer for ADC structure
  /* buffer and averaging settings */
//  size_t buffer_size;
//  uint32_t dt_ms; // time between two measurements
  e_average averaging_method;
//  uint32_t tau; // parameter for averaging

  uint32_t max_dt_ms; // maximum time difference in averaging
  float multiplicator; // multiplicator for raw ADC value (convert to real value)
  float a; // for f=a*x+b
  float b; // for f=a*x+b
//  uint32_t bin_width_ms; // bin width for averaging
  float alpha; // weight modificator for averaging
  uint32_t max_N; // maximum number of data used for averaging

  uint32_t period_ms; // set how often data should be send periodically
  uint32_t period_ms_last;

} s_channelSettings;

typedef struct __attribute__((packed))
{
  e_subdevice subdevice;
  s_channelSettings *temperature_channelSettings_ptr;
  /* Temperature loop parameters */
  /* From TempLoop.csv */
  float dT; // [deg C] minimum temperature change to drive ADC
  /* a*(T-T_0)+U_0+U_offset */
  float dV_dT; // [V/deg C]
  float V_opt; // U_0 [V]
  float T_opt; // [deg C]
  float V_offset; // [V] (default 0), not from config file
  /* Old temperature value */
  float T_old;
  int8_t enabled;

  float a_dac; // convert Volts to DAC in faxplusb
  float b_dac; // convert Voolts to DAC in faxplusb

  uint16_t ramp_bit_step;
  uint32_t ramp_bit_step_every_ms;
  uint32_t ramp_bit_step_timestamp_old_ms;
  uint16_t ramp_curent_voltage_set_bits;
  uint16_t ramp_target_voltage_set_bits;
#if DEBUG_SEND_BY_CAN_MACHINE_CONTROL
  uint16_t ramp_target_voltage_set_bits_old; // prevent sending many times msg when is not change in DAC target value
#endif // DEBUG_SEND_BY_CAN_MACHINE_CONTROL
} s_regulatorSettings;


float get_average_atSettings (s_channelSettings *a, uint32_t timestamp);
float get_voltage_for_SiPM_x (float T, s_regulatorSettings *regulatorSettings);
float faxplusb (float value, float a, float b);
float faxplusbcs (float value, s_channelSettings *ch);
uint8_t get_number_of_channels (uint8_t channels_mask);

#endif /* AFE_FUNCTIONS_H_ */
