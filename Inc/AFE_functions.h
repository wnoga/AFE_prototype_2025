/*
 * AFE_functions.h
 *
 *  Created on: Nov 8, 2024
 *      Author: Wojciech Noga
 */

#ifndef AFE_FUNCTIONS_H_
#define AFE_FUNCTIONS_H_

#include "settings.h"

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

  AFECommand_resetCAN = 0x07,

  AFECommand_getSubdeviceStatus = 0x08,

  AFECommand_getSensorDataSi_last_byMask = 0x30,
  AFECommand_getSensorDataSi_average_byMask = 0x31,

  AFECommand_getSensorDataBytes_last_byMask = 0x32,
  AFECommand_getSensorDataBytes_average_byMask = 0x33,

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

  AFECommand_setTemperatureLoop_loop_every_ms = 0xB0,

  AFECommand_setTemperatureLoopForChannelState_byMask_asStatus = 0xC1,
  AFECommand_setDACValueRaw_bySubdeviceMask = 0xC2,
  AFECommand_setDACValueSi_bySubdeviceMask = 0xC3,
  AFECommand_stopTemperatureLoopForAllChannels = 0xC4,
  AFECommand_setDAC_bySubdeviceMask = 0xC5,
  AFECommand_setDACRampOneBytePerMillisecond_ms = 0xC6,
  AFECommand_setDACTargetSi_bySubdeviceMask = 0xC7,

  AFECommand_setAveragingMode_byMask = 0xD0,
  AFECommand_setAveragingAlpha_byMask = 0xD1,
  AFECommand_setAveragingBufferSize_byMask = 0xD2,
  AFECommand_setChannel_dt_ms_byMask = 0xD3,
  AFECommand_setAveraging_max_dt_ms_byMask = 0xD4,
  AFECommand_setAveragingSubdevice = 0xD6,
  AFECommand_setChannel_a_byMask = 0xD7,
  AFECommand_setChannel_b_byMask = 0xD8,
  AFECommand_setChannel_period_ms_byMask = 0xD9,

  AFECommand_setChannelBufferSize = 0xE0,

  AFECommand_setRegulator_ramp_enabled_byMask = 0xE1,
  AFECommand_setRegulator_T_opt_byMask = 0xE3,
  AFECommand_setRegulator_dT_byMask = 0xE4,
  AFECommand_setRegulator_a_dac_byMask = 0xE5,
  AFECommand_setRegulator_b_dac_byMask = 0xE6,
  AFECommand_setRegulator_dV_dT_byMask = 0xE7,
  AFECommand_setRegulator_V_opt_byMask = 0xE8,
  AFECommand_setRegulator_V_offset_byMask = 0xE9,

  AFECommand_debug_machine_control = 0xF1,
  AFECommand_clearRegulator_T_old = 0xF2
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
  e_average_WEIGHTED_EXPONENTIAL
#if USE_ARIMA
  ,e_average_ARIMA
#endif // USE_ARIMA
  ,e_average_len // Keep for know the length of that enum
} e_average;

typedef struct
{
  uint8_t channel_nr; // ADC channel number
  // exponential
  AFECommandSubdevice subdevice; // master or slave
  s_BufferADC *buffer_ADC; // pointer to buffer for ADC structure
  /* buffer and averaging settings */
  e_average averaging_method;

  uint32_t max_dt_ms; // maximum time difference in averaging
  float a; // for f=a*x+b
  float b; // for f=a*x+b
  float alpha; // weight modificator for averaging
  uint32_t max_N; // maximum number of data used for averaging

  uint32_t period_ms; // set how often data should be send periodically
  uint32_t period_ms_last;

} s_channelSettings;

typedef struct
{
  AFECommandSubdevice subdevice;
  s_channelSettings *temperature_channelSettings_ptr;
  s_channelSettings *voltage_channelSettings_ptr;
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
  int8_t ramp_target_reached;
  int8_t ramp_enabled;
#if DEBUG_SEND_BY_CAN_MACHINE_CONTROL
  uint16_t ramp_target_voltage_set_bits_old; // prevent sending many times msg when is not change in DAC target value
#endif // DEBUG_SEND_BY_CAN_MACHINE_CONTROL
  float V; // Last voltage
  float T; // Last temperature
  float V_target; // Last voltage target

  uint32_t loop_every_ms;
  uint32_t last_loop_every_ms;
} s_regulatorSettings;


float get_average_from_buffer (s_BufferADC *cb, size_t N, uint32_t timestamp_ms, uint32_t max_dt_ms, e_average method, float alpha);
float get_average_atSettings (s_channelSettings *a, uint32_t timestamp);
float get_voltage_for_SiPM_x (float T, s_regulatorSettings *regulatorSettings);
float faxplusb (float value, float a, float b);
float faxplusbcs (float value, s_channelSettings *ch);
uint8_t get_number_of_channels (uint8_t channels_mask);

uint16_t machine_DAC_convert_V_to_DAC_value (float V, s_regulatorSettings *regulatorSettings);

#endif /* AFE_FUNCTIONS_H_ */
