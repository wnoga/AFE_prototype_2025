/*
 * machine_main.c
 *
 *  Created on: Feb 18, 2025
 *      Author: blondier94
 */

#include "machine_main.h"

#include "BufferADC.h"
#include "AFE_functions.h"
#include <math.h>
#include <stdbool.h>

#define X_FAST_HANDLE_FUNCTION_DECLARE static inline void __attribute__((optimize("-O3")))
#define X_SLOW_HANDLE_FUNCTION_DECLARE static inline void __attribute__((optimize("-Os")))
#define X_MEDIUM_HANDLE_FUNCTION_DECLARE static inline void __attribute__((optimize("-O2")))

#if AFE_ADC_SOFT_LAUNCHED
static int8_t afe_adc_soft_active = 0;
static uint32_t afe_adc_soft_timestamp_ms = 0;
static uint32_t afe_adc_soft_period_ms = 500;
#if !AFE_ADC_SOFT_LAUNCHED_SIMPLE_POOL
static int8_t afe_adc_i = 0;
static int8_t afe_adc_soft_started = 0;
#endif // !AFE_ADC_SOFT_LAUNCHED_SIMPLE_POOL
#endif

volatile uint16_t adc_dma_buffer[AFE_NUMBER_OF_CHANNELS];
#if USE_STACK_FOR_BUFFER
s_ADC_Measurement *adc_measurement_raw[AFE_NUMBER_OF_CHANNELS];
#else
static s_ADC_Measurement adc_measurement_raw[AFE_NUMBER_OF_CHANNELS][ADC_MEASUREMENT_RAW_SIZE_MAX];
#endif

//PA0     ------> ADC_IN0
//PA1     ------> ADC_IN1
//PA2     ------> ADC_IN2
//PA3     ------> ADC_IN3
//PA6     ------> ADC_IN6
//PA7     ------> ADC_IN7
//PB0     ------> ADC_IN8
//PB1     ------> ADC_IN9

typedef enum
{
  e_adc_channel_DC_LEVEL_MEAS0 = 0,
  e_adc_channel_DC_LEVEL_MEAS1,
  e_adc_channel_U_SIPM_MEAS0,
  e_adc_channel_U_SIPM_MEAS1,
  e_adc_channel_I_SIPM_MEAS0,
  e_adc_channel_I_SIPM_MEAS1,
  e_adc_channel_TEMP_EXT,
  e_adc_channel_TEMP_LOCAL
} e_adc_channel;

static s_BufferADC bufferADC[AFE_NUMBER_OF_CHANNELS];

static s_channelSettings afe_channelSettings[AFE_NUMBER_OF_CHANNELS];

static s_regulatorSettings afe_regulatorSettings[AFE_NUMBER_OF_SUBDEVICES]; // Regulator settings for master and slave

static e_machine_main machine_main_status = e_machine_main_init;

void __attribute__ ((cold, optimize("-Os")))
config_adc_channels (ADC_HandleTypeDef *hadc)
{
  ADC_ChannelConfTypeDef sConfig =
    { 0 };
  const uint32_t channels[] =
    {
    ADC_CHANNEL_0,
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_6,
    ADC_CHANNEL_7,
    ADC_CHANNEL_8,
    ADC_CHANNEL_9 };

  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = AFE_ADC_SAMPLING_TIME;
  for (size_t i0 = 0; i0 < AFE_NUMBER_OF_CHANNELS; ++i0)
    {
      sConfig.Channel = channels[i0];
      HAL_ADC_ConfigChannel (hadc, &sConfig);
    }
}

void
update_buffer_by_channelSettings (s_channelSettings *channelSettings)
{
  channelSettings->buffer_ADC->tail = channelSettings->buffer_ADC->head = 0;
}

#if USE_STACK_FOR_BUFFER
void
change_buffer_size_by_channelSettings (s_channelSettings *channelSettings, size_t new_buffer_size)
{
  channelSettings->buffer_ADC->buffer_size = new_buffer_size;
  uint32_t tmp = channelSettings->period_ms; // Temporary disable writing to this memory
  channelSettings->period_ms = 0;
  realloc(channelSettings->buffer_ADC->buffer,new_buffer_size);
  channelSettings->period_ms = tmp;
}
#endif // USE_STACK_FOR_BUFFER
/***
 * tmp->data[0] and tmp->data[1] should be set before this function
 */
static void __attribute__((deprecated))
CANCircularBuffer_enqueueMessage_and_update_channelSettings_byMsg (CANCircularBuffer_t *cb,
								   const s_can_msg_recieved *msg,
								   s_channelSettings *chs0,
								   CAN_Message_t *tmp,
								   const size_t offset,
								   const uint8_t size)
{
  // NULL pointer checks
  if (!cb || !msg || !chs0 || !tmp) return;

  uint8_t channels = msg->Data[2];
  tmp->data[2] = channels;

  // Prevent buffer overflow
  if (size > sizeof(tmp->data) - 3) return;

  memcpy (&tmp->data[3], &msg->Data[3], size);
  tmp->dlc = 3 + size; // Ensure valid DLC calculation

  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
    {
      // Ensure we only shift within valid bit range
      if (channel < 8 && (channels & (1 << channel)) != 0)
	{
	  // Prevent buffer overflow
	  if (offset + size <= sizeof(s_channelSettings))
	    {
	      memcpy ((uint8_t*) &chs0[channel] + offset, &msg->Data[3], size);
	      update_buffer_by_channelSettings (&chs0[channel]);
	    }
	}
    }

  CANCircularBuffer_enqueueMessage (cb, tmp);
}

static void
machine_GPIO_WritePin (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
#if DEBUG_HARDWARE_CONTROL_DISABLED | HARDWARE_CONTROL_GPIO_DISABLED
#warning "DEBUG_HARDWARE_CONTROL_DISABLED"
  return;
#endif
  HAL_GPIO_WritePin (GPIOx, GPIO_Pin, PinState);
}

static HAL_StatusTypeDef
machine_SPI_Transmit (SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
#if DEBUG_HARDWARE_CONTROL_DISABLED | HARDWARE_CONTROL_SPI_DISABLED
#warning "DEBUG_HARDWARE_CONTROL_DISABLED"
  return HAL_OK;
#endif
  /* SPI should be set to send 10-bits */
  return HAL_SPI_Transmit (hspi, pData, Size, Timeout);
}

static HAL_StatusTypeDef
AD8402_Write (SPI_HandleTypeDef *hspi, uint8_t channel, uint8_t value, uint32_t timeout)
{
  uint16_t toTransmit = (channel & 0x01) | (value & 0xFF); // A1 should be always 0 for AD8402
  return HAL_SPI_Transmit (hspi, (uint8_t*) &toTransmit, 1, timeout);
}

static void
machine_DAC_set (s_regulatorSettings *rptr, uint16_t value)
{
#if DEBUG_HARDWARE_CONTROL_DISABLED | HARDWARE_CONTROL_DAC_DISABLED
#warning "DEBUG_HARDWARE_CONTROL_DISABLED"
  return;
#endif
  rptr->ramp_curent_voltage_set_bits = value;
  switch (rptr->subdevice)
    {
    case AFECommandSubdevice_master:
      {
	if (HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value) != HAL_OK)
	  {
	    Error_Handler();
	  }
	break;
      }
    case AFECommandSubdevice_slave:
      {
	if (HAL_DAC_SetValue (&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value) != HAL_OK)
	  {
	    Error_Handler();
	  }
	break;
      }
    case AFECommandSubdevice_both:
      {
	if (HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value) != HAL_OK)
	  {
	    Error_Handler();
	  }
	if (HAL_DAC_SetValue (&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value) != HAL_OK)
	  {
	    Error_Handler();
	  }
	break;
      }
    default:
      break;
    }
}

static HAL_StatusTypeDef
machine_DAC_switch (s_regulatorSettings *rptr, uint8_t enable)
{
#if DEBUG_HARDWARE_CONTROL_DISABLED | HARDWARE_CONTROL_DAC_DISABLED
#warning "DEBUG_HARDWARE_CONTROL_DISABLED"
  return HAL_OK;
#endif
  switch (rptr->subdevice)
    {
    case AFECommandSubdevice_master:
      {
	if (enable == 1)
	  {
	    if (HAL_DAC_Start (&hdac, DAC_CHANNEL_1) != HAL_OK)
	      {
		Error_Handler();
	      }
	  }
	else
	  {
	    if (HAL_DAC_Stop (&hdac, DAC_CHANNEL_1) != HAL_OK)
	      {
		Error_Handler();
	      }
	  }
	break;
      }
    case AFECommandSubdevice_slave:
      {
	if (enable == 1)
	  {
	    if (HAL_DAC_Start (&hdac, DAC_CHANNEL_2) != HAL_OK)
	      {
		Error_Handler();
	      }
	  }
	else
	  {
	    if (HAL_DAC_Stop (&hdac, DAC_CHANNEL_2) != HAL_OK)
	      {
		Error_Handler();
	      }
	  }
	break;
      }
    case AFECommandSubdevice_both:
      {
	if (enable == 1)
	  {
	    if (HAL_DAC_Start (&hdac, DAC_CHANNEL_1) != HAL_OK)
	      {
		Error_Handler();
	      }
	    if (HAL_DAC_Start (&hdac, DAC_CHANNEL_2) != HAL_OK)
	      {
		Error_Handler();
	      }
	  }
	else
	  {
	    if (HAL_DAC_Stop (&hdac, DAC_CHANNEL_1) != HAL_OK)
	      {
		Error_Handler();
	      }
	    if (HAL_DAC_Stop (&hdac, DAC_CHANNEL_2) != HAL_OK)
	      {
		Error_Handler();
	      }
	  }
	break;
      }
    default:
      break;
    }
  return HAL_OK;
}

static void
machine_DAC_set_and_start (s_regulatorSettings *rptr, uint16_t value)
{
#if DEBUG_HARDWARE_CONTROL_DISABLED | HARDWARE_CONTROL_DAC_DISABLED
#warning "DEBUG_HARDWARE_CONTROL_DISABLED"
  return;
#endif
  machine_DAC_set (rptr, value);
  machine_DAC_switch (rptr, 1);
}

void __attribute__ ((cold, optimize("-Os")))
machine_main_init_0 (void)
{
  /* Set state to init */
  machine_main_status = e_machine_main_init;
  for (uint8_t i0 = 0; i0 < AFE_NUMBER_OF_CHANNELS; ++i0)
    {
      afe_channelSettings[i0].channel_nr = i0;
      afe_channelSettings[i0].averaging_method = e_average_NONE;
      afe_channelSettings[i0].alpha = 1.0;
      afe_channelSettings[i0].buffer_ADC = &bufferADC[i0];
      afe_channelSettings[i0].max_dt_ms = 3600 * 1000; // 1 hour
      afe_channelSettings[i0].a = 1.0;
      afe_channelSettings[i0].b = 0.0;
      afe_channelSettings[i0].max_N = ADC_MEASUREMENT_RAW_SIZE_MAX;

      afe_channelSettings[i0].period_ms = 0;
      afe_channelSettings[i0].period_ms_last = 0;

      init_buffer (&bufferADC[i0], &adc_measurement_raw[i0][0], ADC_MEASUREMENT_RAW_SIZE_MAX,
      ADC_MEASUREMENT_RAW_DEFAULT_DT_MS);
    }
  /* Set default values for regulator */

  for (uint8_t i0 = 0; i0 < 2; ++i0)
    {
      memset (&afe_regulatorSettings[i0], 0, sizeof(s_regulatorSettings));
#if AFE_REGULATOR_DEFAULT_RAMP_ENABLED
      afe_regulatorSettings[i0].ramp_enabled = AFE_REGULATOR_DEFAULT_RAMP_ENABLED;
#endif
#if AFE_REGULATOR_SET_DEFAULT_PARAMS
      afe_regulatorSettings[i0].dV_dT = AFE_REGULATOR_DEFAULT_dV_dT;
      afe_regulatorSettings[i0].T_opt = AFE_REGULATOR_DEFAULT_T0;
      afe_regulatorSettings[i0].V_opt = AFE_REGULATOR_DEFAULT_U0;
      afe_regulatorSettings[i0].V_offset = AFE_REGULATOR_DEFAULT_U_offset; // Voltage offset
      afe_regulatorSettings[i0].dT = AFE_REGULATOR_DEFAULT_dT; // delta Temperature when new DAC value can be set
      afe_regulatorSettings[i0].T_old = AFE_REGULATOR_DEFAULT_T_old;
#endif // AFE_REGULATOR_SET_DEFAULT_PARAMS
#if AFE_REGULATOR_SET_DEFAULT_RAMP
#if USE_SMALLER_STEPS_NEAR_DAC_TARGET
      afe_regulatorSettings[i0].ramp_bit_step = AFE_REGULATOR_DEFAULT_ramp_bit_step;
#else
      afe_regulatorSettings[i0].ramp_bit_step = 1;
#endif
      afe_regulatorSettings[i0].ramp_bit_step_every_ms =
      AFE_REGULATOR_DEFAULT_ramp_bit_step_every_ms;
      afe_regulatorSettings[i0].ramp_target_voltage_set_bits = AFE_DAC_START;
      afe_regulatorSettings[i0].ramp_curent_voltage_set_bits = 0x0FFF & (~AFE_DAC_START);
#if DEBUG_SEND_BY_CAN_MACHINE_CONTROL
      afe_regulatorSettings[i0].ramp_target_voltage_set_bits_old = 0x0FFF & (~AFE_DAC_START);
#endif // DEBUG_SEND_BY_CAN_MACHINE_CONTROL
#endif // AFE_REGULATOR_SET_DEFAULT_RAMP
      afe_regulatorSettings[i0].loop_every_ms = AFE_TEMPERATURE_LOOP_DEFAULT_loop_every_ms;
    }

  /* Set channel for temperature */
  /* Append ADC channel to regulator settings */
  afe_regulatorSettings[0].temperature_channelSettings_ptr =
      &afe_channelSettings[e_ADC_CHANNEL_TEMP_LOCAL];
  afe_regulatorSettings[0].voltage_channelSettings_ptr =
      &afe_channelSettings[e_ADC_CHANNEL_U_SIPM_MEAS0];
  afe_regulatorSettings[0].subdevice = AFECommandSubdevice_master;

  afe_regulatorSettings[1].temperature_channelSettings_ptr =
      &afe_channelSettings[e_ADC_CHANNEL_TEMP_EXT];
  afe_regulatorSettings[1].voltage_channelSettings_ptr =
      &afe_channelSettings[e_ADC_CHANNEL_U_SIPM_MEAS1];
  afe_regulatorSettings[1].subdevice = AFECommandSubdevice_slave;
}

static inline void __attribute__((always_inline, optimize("-O3")))
process_temperature_loop (s_regulatorSettings *rptr, uint32_t timestamp_ms)
{
#if HARDWARE_CONTROL_TEMPERATURE_LOOP_DISABLED
  // Temperature loop is disabled, do nothing.
  return;
#endif // HARDWARE_CONTROL_TEMPERATURE_LOOP_DISABLED
  if (rptr->enabled == 0)
    {
      return;
    }
  // 0. Limit frequency.
  if ((timestamp_ms - rptr->last_loop_every_ms)
      < rptr->loop_every_ms)
    {
      rptr->last_loop_every_ms = timestamp_ms;
      // 1. Get the current temperature from the sensor.
      float current_temperature = get_average_atSettings (rptr->temperature_channelSettings_ptr,
							  timestamp_ms);
      // 2. Validate the temperature reading.
      if (isnan(current_temperature))
	{
	  /* Skip if temperature reading is not a number (e.g., sensor error). */
	  return;
	}

      // 3. Check if the temperature has changed significantly (outside the dead-band).
      float temperature_delta = fabsf (current_temperature - rptr->T_old);
      if (temperature_delta < rptr->dT)
	{
	  // No significant change, no adjustment needed.
	  return;
	}

      // 4. Calculate the new target voltage and corresponding DAC value.
      float new_target_voltage = get_voltage_for_SiPM_x (current_temperature, rptr);
      uint16_t new_target_dac_bits = machine_DAC_convert_V_to_DAC_value (new_target_voltage, rptr);

      // 5. Update the regulator's state with the new values.
      // Set the target for the DAC ramp function.
      rptr->ramp_target_voltage_set_bits = new_target_dac_bits;

      // Update status variables for monitoring.
      rptr->T = current_temperature;
      rptr->V_target = new_target_voltage;

      // Update the last-seen temperature for the next dead-band check.
      rptr->T_old = current_temperature;

#if DEBUG_SEND_BY_CAN_MACHINE_CONTROL
      // 6. Send a debug message via CAN if the target DAC value has changed.
      if (new_target_dac_bits != rptr->ramp_target_voltage_set_bits_old)
	{
	  CAN_Message_t tmp; // Local message for debug
	  tmp.id = CAN_ID_IN_MSG;
	  tmp.timestamp = HAL_GetTick (); // for timeout
	  tmp.data[0] = AFECommand_debug_machine_control;
	  enqueueSubdeviceStatus (&tmp, rptr->subdevice);
	}
      rptr->ramp_target_voltage_set_bits_old = new_target_dac_bits;
#endif // DEBUG_SEND_BY_CAN_MACHINE_CONTROL
    }
}

static inline void __attribute__((always_inline, optimize("-O3")))
process_dac_ramping (s_regulatorSettings *rptr, uint32_t timestamp_ms)
{
#if HARDWARE_CONTROL_TEMPERATURE_LOOP_RAMP_BIT_DISABLED
  // DAC ramping is disabled, do nothing.
  return;
#else // HARDWARE_CONTROL_TEMPERATURE_LOOP_RAMP_BIT_DISABLED
  if (rptr->ramp_enabled == 0)
    {
      return;
    }
  rptr->ramp_curent_voltage_set_bits = (uint16_t) HAL_DAC_GetValue ( // Get current DAC value
      &hdac, rptr->subdevice == AFECommandSubdevice_master ? DAC_CHANNEL_1 : DAC_CHANNEL_2);
  if ((timestamp_ms - rptr->ramp_bit_step_timestamp_old_ms) >= rptr->ramp_bit_step_every_ms)
    {
      uint16_t d_bit = rptr->ramp_bit_step;
      uint16_t set_to_bit = 0;
#if USE_SMALLER_STEPS_NEAR_DAC_TARGET
      if (abs (
	  (int32_t) rptr->ramp_curent_voltage_set_bits
	      - (int32_t) rptr->ramp_target_voltage_set_bits) < (rptr->ramp_bit_step))
	{
	  d_bit = rptr->ramp_bit_step / 10;
	}
      if (d_bit < 1)
	{
	  d_bit = 1;
	}
#endif // USE_SMALLER_STEPS_NEAR_DAC_TARGET
      rptr->ramp_bit_step_timestamp_old_ms = timestamp_ms;
      if (rptr->ramp_curent_voltage_set_bits != rptr->ramp_target_voltage_set_bits)
	{
	  rptr->ramp_target_reached = 0;
	  if (rptr->ramp_curent_voltage_set_bits < rptr->ramp_target_voltage_set_bits)
	    {
	      if (((int32_t) rptr->ramp_curent_voltage_set_bits + (int32_t) d_bit)
		  >= (int32_t) rptr->ramp_target_voltage_set_bits)
		{
		  set_to_bit = rptr->ramp_target_voltage_set_bits;
		}
	      else
		{
		  set_to_bit = rptr->ramp_curent_voltage_set_bits + d_bit;
		}
	    }
	  else
	    {
	      if (((int32_t) rptr->ramp_curent_voltage_set_bits - (int32_t) d_bit)
		  <= (int32_t) rptr->ramp_target_voltage_set_bits)
		{
		  set_to_bit = rptr->ramp_target_voltage_set_bits;
		}
	      else
		{
		  set_to_bit = rptr->ramp_curent_voltage_set_bits - d_bit;
		}
	    }

	  if (rptr->ramp_curent_voltage_set_bits >= AFE_DAC_MAX)
	    {
	      rptr->ramp_curent_voltage_set_bits = AFE_DAC_MAX;
	    }
#if TEMPERATURE_LOOP_HARDWARE_CONTROL_DAC_DISABLED
          // DAC hardware control is disabled, do nothing.
#else // TEMPERATURE_LOOP_HARDWARE_CONTROL_DAC_DISABLED
	  machine_DAC_set (rptr, set_to_bit);
#endif // TEMPERATURE_LOOP_HARDWARE_CONTROL_DAC_DISABLED
	}
      else
	{
#if TEMPERATURE_LOOP_CORRECT_BY_READING_ENABLED
	  float vdiff = fabsf (rptr->V - rptr->V_target);
	  uint16_t d = (uint16_t) roundf (fabsf (rptr->a_dac * vdiff));

	  if (vdiff > 0.1)
	    {
	      if (rptr->V > rptr->V_target)
		{
		  rptr->ramp_target_voltage_set_bits += d; /* reversed logic */
		}
	      else
		{
		  rptr->ramp_target_voltage_set_bits -= d;
		}
	    }
#endif // TEMPERATURE_LOOP_CORRECT_BY_READING_ENABLED
	  if (rptr->ramp_target_reached == 0)
	    {
	      rptr->ramp_target_reached = 1;
	      CAN_Message_t tmp; // Local message for debug
	      tmp.id = CAN_ID_IN_MSG;
	      tmp.timestamp = HAL_GetTick (); // for timeout
	      tmp.data[0] = AFECommand_debug_machine_control;
	      enqueueSubdeviceStatus (&tmp, rptr->subdevice);
	    }
	}
    }
#endif // HARDWARE_CONTROL_TEMPERATURE_LOOP_RAMP_BIT_DISABLED
}

void
enqueueSubdeviceStatus (CAN_Message_t *reply, uint8_t masked_channel)
{
  const uint8_t msg_count_per_subdev = 12; // Should to be < 0x0F, to send more than 15 msgs
  uint8_t total_msg_count = msg_count_per_subdev;
  if (AFECommandSubdevice_both == (masked_channel & AFECommandSubdevice_both))
    {
      total_msg_count = 0x0F; // Keep above msg_count_per_subdev
    }
  for (uint8_t i = 0; i < 2; ++i)
    {
      if (0x01 & (masked_channel >> i))
	{
	  uint8_t cnt = 0;
	  uint8_t subdev = 1 << i;
	  s_regulatorSettings *rs = &afe_regulatorSettings[i];
	  s_ADC_Measurement tmp_adc;
	  get_n_latest_from_buffer (rs->voltage_channelSettings_ptr->buffer_ADC, 1, &tmp_adc);
	  rs->V = faxplusbcs ((float) tmp_adc.adc_value, rs->voltage_channelSettings_ptr);
	  // 0. Voltage
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, cnt++, total_msg_count,
						       subdev, &rs->V);
	  // 1. Voltage in bytes
	  float tmp_float = tmp_adc.adc_value;
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, cnt++, total_msg_count,
						       subdev, &tmp_float);
	  // 2. Ramp Target Voltage
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, cnt++, total_msg_count,
						       subdev, &rs->V_target);
	  // 3. Ramp Target Voltage in bytes
	  tmp_float = rs->ramp_target_voltage_set_bits;
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, cnt++, total_msg_count,
						       subdev, &tmp_float);
	  // 4. Ramp Current Voltage in bytes
	  tmp_float = rs->ramp_curent_voltage_set_bits;
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, cnt++, total_msg_count,
						       subdev, &tmp_float);
	  // 5. Average temperature
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, cnt++, total_msg_count,
						       subdev, &rs->T);
	  // 6. Last temperature in bytes
	  get_n_latest_from_buffer (rs->temperature_channelSettings_ptr->buffer_ADC, 1, &tmp_adc);
	  tmp_float = tmp_adc.adc_value;
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, cnt++, total_msg_count,
						       subdev, &tmp_float);
	  // 7. Old temperature
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, cnt++, total_msg_count,
						       subdev, &rs->T_old);
	  // 8. V Offset
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, cnt++, total_msg_count,
						       subdev, &rs->V_offset);
	  // 9. Enabled?
	  CANCircularBuffer_enqueueMessage_data (&canTxBuffer, reply, cnt++, total_msg_count, subdev,
						 (uint8_t*) &rs->enabled, 1);
	  // 10. Ramp target reached?
	  CANCircularBuffer_enqueueMessage_data (&canTxBuffer, reply, cnt++, total_msg_count, subdev,
						 (uint8_t*) &rs->ramp_target_reached, 1);
	  // 11. Timestamp
	  CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, reply, cnt++, total_msg_count,
							 subdev,
							 rs->ramp_bit_step_timestamp_old_ms);

	  total_msg_count = msg_count_per_subdev; // Reset counter (works only if subdevs <= 2)
	}
    }
}

/*
 * =================================================================================
 * CAN Command Handlers
 * =================================================================================
 * These static functions are called from can_execute() to handle specific
 * CAN commands, making the main dispatcher more modular and readable.
 */

/* --- System & Informational Command Handlers --- */

X_MEDIUM_HANDLE_FUNCTION_DECLARE
handle_getSubdeviceStatus (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  reply->id = CAN_ID_IN_MSG;
  reply->timestamp = HAL_GetTick (); // for timeout
  uint8_t masked_channel = msg->Data[2] & AFECommandSubdevice_both;
  enqueueSubdeviceStatus (reply, masked_channel);
}

X_SLOW_HANDLE_FUNCTION_DECLARE
handle_getSerialNumber (CAN_Message_t *reply)
{
  reply->id |= e_CANIdFunctionCode_multipleRead & 0b11;
  reply->dlc = 2 + 4;
  for (uint8_t i0 = 0; i0 < 3; ++i0)
    {
      reply->data[1] = get_byte_of_message_number (i0, 3);
      memcpy (&reply->data[2], (uint8_t*) &UID[i0], sizeof(uint32_t));
      CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
    }
}

X_SLOW_HANDLE_FUNCTION_DECLARE
handle_getVersion (CAN_Message_t *reply)
{
  reply->dlc = 2 + verArrLen;
  memcpy (&reply->data[2], &verArr[0], verArrLen);
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

#if AFE_ADC_HARD_BY_TIMER
static HAL_StatusTypeDef
machine_set_tim_period_ms (TIM_HandleTypeDef *htim, uint32_t period_ms)
{
  if (period_ms == 0)
    {
      return HAL_ERROR;
    }

  const uint32_t tim_clk = 48000000; // From SystemClock_Config()
  // Calculate the total number of timer clock cycles for the desired period.
  const uint64_t total_cycles = (uint64_t) period_ms * (tim_clk / 1000);

  // The product of (PSC+1) and (ARR+1) must be <= 2^32
  if (total_cycles > 0x100000000ULL)
    {
      return HAL_ERROR; // Period is too long to configure.
    }

  // Strategy 1: Attempt to use a prescaler that gives a 1 MHz counter clock (1 us resolution).
  // This is ideal for common, shorter periods.
  uint32_t psc_candidate = (tim_clk / 1000000) - 1; // Should be 47 for 48MHz clock
  if (psc_candidate < 65536)
    {
      uint64_t period_candidate_plus_1 = total_cycles / (psc_candidate + 1);
      if ((period_candidate_plus_1 > 0) && (period_candidate_plus_1 <= 65536)
	  && (total_cycles % (psc_candidate + 1) == 0))
	{
	  htim->Instance->PSC = psc_candidate;
	  htim->Instance->ARR = period_candidate_plus_1 - 1;
	  return HAL_OK;
	}
    }

  // Strategy 2: If the first strategy fails, search for the smallest valid prescaler
  // to maximize the period register for best resolution.
  uint32_t psc_plus_1 = (total_cycles + 65535) / 65536; // ceiling division
  if (psc_plus_1 == 0)
    {
      psc_plus_1 = 1;
    }

  while (((total_cycles % psc_plus_1) != 0) && (psc_plus_1 <= 65536))
    {
      psc_plus_1++;
    }

  if (psc_plus_1 <= 65536)
    {
      htim->Instance->PSC = psc_plus_1 - 1;
      htim->Instance->ARR = (total_cycles / psc_plus_1) - 1;
      return HAL_OK;
    }

  return HAL_ERROR; // No suitable combination found
}
#endif // AFE_ADC_HARD_BY_TIMER

X_FAST_HANDLE_FUNCTION_DECLARE
handle_startADC (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  uint32_t ms = 0;
  memcpy ((uint8_t*) &ms, &msg->Data[3], sizeof(uint32_t));
#if AFE_ADC_SOFT_LAUNCHED
  afe_adc_soft_active = 1;
  afe_adc_soft_period_ms = ms;
  afe_adc_soft_timestamp_ms = HAL_GetTick ();
#else // AFE_ADC_SOFT_LAUNCHED
  HAL_ADC_Stop_DMA (&hadc); // Important: Stop before restart
  HAL_ADC_Start_DMA (&hadc, &adc_dma_buffer[0], AFE_NUMBER_OF_CHANNELS);
#if AFE_ADC_HARD_BY_TIMER
  machine_set_tim_period_ms (&htim1, ms);
  HAL_TIM_Base_Start (&htim1);
#endif // AFE_ADC_HARD_BY_TIMER
#endif // AFE_ADC_SOFT_LAUNCHED
  memcpy (&reply->data[0], &msg->Data[0], msg->DLC);
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

/* --- Data Acquisition Command Handlers --- */

X_FAST_HANDLE_FUNCTION_DECLARE
handle_getSensorDataSi_last_byMask (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  uint8_t channels = msg->Data[2];
  s_ADC_Measurement adc_val;
  float adc_value_real;
  uint8_t total_msg_count = get_number_of_channels (channels) + 1; // Channels + timestamp
  uint8_t msg_index = 0;
  uint8_t forThisChannels = 0x00;
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
    {
      if (channels & (1 << channel)) // loop over channel mask
	{
	  get_n_latest_from_buffer (afe_channelSettings[channel].buffer_ADC, 1, &adc_val);
	  adc_value_real = faxplusbcs (adc_val.adc_value, &afe_channelSettings[channel]);
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, msg_index,
						       total_msg_count, 1 << channel,
						       &adc_value_real);
	  forThisChannels |= (1 << channel);
	  ++msg_index;
	}
    }
  CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, reply, msg_index, total_msg_count,
						 forThisChannels, reply->timestamp);
}

X_FAST_HANDLE_FUNCTION_DECLARE
handle_getSensorDataSi_average_byMask (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  uint8_t channels = msg->Data[2];
  float adc_value_real;
  uint8_t total_msg_count = get_number_of_channels (channels) + 1;
  uint8_t msg_index = 0;
  uint8_t forThisChannels = 0x00;
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
    {
      if (channels & (1 << channel)) // loop over channel mask
	{
	  adc_value_real = get_average_atSettings (&afe_channelSettings[channel], reply->timestamp);
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, msg_index,
						       total_msg_count, 1 << channel,
						       &adc_value_real);
	  forThisChannels |= (1 << channel);
	  ++msg_index;
	}
    }
  CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, reply, msg_index, total_msg_count,
						 forThisChannels, reply->timestamp);
}

X_FAST_HANDLE_FUNCTION_DECLARE
handle_getSensorDataBytes_last_byMask (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  uint8_t channels = msg->Data[2];
  s_ADC_Measurement adc_val;
  float adc_value_real;
  uint8_t total_msg_count = get_number_of_channels (channels) + 1; // Channels + timestamp
  uint8_t msg_index = 0;
  uint8_t forThisChannels = 0x00;
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
    {
      if (channels & (1 << channel)) // loop over channel mask
	{
	  get_n_latest_from_buffer (afe_channelSettings[channel].buffer_ADC, 1, &adc_val);
	  adc_value_real = adc_val.adc_value; // Send bytes as float
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, msg_index,
						       total_msg_count, 1 << channel,
						       &adc_value_real);
	  forThisChannels |= (1 << channel);
	  ++msg_index;
	}
    }
  CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, reply, msg_index, total_msg_count,
						 forThisChannels, reply->timestamp);
}

X_FAST_HANDLE_FUNCTION_DECLARE
handle_getSensorDataBytes_average_byMask (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  uint8_t channels = msg->Data[2];
  float adc_value_real;
  uint8_t total_msg_count = get_number_of_channels (channels) + 1;
  uint8_t msg_index = 0;
  uint8_t forThisChannels = 0x00;
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
    {
      if (channels & (1 << channel)) // loop over channel mask
	{
	  s_channelSettings *a = &afe_channelSettings[channel];
	  adc_value_real = get_average_from_buffer (a->buffer_ADC, a->max_N, reply->timestamp,
						    a->max_dt_ms, a->averaging_method, a->alpha);
	  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, reply, msg_index,
						       total_msg_count, 1 << channel,
						       &adc_value_real);
	  forThisChannels |= (1 << channel);
	  ++msg_index;
	}
    }
  CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, reply, msg_index, total_msg_count,
						 forThisChannels, reply->timestamp);
}
/* --- Hardware Control Command Handlers --- */

X_SLOW_HANDLE_FUNCTION_DECLARE
handle_transmitSPIData (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  uint8_t spiData[5];
  uint8_t spiDataLen = msg->Data[2];
  reply->dlc = 3;
  if (spiDataLen > 5)
    {
      reply->data[2] = HAL_ERROR;
      CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
      return;
    }
  memcpy (&spiData[0], &msg->Data[3], spiDataLen);
  reply->data[2] = machine_SPI_Transmit (&hspi1, &spiData[0], spiDataLen, TIMEOUT_SPI1_MS);
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

X_SLOW_HANDLE_FUNCTION_DECLARE
handle_setAD8402Value (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  uint8_t channels = msg->Data[2];
  uint8_t value = msg->Data[3];
  reply->data[2] = channels;
  reply->data[3] = value;
  reply->data[4] = 0x00; // masked error status
  reply->dlc = 5;
  for (uint8_t channel = 0; channel < NUMBER_OF_AD8402_CHANNELS; ++channel)
    {
      if (channels & (1 << channel))
	{
	  reply->data[4] |= (uint8_t) (
	      AD8402_Write (&hspi1, channel, value, TIMEOUT_SPI1_MS) == HAL_OK ? 0 : 1) << channel;
	}
    }
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

X_SLOW_HANDLE_FUNCTION_DECLARE
handle_writeGPIO (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  GPIO_TypeDef *GPIOx = GetGPIOPortByEnumerator (msg->Data[2]);
  uint32_t GPIO_Pin = 1 << msg->Data[3]; // GPbIO Pin mask
  uint8_t PinState = msg->Data[4];
  machine_GPIO_WritePin (GPIOx, (uint16_t) GPIO_Pin, PinState);
  reply->data[2] = msg->Data[2];
  reply->data[3] = msg->Data[3];
  reply->data[4] = msg->Data[4];
  reply->dlc = 5;
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

/* --- Temperature Loop & DAC Command Handlers --- */

X_FAST_HANDLE_FUNCTION_DECLARE
handle_set_temperature_loop_state (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  uint8_t channels = msg->Data[2];
  uint8_t status = msg->Data[3];
  reply->data[2] = channels;
  reply->data[3] = status;
  reply->dlc = 4;
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
    {
      if (channels & (1 << channel))
	{
	  afe_regulatorSettings[channel].enabled = (status != 0);
	}
    }
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

X_FAST_HANDLE_FUNCTION_DECLARE
handle_setDACValueRaw_bySubdeviceMask (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  uint8_t channels = msg->Data[2];
  uint16_t value = 0;
  reply->data[2] = msg->Data[2];
  memcpy (&reply->data[3], &msg->Data[3], sizeof(uint16_t));
  reply->dlc = 3 + sizeof(uint16_t);
  memcpy (&value, &msg->Data[3], sizeof(uint16_t));
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
    {
      if (channels & (1 << channel))
	{
	  machine_DAC_set (&afe_regulatorSettings[channel], value);
	}
    }
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

X_FAST_HANDLE_FUNCTION_DECLARE
handle_setDACValueSi_bySubdeviceMask (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  float valueSi;
  uint8_t subdevice_mask = msg->Data[2];
  memcpy (&valueSi, &msg->Data[3], sizeof(float)); /* Copy SI value */
  memcpy (&reply->data[0], &msg->Data[0], msg->DLC); /* Return the same msg */
  reply->dlc = msg->DLC;

  for (uint8_t subdev = 0; subdev < AFE_NUMBER_OF_SUBDEVICES; ++subdev)
    {
      if (subdevice_mask & (1 << subdev))
	{
	  uint16_t value = machine_DAC_convert_V_to_DAC_value (valueSi,
							       &afe_regulatorSettings[subdev]);
	  machine_DAC_set (&afe_regulatorSettings[subdev], value);
	}
    }
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

X_FAST_HANDLE_FUNCTION_DECLARE
handle_setDACTargetSi_bySubdeviceMask (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  float valueSi;
  uint8_t subdevice_mask = msg->Data[2];
  memcpy (&valueSi, &msg->Data[3], sizeof(float)); /* Copy SI value */
  memcpy (&reply->data[0], &msg->Data[0], msg->DLC); /* Return the same msg */
  reply->dlc = msg->DLC;

  for (uint8_t subdev = 0; subdev < AFE_NUMBER_OF_SUBDEVICES; ++subdev)
    {
      if (subdevice_mask & (1 << subdev))
	{
	  afe_regulatorSettings[subdev].V_target = valueSi;
	  afe_regulatorSettings[subdev].ramp_target_voltage_set_bits =
	      machine_DAC_convert_V_to_DAC_value (valueSi, &afe_regulatorSettings[subdev]);
	}
    }
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

X_FAST_HANDLE_FUNCTION_DECLARE
handle_stopTemperatureLoopForAllChannels (CAN_Message_t *reply)
{
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
    {
      afe_regulatorSettings[channel].enabled = 0;
    }
  reply->data[2] = 1; // Success status
  reply->dlc = 3;
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

X_FAST_HANDLE_FUNCTION_DECLARE
handle_setDAC_bySubdeviceMask (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  uint8_t channel_mask = msg->Data[2];
  uint8_t value = msg->Data[3];
  reply->data[2] = channel_mask; // Copy mask
  reply->data[3] = value;	 // Copy value
  reply->data[4] = 0x00;	 // Clear error status mask
  reply->dlc = 5;

  if (value > 1) // Only 0 (stop) or 1 (start) are valid
    {
      return; // Invalid parameter, do nothing.
    }

  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
    {
      if (channel_mask & (1 << channel))
	{
	  reply->data[4] |= machine_DAC_switch (&afe_regulatorSettings[channel], value) << channel;
	}
    }
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

X_FAST_HANDLE_FUNCTION_DECLARE
handle_setDACRampOneBytePerMillisecond (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
#warning "Implement and test this"
  // When implemented, this handler will process the command.
  // For now, it does nothing, matching the original behavior.
  (void) msg;
  (void) reply;
}

/* --- Channel & Regulator Settings Handlers --- */

// Generic handler for setting a property on multiple channels based on a mask
X_SLOW_HANDLE_FUNCTION_DECLARE
handle_set_channel_property (const s_can_msg_recieved *msg, CAN_Message_t *reply, size_t offset,
			     size_t size, bool reset_buffer)
{
  uint8_t channelMask = msg->Data[2];
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
    {
      if (channelMask & (1 << channel))
	{
	  uint8_t *dst = (uint8_t*) &afe_channelSettings[channel];
	  memcpy (dst + offset, &msg->Data[3], size);
	  if (reset_buffer)
	    {
	      update_buffer_by_channelSettings (&afe_channelSettings[channel]);
	    }
	}
    }
  // Echo the command back as confirmation
  memcpy (&reply->data[0], &msg->Data[0], msg->DLC);
  reply->dlc = msg->DLC;
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

// Generic handler for setting a property on subdevices (regulators)
X_SLOW_HANDLE_FUNCTION_DECLARE
handle_set_regulator_property (const s_can_msg_recieved *msg, CAN_Message_t *reply, size_t offset,
			       size_t size)
{
  uint8_t subdevMask = msg->Data[2];
  for (uint8_t subdev = 0; subdev < AFE_NUMBER_OF_SUBDEVICES; ++subdev)
    {
      if (subdevMask & (1 << subdev))
	{
	  uint8_t *dst = (uint8_t*) &afe_regulatorSettings[subdev];
	  memcpy (dst + offset, &msg->Data[3], size);
	}
    }
  // Echo the command back as confirmation
  memcpy (&reply->data[0], &msg->Data[0], msg->DLC);
  reply->dlc = msg->DLC;
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

/* --- Unknown Command Handler --- */

X_SLOW_HANDLE_FUNCTION_DECLARE
handle_unknown_command (CAN_Message_t *reply)
{
  reply->data[0] = 0xFF;
  reply->data[1] = 0x34;
  reply->data[2] = 0x56;
  reply->data[3] = 0x78;
  reply->data[4] = 0x90;
  reply->data[5] = 0xAB;
  reply->data[6] = 0xCD;
  reply->data[7] = 0xEF;
  reply->dlc = 8;
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

/***
 */
static uint32_t value0;
void __attribute__ ((optimize("-Os")))
can_execute (const s_can_msg_recieved msg)
{
  CAN_Message_t tmp;
  uint8_t command = (uint8_t) msg.Data[0];
  uint16_t msg_id = CAN_ID_IN_MSG;
  tmp.timestamp = HAL_GetTick ();
  tmp.id = msg_id;
  tmp.data[0] = command; // Standard reply [function]
  tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
  tmp.dlc = 8; // Default DLC
  switch ((AFECommand) command)
    {
    case AFECommand_getSerialNumber: // getSerialNumber
      {
	handle_getSerialNumber (&tmp);
	break;
      }
    case AFECommand_getVersion: // getVersion
      {
	handle_getVersion (&tmp);
	break;
      }
    case 0x02: // setValue0
      {
	tmp.dlc = 3;
	value0 = *((uint32_t*) &msg.Data[2]);
	tmp.data[2] = 1;
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_resetAll: // resetAll
      {
	NVIC_SystemReset ();
	break;
      }

    case AFECommand_startADC:
      {
	handle_startADC (&msg, &tmp);
	break;
      }

    case AFECommand_getTimestamp:
      {
	CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, 0, 1, 0x00,
						       HAL_GetTick ());
	break;
      }

    case AFECommand_getSyncTimestamp:
      {
	/* timestamp */
	CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, 0, 2, 0x00,
						       HAL_GetTick ());
	/* recieved timestamp */
	CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, 1, 2, 0x00,
						       msg.timestamp);
	break;
      }

    case AFECommand_getSubdeviceStatus:
      {
	handle_getSubdeviceStatus (&msg, &tmp);
	break;
      }

      /**** 0x30 ****/
      /* Send SI data [Data[0]] from ADC channel [Data[2]] as float value[Data[3:7]]  */
    case AFECommand_getSensorDataSi_last_byMask:
      {
	handle_getSensorDataSi_last_byMask (&msg, &tmp);
	break;
      }
      /* Send SI data [Data[0]] from ADC channel [Data[2]] as float average value[Data[3:7]]  */
    case AFECommand_getSensorDataSi_average_byMask:
      {
	handle_getSensorDataBytes_average_byMask (&msg, &tmp);
	break;
      }
    case AFECommand_getSensorDataBytes_last_byMask:
      {
	handle_getSensorDataBytes_last_byMask (&msg, &tmp);
	break;
      }
    case AFECommand_getSensorDataBytes_average_byMask:
      {
	handle_getSensorDataSi_average_byMask (&msg, &tmp);
	break;
      }
    case AFECommand_getSensorDataSi_periodic:
      {
#warning "Update this" // TODO Update this
	break;
      }

      /**** 0x40 set periodic report ****/
    case AFECommand_setSensorDataSi_periodic_last:
      {

	break;
      }

      /* 0xA0 */
    case AFECommand_transmitSPIData: // Send data[2] bytes of data[3] by SPI1; return data[2] as error code
      {
	handle_transmitSPIData (&msg, &tmp);
#warning "Test SPI!" // TODO Test SPI
	break;
      }
    case AFECommand_setAD8402Value_byte_byMask:
      {
	handle_setAD8402Value (&msg, &tmp);
	break;
      }
    case AFECommand_writeGPIO:
      {
	handle_writeGPIO (&msg, &tmp);
	break;
      }

    case AFECommand_setCanMsgBurstDelay_ms:
      {
	memcpy (&canMsgBurstDelay_ms, &msg.Data[3], sizeof(uint32_t));
	CANCircularBuffer_enqueueMessage_data (&canTxBuffer, &tmp, 0, 1, 0x00,
					       (uint8_t*) &canMsgBurstDelay_ms, sizeof(uint32_t));
	break;
      }

    case AFECommand_setAfe_can_watchdog_timeout_ms:
      {
	memcpy (&afe_can_watchdog_timeout_ms, &msg.Data[3], sizeof(uint32_t));
	CANCircularBuffer_enqueueMessage_data (&canTxBuffer, &tmp, 0, 1, 0x00,
					       (uint8_t*) &afe_can_watchdog_timeout_ms,
					       sizeof(uint32_t));
	break;
      }

    case AFECommand_setTemperatureLoop_loop_every_ms:
      {
	handle_set_regulator_property (&msg, &tmp, offsetof(s_regulatorSettings, loop_every_ms),
				       sizeof(uint32_t));
	break;
      }

      /* 0xC0 */
      /* Temperature loop runtime */
    case AFECommand_setTemperatureLoopForChannelState_byMask_asStatus: // start or stop [Data[3] temperature loop for channel [Data[2]]
      {
	handle_set_temperature_loop_state (&msg, &tmp);
	break;
      }
    case AFECommand_setDACValueRaw_bySubdeviceMask: // set [Data[3] temperature loop for channel [Data[2]]
      {
	handle_setDACValueRaw_bySubdeviceMask (&msg, &tmp);
	break;
      }
    case AFECommand_setDACValueSi_bySubdeviceMask: // set [Data[3] temperature loop for channel [Data[2]]
      {
	handle_setDACValueSi_bySubdeviceMask (&msg, &tmp);
	break;
      }
    case AFECommand_stopTemperatureLoopForAllChannels: // stop temperature loop for all channels
      {
	handle_stopTemperatureLoopForAllChannels (&tmp);
	break;
      }
    case AFECommand_setDAC_bySubdeviceMask: // Start or stop DAC
      {
	handle_setDAC_bySubdeviceMask (&msg, &tmp);
	break;
      }
    case AFECommand_setDACRampOneBytePerMillisecond_ms:
      {
	handle_setDACRampOneBytePerMillisecond (&msg, &tmp);
	break;
      }
    case AFECommand_setDACTargetSi_bySubdeviceMask:
      {
	handle_setDACTargetSi_bySubdeviceMask (&msg, &tmp);
	break;
      }

      /*********** 0xD0 ************/
      /* Set setting [Data[0]] for adc channel [Data[2]] by uint32_t value[Data[3:7]]  */
    case AFECommand_setAveragingMode_byMask: // set averagingSettings.averaging_method
      {
	handle_set_channel_property (&msg, &tmp, offsetof(s_channelSettings, averaging_method),
				     sizeof(e_average), true);
	break;
      }
    case AFECommand_setAveragingAlpha_byMask: // set averagingSettings.alpha
      {
	handle_set_channel_property (&msg, &tmp, offsetof(s_channelSettings, alpha), sizeof(float),
	true);
	break;
      }
    case AFECommand_setAveragingBufferSize_byMask: // set averagingSettings.buffer_size
      {
	handle_set_channel_property (&msg, &tmp,
	offsetof(s_channelSettings, buffer_ADC) + offsetof(s_BufferADC, buffer_size),
				     sizeof(uint32_t), true);
	break;
      }
    case AFECommand_setChannel_dt_ms_byMask:
      {
	handle_set_channel_property (
	    &msg, &tmp, offsetof(s_channelSettings, buffer_ADC) + offsetof(s_BufferADC, dt_ms),
	    sizeof(uint32_t), true);
	break;
      }
    case AFECommand_setAveraging_max_dt_ms_byMask: // set averagingSettings.max_dt_ms
      {
	handle_set_channel_property (&msg, &tmp, offsetof(s_channelSettings, max_dt_ms),
				     sizeof(uint32_t), true);
	break;
      }
    case AFECommand_setAveragingSubdevice: // set averagingSettings.subdevice
      {
	Error_Handler(); // This command seems to be unimplemented.
	break;
      }
    case AFECommand_setChannel_a_byMask:
      {
	handle_set_channel_property (&msg, &tmp, offsetof(s_channelSettings, a), sizeof(float),
	true);
	break;
      }
    case AFECommand_setChannel_b_byMask:
      {
	handle_set_channel_property (&msg, &tmp, offsetof(s_channelSettings, b), sizeof(float),
	true);
	break;
      }
    case AFECommand_setChannel_period_ms_byMask:
      {
	// Note: period_ms does not require a buffer reset.
	handle_set_channel_property (&msg, &tmp, offsetof(s_channelSettings, period_ms),
				     sizeof(uint32_t), false);
	break;
      }
    case AFECommand_setChannelBufferSize:
      {
	// This command is functionally identical to AFECommand_setAveragingBufferSize_byMask
	handle_set_channel_property (&msg, &tmp,
	offsetof(s_channelSettings, buffer_ADC) + offsetof(s_BufferADC, buffer_size),
				     sizeof(uint32_t), true);
	break;
      }

    case AFECommand_setRegulator_ramp_enabled_byMask:
      {
	handle_set_regulator_property (&msg, &tmp, offsetof(s_regulatorSettings, ramp_enabled),
				       sizeof(int8_t));
	break;
      }

    case AFECommand_setRegulator_T_opt_byMask:
      {
	handle_set_regulator_property (&msg, &tmp, offsetof(s_regulatorSettings, T_opt),
				       sizeof(float));
	break;
      }
    case AFECommand_setRegulator_dT_byMask:
      {
	handle_set_regulator_property (&msg, &tmp, offsetof(s_regulatorSettings, dT),
				       sizeof(float));
	break;
      }
    case AFECommand_setRegulator_a_dac_byMask:
      {
	handle_set_regulator_property (&msg, &tmp, offsetof(s_regulatorSettings, a_dac),
				       sizeof(float));
	break;
      }
    case AFECommand_setRegulator_b_dac_byMask:
      {
	handle_set_regulator_property (&msg, &tmp, offsetof(s_regulatorSettings, b_dac),
				       sizeof(float));
	break;
      }
    case AFECommand_setRegulator_dV_dT_byMask:
      {
	handle_set_regulator_property (&msg, &tmp, offsetof(s_regulatorSettings, dV_dT),
				       sizeof(float));
	break;
      }
    case AFECommand_setRegulator_V_opt_byMask:
      {
	handle_set_regulator_property (&msg, &tmp, offsetof(s_regulatorSettings, V_opt),
				       sizeof(float));
	break;
      }
    case AFECommand_setRegulator_V_offset_byMask:
      {
	handle_set_regulator_property (&msg, &tmp, offsetof(s_regulatorSettings, V_offset),
				       sizeof(float));
	break;
      }
    case 0xF7:
      {
	uint16_t randomExample[AFE_NUMBER_OF_CHANNELS] =
	  { 1, 1, 1, 1, 1, 1, 1, 1 };
	uint8_t channels = msg.Data[2];
	s_ADC_Measurement adc_val;
	float adc_value_real;
	uint8_t total_msg_count = get_number_of_channels (channels);
	uint8_t msg_index = 0;
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channels & (1 << channel)) // loop over channel mask
	      {
		adc_val.adc_value = randomExample[channel];
		adc_value_real = faxplusbcs (adc_val.adc_value, &afe_channelSettings[channel]);
		CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, msg_index,
							     total_msg_count, 1 << channel,
							     &adc_value_real);
		++msg_index;
	      }
	  }
	break;
      }
    case 0xF8:
      {
	machine_GPIO_WritePin (GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	machine_GPIO_WritePin (GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2000);
	HAL_DAC_Start (&hdac, DAC_CHANNEL_1);
	break;
      }
    case 0xF9:
      {
	uint8_t channels = 0xFF;
	s_ADC_Measurement adc_val;
	float adc_value_real;
	uint8_t total_msg_count = get_number_of_channels (channels) + 1; // Channels + timestamp
	uint8_t msg_index = 0;
	uint8_t forThisChannels = 0x00;
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channels & (1 << channel)) // loop over channel mask
	      {
		HAL_ADC_Start (&hadc);
		HAL_ADC_PollForConversion (&hadc, 1000);
		adc_val.adc_value = (uint16_t) HAL_ADC_GetValue (&hadc);
		adc_value_real = faxplusbcs (adc_val.adc_value, &afe_channelSettings[channel]);
		CANCircularBuffer_enqueueMessage_data (&canTxBuffer, &tmp, msg_index,
						       total_msg_count, 1 << channel,
						       (uint8_t*) &adc_value_real, sizeof(float));
		forThisChannels |= (1 << channel);
		++msg_index;
	      }
	  }
	CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, msg_index,
						       total_msg_count, forThisChannels,
						       tmp.timestamp);
	break;
      }
    case AFECommand_clearRegulator_T_old:
      {
	handle_set_regulator_property (&msg, &tmp, offsetof(s_regulatorSettings, T_old),
				       sizeof(float));
	break;
      }
    default:
      {
	handle_unknown_command (&tmp);
	break;
      }
    }
}

void __attribute__ ((optimize("-O3")))
machine_control (void)
{
  for (uint8_t subdev = 0; subdev < AFE_NUMBER_OF_SUBDEVICES; ++subdev)
    {
      s_regulatorSettings *regulatorSettings_ptr = &afe_regulatorSettings[subdev];
      uint32_t timestamp_ms = HAL_GetTick ();

      process_temperature_loop (regulatorSettings_ptr, timestamp_ms);
      process_dac_ramping (regulatorSettings_ptr, timestamp_ms);
    }
}

static void __attribute__ ((optimize("-O3")))
machine_periodic_report (void)
{
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
    {
      s_channelSettings *ptr = &afe_channelSettings[channel];
      uint32_t timestamp = HAL_GetTick ();
      if (ptr->period_ms > 0)
	{
	  if ((timestamp - ptr->period_ms_last) >= ptr->period_ms)
	    {
	      float adc_value_real = 0.0;
	      CAN_Message_t tmp;
	      tmp.id = CAN_ID_IN_MSG;
	      tmp.data[0] = AFECommand_getSensorDataSi_periodic;
	      tmp.timestamp = timestamp;
	      uint8_t channel_mask = 1 << channel;
	      s_ADC_Measurement adc_val;
	      const uint8_t total_msg_count = 4;

	      /* Get last data */
	      if (get_n_latest_from_buffer (afe_channelSettings[channel].buffer_ADC, 1, &adc_val))
		{
		  /* Value in SI */
		  adc_value_real = faxplusbcs (adc_val.adc_value, &afe_channelSettings[channel]);
		  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, 0,
							       total_msg_count, channel_mask,
							       &adc_value_real);
		  /* Value in ADC */
		  adc_value_real = adc_val.adc_value;
		  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, 1,
							       total_msg_count, channel_mask,
							       &adc_value_real);
		  /* Add last data timestamp */
		  CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, 2,
								 total_msg_count, channel_mask,
								 adc_val.timestamp_ms);
		  /* Get average data */
		  adc_value_real = get_average_atSettings (ptr, timestamp);
		  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, 3,
							       total_msg_count, channel_mask,
							       &adc_value_real);
		  /* Add calculation timestamp */
		  CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, 4,
								 total_msg_count, channel_mask,
								 timestamp);

		  ptr->period_ms_last = timestamp;
		}
	    }
	}
    }
}

#if MACHINE_DEBUG_RUN
static void
machine_debug_run (void)
{
#if WATCHDOG_FOR_CAN_RECIEVER_ENABLED
  afe_can_watchdog_timestamp_ms = HAL_GetTick();
#endif // WATCHDOG_FOR_CAN_RECIEVER_ENABLED

}
#endif

#if AFE_ADC_SOFT_LAUNCHED
static inline void __attribute__((always_inline, optimize("-O3")))
machine_adc_pool (void)
{
  if (afe_adc_soft_active)
    {
#if AFE_ADC_ISR
	    if (afe_adc_i == AFE_NUMBER_OF_CHANNELS)
	      {
		++afe_adc_i;
		HAL_ADC_Stop_IT (&hadc);
		s_ADC_Measurement _adc;
		_adc.timestamp_ms = afe_adc_soft_timestamp_ms;
		for (uint8_t i = 0; i < AFE_NUMBER_OF_CHANNELS; ++i)
		  {
		    _adc.adc_value = adc_dma_buffer[i];
		    add_to_buffer (&bufferADC[i], &_adc);
		  }
	      }
	    if ((HAL_GetTick () - afe_adc_soft_timestamp_ms) >= afe_adc_soft_period_ms)
	      {
		HAL_ADC_Stop_IT (&hadc);
		afe_adc_soft_timestamp_ms = HAL_GetTick ();
		afe_adc_i = 0;
		HAL_ADC_Start_IT (&hadc);
	      }
#else // AFE_ADC_ISR
      if ((HAL_GetTick () - afe_adc_soft_timestamp_ms) >= afe_adc_soft_period_ms)
	{
	  afe_adc_soft_timestamp_ms = HAL_GetTick ();
#if AFE_ADC_SOFT_LAUNCHED_SIMPLE_POOL
	  s_ADC_Measurement _adc;
	  _adc.timestamp_ms = afe_adc_soft_timestamp_ms;

	  for (uint8_t i = 0; i < AFE_NUMBER_OF_CHANNELS; ++i)
	    {
	      if (HAL_ADC_Start (&hadc) != HAL_OK)
		{
		  Error_Handler();
		}
	      if (HAL_ADC_PollForConversion (&hadc, 100) != HAL_OK)
		{
		  Error_Handler();
		}

	      adc_dma_buffer[i] = (uint16_t) HAL_ADC_GetValue (&hadc);
	    }
	  for (uint8_t i = 0; i < AFE_NUMBER_OF_CHANNELS; ++i)
	    {
	      _adc.adc_value = adc_dma_buffer[i];
	      add_to_buffer (&bufferADC[i], &_adc);
	    }
	}
#else // AFE_ADC_SOFT_LAUNCHED_SIMPLE_POOL
		if (afe_adc_soft_started) /* ERROR DETECTED*/
		  {
		    /* TODO Restart conversion queue */
		    HAL_ADC_Stop (&hadc);
		  }
		afe_adc_i = 0;
		afe_adc_soft_started = 1;
		HAL_ADC_Start (&hadc);

	      }
	    if (afe_adc_soft_started)
	      {
		if (HAL_ADC_PollForConversion (&hadc, 0) == HAL_OK)
		  {
		    adc_dma_buffer[afe_adc_i] = (uint16_t) HAL_ADC_GetValue (&hadc);
		    ++afe_adc_i;
		    if (afe_adc_i == AFE_NUMBER_OF_CHANNELS) /* All channels was read */
		      {
			afe_adc_soft_started = 0;
			s_ADC_Measurement _adc;
			_adc.timestamp_ms = afe_adc_soft_timestamp_ms;
			for (uint8_t i = 0; i < AFE_NUMBER_OF_CHANNELS; ++i)
			  {
			    _adc.adc_value = adc_dma_buffer[i];
			    add_to_buffer (&bufferADC[i], &_adc);
			  }
		      }
		    else
		      {
			HAL_ADC_Start (&hadc);
		      }
		  }
	      }
#endif // AFE_ADC_SOFT_LAUNCHED_SIMPLE_POOL
#endif // AFE_ADC_ISR
    }
}
#endif // AFE_ADC_SOFT_LAUNCHED

static void __attribute__((cold, optimize("-Os")))
machine_main_init_case (void)
{
  machine_main_status = e_machine_main_idle;
#if USE_UART_AS_DEBUG_OUTPUT
  modify_aurt_as_test_led ();

  const uint32_t dd = 25;
  for (uint8_t _ = 0; _ < 10; ++_)
    {
      blink1 ();
      HAL_Delay (dd);
    }
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
#endif

  for (uint8_t i0 = 0; i0 < AFE_NUMBER_OF_CHANNELS; ++i0)
    {
      adc_dma_buffer[i0] = 0;
    }
#if !AFE_ADC_SOFT_LAUNCHED_SIMPLE_POOL
	afe_adc_i = 0;
	afe_adc_soft_started = 0;
#endif // !AFE_ADC_SOFT_LAUNCHED_SIMPLE_POOL
#if MACHINE_DEBUG_RUN
	_startADC (0);
	uint16_t dac_value_master = 3000;
	uint16_t dac_value_slave = 3000;
	HAL_DAC_Start (&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start (&hdac, DAC_CHANNEL_2);
	HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value_master);
	HAL_DAC_SetValue (&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value_slave);
#endif
}

inline void __attribute__((always_inline, optimize("-O3")))
machine_main (void)
{
  switch (machine_main_status)
    {
    case e_machine_main_init:
      {
	machine_main_init_case();
	break;
      }
    case e_machine_main_idle:
      {
#if AFE_ADC_SOFT_LAUNCHED
	machine_adc_pool ();
#endif // AFE_ADC_SOFT_LAUNCHED
	/* Update CAN machine */
	can_machine ();
	/* Check if any new CAN message received */
	if (canRxFlag)
	  {
	    canRxFlag = 0;
	    /* Execute command */
	    can_execute (can_msg_received);
	  }
#if MACHINE_DEBUG_RUN
	machine_debug_run ();
	break;
#endif
	machine_control ();
	machine_periodic_report ();
	break;
      }
    default:
      {
	Error_Handler();
	break;
      }
    }
}

#if AFE_ADC_SOFT_LAUNCHED
#else
/* DMA Transfer Complete Callback */
void __attribute__ ((optimize("-O3")))
HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
#if AFE_ADC_ISR & AFE_ADC_SOFT_LAUNCHED
  if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))
    {
      adc_dma_buffer[afe_adc_i] = (uint16_t) HAL_ADC_GetValue (hadc);
      ++afe_adc_i;
      if (afe_adc_i < AFE_NUMBER_OF_CHANNELS)
	{
	  HAL_ADC_Start_IT (hadc);
	}
    }
#else
//  _rdy2cpy = 1;
//    __DSB();  // Data Synchronization Barrier (ensures previous memory accesses complete)
  /* Store timestamp */
  HAL_ADC_Stop_DMA (hadc); // Important: Stop before restart
  s_ADC_Measurement _ADC_Measurement_HAL_ADC_ConvCpltCallback;
  _ADC_Measurement_HAL_ADC_ConvCpltCallback.timestamp_ms = HAL_GetTick ();
  for (uint8_t i = 0; i < AFE_NUMBER_OF_CHANNELS; ++i)
    {
      /* Ensure the latest ADC value is read from memory (not a cached value) */
//        __DMB();  // Data Memory Barrier (ensures correct ordering of memory operations)
//      _ADC_Measurement_HAL_ADC_ConvCpltCallback.adc_value = 0x1FFF & (uint16_t)adc_dma_buffer[i];
      _ADC_Measurement_HAL_ADC_ConvCpltCallback.adc_value = 0x0FFF & adc_dma_buffer[i];
      add_to_buffer (&bufferADC[i], &_ADC_Measurement_HAL_ADC_ConvCpltCallback);
      /* Ensure value is stored before proceeding */
//        __DSB();
    }
  HAL_ADC_Start_DMA (hadc, (uint32_t*) &adc_dma_buffer[0], AFE_NUMBER_OF_CHANNELS);
  /* Ensure ISR execution order is correct before exiting */
//    __ISB();  // Instruction Synchronization Barrier (ensures all instructions complete before next)
#endif
}
#endif // AFE_ADC_SOFT_LAUNCHED
