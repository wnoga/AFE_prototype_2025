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

extern uint16_t adc_dma_buffer[];
uint16_t adc_dma_buffer[AFE_NUMBER_OF_CHANNELS];
#if USE_STACK_FOR_BUFFER
s_ADC_Measurement *adc_measurement_raw[AFE_NUMBER_OF_CHANNELS];
#else
s_ADC_Measurement adc_measurement_raw[AFE_NUMBER_OF_CHANNELS][ADC_MEASUREMENT_RAW_SIZE_MAX];
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

s_BufferADC bufferADC[AFE_NUMBER_OF_CHANNELS];

s_channelSettings afe_channelSettings[AFE_NUMBER_OF_CHANNELS];

s_regulatorSettings afe_regulatorSettings[AFE_NUMBER_OF_SUBDEVICES]; // Regulator settings for master and slave

volatile e_machine_main machine_main_status = e_machine_main_init;

uint32_t periodic_send_info_period_ms = 1500;
uint32_t periodic_send_info_start_ms = 0;

int8_t machnie_flag_averaging_enabled[AFE_NUMBER_OF_CHANNELS];
int8_t machine_temperatureLoop_enabled[2];

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

/***
 * Keep this to avoid mistake from values
 * channel 0x00 -> DAC_CHANNEL_1
 * channel 0x10 -> DAC_CHANNEL_2
 * channel 0x11 -> both
 */
static void
machine_DAC_set_and_start (uint8_t channel, uint16_t value)
{
#if DEBUG_HARDWARE_CONTROL_DISABLED | HARDWARE_CONTROL_DAC_DISABLED
#warning "DEBUG_HARDWARE_CONTROL_DISABLED"
  return;
#endif
  switch (channel)
    {
    case DAC_CHANNEL_1:
      {
	if (HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value) != HAL_OK)
	  {
	    Error_Handler();
	  }
	if (HAL_DAC_Start (&hdac, DAC_CHANNEL_1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	break;
      }
    case DAC_CHANNEL_2:
      {
	if (HAL_DAC_SetValue (&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value) != HAL_OK)
	  {
	    Error_Handler();
	  }
	if (HAL_DAC_Start (&hdac, DAC_CHANNEL_2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	break;
      }
    case 0x11:
      {
	if (HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value) != HAL_OK)
	  {
	    Error_Handler();
	  }
	if (HAL_DAC_SetValue (&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value) != HAL_OK)
	  {
	    Error_Handler();
	  }
	if (HAL_DAC_Start (&hdac, DAC_CHANNEL_1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	if (HAL_DAC_Start (&hdac, DAC_CHANNEL_2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	break;
      }
    default:
      break;
    }
}

/***
 * Keep this to avoid mistake from values
 * channel 0x00 -> DAC_CHANNEL_1
 * channel 0x10 -> DAC_CHANNEL_2
 * channel 0x11 -> both
 */
static void
machine_DAC_set (uint8_t channel, uint16_t value)
{
#if DEBUG_HARDWARE_CONTROL_DISABLED | HARDWARE_CONTROL_DAC_DISABLED
#warning "DEBUG_HARDWARE_CONTROL_DISABLED"
  return;
#endif
  switch (channel)
    {
    case DAC_CHANNEL_1:
      {
	if (HAL_DAC_SetValue (&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value) != HAL_OK)
	  {
	    Error_Handler();
	  }
	break;
      }
    case DAC_CHANNEL_2:
      {
	if (HAL_DAC_SetValue (&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value) != HAL_OK)
	  {
	    Error_Handler();
	  }
	break;
      }
    case 0x11:
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

void
machine_DAC_set_si (uint8_t channel, float voltage)
{
  const float V_min = 50.0f;
  const float V_max = 82.0f;
  if (voltage == 0)
    {
      machine_DAC_set (channel, UINT16_MAX);
      return;
    }
  else if (voltage < V_min)
    {
      voltage = V_min;
    }
  else if (voltage > V_max)
    {
      voltage = V_max;
    }
  uint16_t value = UINT16_MAX * (voltage - V_min) / (V_max - V_min);
  machine_DAC_set (channel, value);
}

/***
 * Keep this to avoid mistake from values
 * channel 0x00 -> DAC_CHANNEL_1
 * channel 0x10 -> DAC_CHANNEL_2
 * channel 0x11 -> both
 */
static HAL_StatusTypeDef
machine_DAC_switch (uint8_t channel, uint8_t enable)
{
#if DEBUG_HARDWARE_CONTROL_DISABLED | HARDWARE_CONTROL_DAC_DISABLED
#warning "DEBUG_HARDWARE_CONTROL_DISABLED"
  return HAL_OK;
#endif
  switch (channel)
    {
    case DAC_CHANNEL_1:
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
    case DAC_CHANNEL_2:
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
    case 0x11:
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

/***
 * FIXME Create conversion from float to DAC uint16_t value
 */
inline uint16_t __attribute ((always_inline, optimize("-O3")))
machine_DAC_convert_V_to_DAC_value (float V, s_regulatorSettings *regulatorSettings)
{
  return faxplusb (V, regulatorSettings->a_dac, regulatorSettings->b_dac);
}

/***
 */
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
      afe_channelSettings[i0].multiplicator = 1.0;
      afe_channelSettings[i0].a = 1.0;
      afe_channelSettings[i0].b = 0.0;
      afe_channelSettings[i0].max_N = ADC_MEASUREMENT_RAW_SIZE_MAX;

      afe_channelSettings[i0].period_ms = 0;
      afe_channelSettings[i0].period_ms_last = 0;

      machnie_flag_averaging_enabled[i0] = 0;

      init_buffer (&bufferADC[i0], &adc_measurement_raw[i0][0], ADC_MEASUREMENT_RAW_SIZE_MAX,
      ADC_MEASUREMENT_RAW_DEFAULT_DT_MS);
    }
  /* Set default values for regulator */

  for (uint8_t i0 = 0; i0 < 2; ++i0)
    {
      afe_regulatorSettings[i0].dV_dT = AFE_REGULATOR_DEFAULT_dV_dT;
      afe_regulatorSettings[i0].T_opt = AFE_REGULATOR_DEFAULT_T0;
      afe_regulatorSettings[i0].V_opt = AFE_REGULATOR_DEFAULT_U0;
      afe_regulatorSettings[i0].V_offset = AFE_REGULATOR_DEFAULT_U_offset; // Voltage offset
      afe_regulatorSettings[i0].dT = AFE_REGULATOR_DEFAULT_dT; // delta Temperature when new DAC value can be set
      afe_regulatorSettings[i0].T_old = afe_regulatorSettings[i0].T_opt;
      afe_regulatorSettings[i0].enabled = 0;
#if USE_SMALLER_STEPS_NEAR_DAC_TARGET
      afe_regulatorSettings[i0].ramp_bit_step = 100;
#else
      afe_regulatorSettings[i0].ramp_bit_step = 1;
#endif
      afe_regulatorSettings[i0].ramp_bit_step_every_ms = 100;
      afe_regulatorSettings[i0].ramp_bit_step_timestamp_old_ms = 0;
      afe_regulatorSettings[i0].ramp_curent_voltage_set_bits = AFE_DAC_START;
      afe_regulatorSettings[i0].ramp_target_voltage_set_bits = AFE_DAC_START;
#if DEBUG_SEND_BY_CAN_MACHINE_CONTROL
      afe_regulatorSettings[i0].ramp_target_voltage_set_bits_old = !AFE_DAC_START;
#endif // DEBUG_SEND_BY_CAN_MACHINE_CONTROL
    }

  /* Set channel for temperature */
  /* Append ADC channel to regulator settings */
  afe_regulatorSettings[0].temperature_channelSettings_ptr =
      &afe_channelSettings[e_ADC_CHANNEL_TEMP_LOCAL];
  afe_regulatorSettings[1].temperature_channelSettings_ptr =
      &afe_channelSettings[e_ADC_CHANNEL_TEMP_EXT];
}

/**
 * @brief Processes the temperature control loop for a given regulator.
 * This function calculates the required voltage for SiPMs based on temperature
 * and updates the target DAC value for ramping.
 * @param regulatorSettings_ptr Pointer to the regulator settings structure.
 * @param timestamp_ms Current system timestamp in milliseconds.
 */
static inline void __attribute__((always_inline, optimize("-O3")))
process_temperature_loop (s_regulatorSettings *regulatorSettings_ptr, uint32_t timestamp_ms)
{
#if HARDWARE_CONTROL_TEMPERATURE_LOOP_DISABLED
  // Temperature loop is disabled, do nothing.
#else // HARDWARE_CONTROL_TEMPERATURE_LOOP_DISABLED
  float average_Temperature = get_average_atSettings (
      regulatorSettings_ptr->temperature_channelSettings_ptr, timestamp_ms);
  if (isnan(average_Temperature))
    {
      /* Skip if cannot calculate average */
      return;
    }

  if (fabsf (average_Temperature - regulatorSettings_ptr->T_old) >= regulatorSettings_ptr->dT)
    {
      float voltage_for_SiPM = get_voltage_for_SiPM_x (average_Temperature,
                                                       regulatorSettings_ptr);
      regulatorSettings_ptr->ramp_target_voltage_set_bits = machine_DAC_convert_V_to_DAC_value (
          voltage_for_SiPM, regulatorSettings_ptr);
#if DEBUG_SEND_BY_CAN_MACHINE_CONTROL
      if (regulatorSettings_ptr->ramp_target_voltage_set_bits
          != regulatorSettings_ptr->ramp_target_voltage_set_bits_old)
        {
          CAN_Message_t tmp; // Local message for debug
          tmp.id = CAN_ID_IN_MSG;
          tmp.timestamp = HAL_GetTick (); // for timeout
          tmp.data[0] = AFECommand_debug_machine_control;
          // Voltage
          CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, 0, 4,
                                                       regulatorSettings_ptr->subdevice,
                                                       &voltage_for_SiPM);
          // Average temperature
          CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, 1, 4,
                                                       regulatorSettings_ptr->subdevice,
                                                       &average_Temperature);
          // Old temperature
          CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, 2, 4,
                                                       regulatorSettings_ptr->subdevice,
                                                       &regulatorSettings_ptr->T_old);
          // Timestamp
          CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, 3, 4,
                                                       regulatorSettings_ptr->subdevice,
                                                       timestamp_ms);
        }
      regulatorSettings_ptr->ramp_target_voltage_set_bits_old =
          regulatorSettings_ptr->ramp_target_voltage_set_bits;
#endif // DEBUG_SEND_BY_CAN_MACHINE_CONTROL
      regulatorSettings_ptr->T_old = average_Temperature;
    }
#endif // HARDWARE_CONTROL_TEMPERATURE_LOOP_DISABLED
}

/**
 * @brief Processes the DAC ramping logic for a given regulator.
 * This function smoothly adjusts the current DAC output towards the target DAC value.
 * @param regulatorSettings_ptr Pointer to the regulator settings structure.
 * @param timestamp_ms Current system timestamp in milliseconds.
 */
static inline void __attribute__((always_inline, optimize("-O3")))
process_dac_ramping (s_regulatorSettings *regulatorSettings_ptr, uint32_t timestamp_ms)
{
#if HARDWARE_CONTROL_TEMPERATURE_LOOP_RAMP_BIT_DISABLED
  // DAC ramping is disabled, do nothing.
#else // HARDWARE_CONTROL_TEMPERATURE_LOOP_RAMP_BIT_DISABLED
  uint32_t d_bit = regulatorSettings_ptr->ramp_bit_step;
#if USE_SMALLER_STEPS_NEAR_DAC_TARGET
  if ((int32_t) regulatorSettings_ptr->ramp_curent_voltage_set_bits
      - (int32_t) regulatorSettings_ptr->ramp_target_voltage_set_bits
      < (regulatorSettings_ptr->ramp_bit_step))
    {
      d_bit = regulatorSettings_ptr->ramp_bit_step / 10;
    }
  if (d_bit < 1)
    {
      d_bit = 1;
    }
#endif // USE_SMALLER_STEPS_NEAR_DAC_TARGET

  if ((timestamp_ms - regulatorSettings_ptr->ramp_bit_step_timestamp_old_ms)
      >= regulatorSettings_ptr->ramp_bit_step_every_ms)
    {
      regulatorSettings_ptr->ramp_bit_step_timestamp_old_ms = timestamp_ms;
      if (regulatorSettings_ptr->ramp_curent_voltage_set_bits
          != regulatorSettings_ptr->ramp_target_voltage_set_bits)
        {
          if (regulatorSettings_ptr->ramp_curent_voltage_set_bits
              < regulatorSettings_ptr->ramp_target_voltage_set_bits)
            {
              if (((int32_t) regulatorSettings_ptr->ramp_curent_voltage_set_bits
                  + (int32_t) regulatorSettings_ptr->ramp_bit_step)
                  >= (int32_t) regulatorSettings_ptr->ramp_target_voltage_set_bits)
                {
                  regulatorSettings_ptr->ramp_curent_voltage_set_bits =
                      regulatorSettings_ptr->ramp_target_voltage_set_bits;
                }
              else
                {
                  regulatorSettings_ptr->ramp_curent_voltage_set_bits += d_bit;
                }
            }
          else
            {
              if (((int32_t) regulatorSettings_ptr->ramp_curent_voltage_set_bits
                  - (int32_t) regulatorSettings_ptr->ramp_bit_step)
                  <= (int32_t) regulatorSettings_ptr->ramp_target_voltage_set_bits)
                {
                  regulatorSettings_ptr->ramp_curent_voltage_set_bits =
                      regulatorSettings_ptr->ramp_target_voltage_set_bits;
                }
              else
                {
                  regulatorSettings_ptr->ramp_curent_voltage_set_bits -= d_bit;
                }
            }

          if (regulatorSettings_ptr->ramp_curent_voltage_set_bits >= AFE_DAC_MAX)
            {
              regulatorSettings_ptr->ramp_curent_voltage_set_bits = AFE_DAC_MAX;
            }
#if TEMPERATURE_LOOP_HARDWARE_CONTROL_DAC_DISABLED
          // DAC hardware control is disabled, do nothing.
#else // TEMPERATURE_LOOP_HARDWARE_CONTROL_DAC_DISABLED
          switch (regulatorSettings_ptr->subdevice) // Use subdevice directly
            {
            case e_subdevice_master:
              machine_DAC_set (DAC_CHANNEL_1,
                               regulatorSettings_ptr->ramp_curent_voltage_set_bits);
              break;
            case e_subdevice_slave:
              machine_DAC_set (DAC_CHANNEL_2,
                               regulatorSettings_ptr->ramp_curent_voltage_set_bits);
              break;
            default:
              Error_Handler(); // Should not happen with e_subdevice enum
              break;
            }
#endif // TEMPERATURE_LOOP_HARDWARE_CONTROL_DAC_DISABLED
        }
    }
#endif // HARDWARE_CONTROL_TEMPERATURE_LOOP_RAMP_BIT_DISABLED
}

/*
 * =================================================================================
 * CAN Command Handlers
 * =================================================================================
 * These static functions are called from can_execute() to handle specific
 * CAN commands, making the main dispatcher more modular and readable.
 */

/* --- System & Informational Command Handlers --- */

static inline void __attribute__((always_inline, optimize("-O3")))
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

static inline void __attribute__((always_inline, optimize("-O3")))
handle_getVersion (CAN_Message_t *reply)
{
  reply->dlc = 2 + verArrLen;
  memcpy (&reply->data[2], &verArr[0], verArrLen);
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

static inline void __attribute__((always_inline, optimize("-O3")))
handle_startADC (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  HAL_ADC_Start_DMA (&hadc, &adc_dma_buffer[0], AFE_NUMBER_OF_CHANNELS);
  HAL_TIM_Base_Start (&htim1);
  htim1.Instance->ARR = 50000 - 1;
  reply->data[2] = msg->Data[2]; // For channels
  reply->data[3] = msg->Data[3]; // Enabled
  reply->data[4] = 0x00;	   // Error
  reply->dlc = 5;
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

/* --- Data Acquisition Command Handlers --- */

static inline void __attribute__((always_inline, optimize("-O3")))
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

static inline void __attribute__((always_inline, optimize("-O3")))
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

/* --- Hardware Control Command Handlers --- */

static inline void __attribute__((always_inline, optimize("-O3")))
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

static inline void __attribute__((always_inline, optimize("-O3")))
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
	  reply->data[4] |= (
	      AD8402_Write (&hspi1, channel, value, TIMEOUT_SPI1_MS) == HAL_OK ? 0 : 1) << channel;
	}
    }
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

static inline void __attribute__((always_inline, optimize("-O3")))
handle_writeGPIO (const s_can_msg_recieved *msg, CAN_Message_t *reply)
{
  GPIO_TypeDef *GPIOx = GetGPIOPortByEnumerator (msg->Data[2]);
  uint32_t GPIO_Pin = 1 << msg->Data[3]; // GPbIO Pin mask
  uint8_t PinState = msg->Data[4];
  machine_GPIO_WritePin (GPIOx, GPIO_Pin, PinState);
  reply->data[2] = msg->Data[2];
  reply->data[3] = msg->Data[3];
  reply->data[4] = msg->Data[4];
  reply->dlc = 5;
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

/* --- Temperature Loop & DAC Command Handlers --- */

static inline void __attribute__((always_inline, optimize("-O3")))
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

static inline void __attribute__((always_inline, optimize("-O3")))
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
	  if (channel == 0)
	    {
	      machine_DAC_set (DAC_CHANNEL_1, value);
	    }
	  else
	    {
	      machine_DAC_set (DAC_CHANNEL_2, value);
	    }
	}
    }
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

static inline void __attribute__((always_inline, optimize("-O3")))
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
	  switch (subdev)
	    {
	    case AFECommandSubdevice_master:
	      machine_DAC_set (DAC_CHANNEL_1, value);
	      break;
	    case AFECommandSubdevice_slave:
	      machine_DAC_set (DAC_CHANNEL_2, value);
	      break;
	    default:
	      Error_Handler ();
	      break;
	    }
	}
    }
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

static inline void __attribute__((always_inline, optimize("-O3")))
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

static inline void __attribute__((always_inline, optimize("-O3")))
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
	  reply->data[4] |=
	      machine_DAC_switch ((channel == 0) ? DAC_CHANNEL_1 : DAC_CHANNEL_2, value) << channel;
	}
    }
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

static inline void __attribute__((always_inline, optimize("-O3")))
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
static void __attribute__((optimize("-O3")))
handle_set_channel_property (const s_can_msg_recieved *msg, CAN_Message_t *reply, size_t offset,
			     size_t size, bool reset_buffer)
{
  uint8_t channelMask = msg->Data[2];
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
    {
      if (channelMask & (1 << channel))
	{
	  memcpy ((uint8_t*) &afe_channelSettings[channel] + offset, &msg->Data[3], size);
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
static void __attribute__((optimize("-O3")))
handle_set_regulator_property (const s_can_msg_recieved *msg, CAN_Message_t *reply, size_t offset,
			       size_t size)
{
  uint8_t subdevMask = msg->Data[2];
  for (uint8_t subdev = 0; subdev < AFE_NUMBER_OF_SUBDEVICES; ++subdev)
    {
      if (subdevMask & (1 << subdev))
	{
	  memcpy ((uint8_t*) &afe_regulatorSettings[subdev] + offset, &msg->Data[3], size);
	}
    }
  // Echo the command back as confirmation
  memcpy (&reply->data[0], &msg->Data[0], msg->DLC);
  reply->dlc = msg->DLC;
  CANCircularBuffer_enqueueMessage (&canTxBuffer, reply);
}

/* --- Unknown Command Handler --- */

static inline void __attribute__((always_inline, optimize("-O3")))
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
void __attribute__ ((optimize("-O3")))
can_execute (const s_can_msg_recieved msg)
{
  CAN_Message_t tmp;
  uint8_t command = (uint8_t) msg.Data[0];
  uint32_t msg_id = CAN_ID_IN_MSG;
  tmp.timestamp = HAL_GetTick ();
  tmp.id = msg_id;
  tmp.data[0] = command; // Standard reply [function]
  tmp.data[1] = 0x00;
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
	handle_set_channel_property (
	    &msg, &tmp,
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
    case AFECommand_setChannel_multiplicator_byMask: // set averagingSettings.multiplicator
      {
	handle_set_channel_property (&msg, &tmp, offsetof(s_channelSettings, multiplicator),
				     sizeof(float), true);
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
	handle_set_channel_property (
	    &msg, &tmp,
	    offsetof(s_channelSettings, buffer_ADC) + offsetof(s_BufferADC, buffer_size),
	    sizeof(uint32_t), true);
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
//		get_n_latest_from_buffer (channelSettings[channel].buffer_ADC, 1, &adc_val);
		HAL_ADC_Start (&hadc);
		HAL_ADC_PollForConversion (&hadc, 1000);
		adc_val.adc_value = HAL_ADC_GetValue (&hadc);
		adc_value_real = faxplusbcs (adc_val.adc_value, &afe_channelSettings[channel]);
//		CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, msg_index,
//							     total_msg_count, channel,
//							     &adc_value_real);
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
    default:
      {
	handle_unknown_command (&tmp);
	break;
      }
    }
}

void __attribute__ ((optimize("-O3")))
machine_calculate_averageValues (float *here)
{
  uint32_t timestamp_now = HAL_GetTick ();
  for (uint8_t i0 = 0; i0 < AFE_NUMBER_OF_CHANNELS; ++i0)
    {
      if (machnie_flag_averaging_enabled[i0])
	{
	  here[i0] = get_average_atSettings (&afe_channelSettings[i0], timestamp_now);
	}
    }
}

/***
 * @brief Temperature control loop.
 * This function implements the temperature control loop for the AFE device.
 * It reads temperature measurements, calculates the required voltage for SiPMs,
 * and adjusts the DAC output accordingly.
 */
void __attribute__ ((optimize("-O3")))
machine_control (void)
{
  /* Calculate values for DAC -- active part of the Temperature Loop */
  for (uint8_t subdev = 0; subdev < 2; ++subdev)
    {
      s_regulatorSettings *regulatorSettings_ptr = &afe_regulatorSettings[subdev];
      if (!regulatorSettings_ptr->enabled)
	{
	  continue;
	}
      uint32_t timestamp_ms = HAL_GetTick ();

      // Process temperature loop logic
      process_temperature_loop(regulatorSettings_ptr, timestamp_ms);

      // Process DAC ramping logic
      process_dac_ramping(regulatorSettings_ptr, timestamp_ms);
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
		  adc_value_real = faxplusbcs (adc_val.adc_value, &afe_channelSettings[channel]);
		  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, 0,
							       total_msg_count, channel_mask,
							       &adc_value_real);
		  /* Add last data timestamp */
		  CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, 1,
								 total_msg_count, channel_mask,
								 adc_val.timestamp_ms);
		  /* Get average data */
		  adc_value_real = get_average_atSettings (ptr, timestamp);
		  CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, 2,
							       total_msg_count, channel_mask,
							       &adc_value_real);
		  /* Add calculation timestamp */
		  CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, 3,
								 total_msg_count, channel_mask,
								 timestamp);

		  ptr->period_ms_last = timestamp;
		}
	    }
	}
    }
}

static uint32_t last_blink = 0;

void
machine_main (void)
{
  switch (machine_main_status)
    {
    case e_machine_main_init:
      {
	machine_main_status = e_machine_main_idle;
	modify_aurt_as_test_led ();

	const uint32_t dd = 25;
	blink1 ();
	HAL_Delay (dd);
	blink1 ();
	HAL_Delay (dd);
	blink1 ();
	HAL_Delay (dd);
	blink1 ();
	for (uint8_t i0 = 0; i0 < AFE_NUMBER_OF_CHANNELS; ++i0)
	  {
	    adc_dma_buffer[i0] = 0;
	  }
//#if WATCHDOG_FOR_CAN_RECIEVER_ENABLED
//	main_machine_soft_watchdog_timestamp_ms = HAL_GetTick();
//#endif // WATCHDOG_FOR_CAN_RECIEVER_ENABLED
	break;
      }
    case e_machine_main_idle:
      {
	/* Update CAN machine */
	can_machine ();
	/* Check if any new CAN message received */
	if (canRxFlag)
	  {
	    canRxFlag = 0;
	    /* Execute command */
	    can_execute (can_msg_received);
	  }

	machine_control ();
	machine_periodic_report ();
	break;
      }
    default:
      Error_Handler();
      break;
    }
}

/* DMA Transfer Complete Callback */
void __attribute__ ((optimize("-O3")))
HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
//    __DSB();  // Data Synchronization Barrier (ensures previous memory accesses complete)

  /* Store timestamp */
  static s_ADC_Measurement _ADC_Measurement_HAL_ADC_ConvCpltCallback;
  _ADC_Measurement_HAL_ADC_ConvCpltCallback.timestamp_ms = HAL_GetTick ();

  for (uint8_t i = 0; i < AFE_NUMBER_OF_CHANNELS; ++i)
    {
      /* Ensure the latest ADC value is read from memory (not a cached value) */
//        __DMB();  // Data Memory Barrier (ensures correct ordering of memory operations)
      _ADC_Measurement_HAL_ADC_ConvCpltCallback.adc_value = adc_dma_buffer[i];

      /* Ensure value is stored before proceeding */
//        __DSB();
      add_to_buffer (&bufferADC[i], &_ADC_Measurement_HAL_ADC_ConvCpltCallback);
    }

  /* Ensure ISR execution order is correct before exiting */
//    __ISB();  // Instruction Synchronization Barrier (ensures all instructions complete before next)
}
