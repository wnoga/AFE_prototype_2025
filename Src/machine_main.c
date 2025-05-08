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

extern uint16_t adc_dma_buffer[];
uint16_t adc_dma_buffer[AFE_NUMBER_OF_CHANNELS];
s_ADC_Measurement adc_measurement_raw[AFE_NUMBER_OF_CHANNELS][ADC_MEASUREMENT_RAW_SIZE_MAX];

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

s_channelSettings channelSettings[AFE_NUMBER_OF_CHANNELS];

s_regulatorSettings regulatorSettings[AFE_NUMBER_OF_SUBDEVICES]; // Regulator settings for master and slave

typedef enum
{
  e_machine_main_init = 0,
  e_machine_main_idle

} e_machine_main;

volatile e_machine_main machine_main_status = e_machine_main_init;

uint32_t periodic_send_info_period_ms = 1500;
uint32_t periodic_send_info_start_ms = 0;

int8_t machnie_flag_averaging_enabled[AFE_NUMBER_OF_CHANNELS];
int8_t machine_temperatureLoop_enabled[2];

void
update_buffer_by_channelSettings (s_channelSettings *averagingSettings)
{
  averagingSettings->buffer_ADC->tail = averagingSettings->buffer_ADC->head = 0;
}

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
  return HAL_SPI_Transmit (hspi, (uint8_t*)&toTransmit, 1, timeout);
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
  return faxplusb(V, regulatorSettings->a_dac, regulatorSettings->b_dac);
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
      channelSettings[i0].channel_nr = i0;
      channelSettings[i0].averaging_method = e_average_NONE;
      channelSettings[i0].alpha = 1.0;
      channelSettings[i0].buffer_ADC = &bufferADC[i0];
      channelSettings[i0].max_dt_ms = 3600 * 1000; // 1 hour
      channelSettings[i0].multiplicator = 1.0;
      channelSettings[i0].a = 1.0;
      channelSettings[i0].b = 0.0;
      channelSettings[i0].max_N = ADC_MEASUREMENT_RAW_SIZE_MAX;

      channelSettings[i0].period_ms = 0;
      channelSettings[i0].period_ms_last = 0;

      machnie_flag_averaging_enabled[i0] = 0;

      init_buffer (&bufferADC[i0], &adc_measurement_raw[i0][0], ADC_MEASUREMENT_RAW_SIZE_MAX,
      ADC_MEASUREMENT_RAW_DEFAULT_DT_MS);
    }
  /* Set default values for regulator */

  for (uint8_t i0 = 0; i0 < 2; ++i0)
    {
      regulatorSettings[i0].dV_dT = AFE_REGULATOR_DEFAULT_dV_dT;
      regulatorSettings[i0].T_opt = AFE_REGULATOR_DEFAULT_T0;
      regulatorSettings[i0].V_opt = AFE_REGULATOR_DEFAULT_U0;
      regulatorSettings[i0].V_offset = AFE_REGULATOR_DEFAULT_U_offset; // Voltage offset
      regulatorSettings[i0].dT = AFE_REGULATOR_DEFAULT_dT; // delta Temperature when new DAC value can be set
      regulatorSettings[i0].T_old = regulatorSettings[i0].T_opt;
      regulatorSettings[i0].enabled = 0;
      regulatorSettings[i0].ramp_bit_step = 1;
      regulatorSettings[i0].ramp_bit_step_every_ms = 100;
      regulatorSettings[i0].ramp_bit_step_timestamp_old_ms = 0;
      regulatorSettings[i0].ramp_curent_voltage_set_bits = AFE_DAC_START;
      regulatorSettings[i0].ramp_target_voltage_set_bits = AFE_DAC_START;
#if DEBUG_SEND_BY_CAN_MACHINE_CONTROL
      regulatorSettings[i0].ramp_target_voltage_set_bits_old = !AFE_DAC_START;
#endif // DEBUG_SEND_BY_CAN_MACHINE_CONTROL
    }

  /* Set channel for temperature */
  /* Append ADC channel to regulator settings */
  regulatorSettings[0].temperature_channelSettings_ptr = &channelSettings[e_ADC_CHANNEL_TEMP_LOCAL];
  regulatorSettings[1].temperature_channelSettings_ptr = &channelSettings[e_ADC_CHANNEL_TEMP_EXT];
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
  // (0xF0 >> 4) = total number of messages in queue
  // (0x0F) = message number (in the queue)
  tmp.dlc = 8;
  switch ((AFECommand) command)
    {
    case AFECommand_getSerialNumber: // getSerialNumber
      {
	tmp.id |= e_CANIdFunctionCode_multipleRead & 0b11;
	tmp.dlc = 2 + 4;
	for (uint8_t i0 = 0; i0 < 3; ++i0)
	  {
	    tmp.data[1] = get_byte_of_message_number (i0, 3);
	    // Copy the UID data, byte-by-byte
	    memcpy (&tmp.data[2], (uint8_t*) &UID[i0], sizeof(uint32_t));
	    CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	  }
	break;
      }
    case AFECommand_getVersion: // getVersion
      {
	tmp.dlc = 2 + verArrLen;
	memcpy (&tmp.data[2], &verArr[0], verArrLen);
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
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
	/* Start ADC in DMA mode */
	HAL_ADC_Start_DMA (&hadc, &adc_dma_buffer[0], AFE_NUMBER_OF_CHANNELS);
	HAL_TIM_Base_Start (&htim1);
	htim1.Instance->ARR = 50000 - 1;
//	htim1.Instance->ARR = 10000 - 1;
	tmp.data[1] = get_byte_of_message_number (0, 1);
	tmp.data[2] = msg.Data[2]; // For channels
	tmp.data[3] = msg.Data[3]; // Enabled
	tmp.data[4] = 0x00;	   // Error
	tmp.dlc = 5;
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }

      /**** 0x30 ****/
      /* Send SI data [Data[0]] from ADC channel [Data[2]] as float value[Data[3:7]]  */
    case AFECommand_getSensorDataSi_last_byMask:
      {
	uint8_t channels = msg.Data[2];
	s_ADC_Measurement adc_val;
	float adc_value_real;
	uint8_t total_msg_count = get_number_of_channels (channels) + 1; // Channels + timestamp
	uint8_t msg_index = 0;
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channels & (1 << channel)) // loop over channel mask
	      {
		get_n_latest_from_buffer (channelSettings[channel].buffer_ADC, 1, &adc_val);
		adc_value_real = faxplusbcs (adc_val.adc_value, &channelSettings[channel]);
		CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, msg_index,
							     total_msg_count, 1<<channel,
							     &adc_value_real);
		++msg_index;
	      }
	  }
	CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, msg_index,
						       total_msg_count, tmp.timestamp);
	break;
      }
      /* Send SI data [Data[0]] from ADC channel [Data[2]] as float average value[Data[3:7]]  */
    case AFECommand_getSensorDataSi_average_byMask:
      {
	uint8_t channels = msg.Data[2];
	float adc_value_real;
	uint8_t total_msg_count = get_number_of_channels (channels) + 1;
	uint8_t msg_index = 0;
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channels & (1 << channel)) // loop over channel mask
	      {
		adc_value_real = get_average_atSettings (&channelSettings[channel], tmp.timestamp);
		CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, msg_index,
							     total_msg_count, 1<<channel,
							     &adc_value_real);
		++msg_index;
	      }
	  }
	CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, msg_index,
						       total_msg_count, tmp.timestamp);
	break;
      }
//      /* Send SI data [Data[0]] from ADC channel [Data[2]] as float average value[Data[3:7]]
//       * and timestamp on next message */
//    case AFECommand_getSensorDataSiAndTimestamp_average_byMask:
//      {
//	uint8_t channels = msg.Data[2];
//	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
//	  {
//	    if (channels & (1 << channel))
//	      {
//		send_SensorDataSiAndTimestamp_average (tmp.id, command, &canTxBuffer,
//						       &channelSettings[channel], HAL_GetTick ());
//	      }
//	  }
//	break;
//      }
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
	uint8_t spiData[5];
	uint8_t spiDataLen = msg.Data[2];
	tmp.dlc = 3;
	if (spiDataLen > 5)
	  {
	    tmp.data[2] = HAL_ERROR;
	    CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	    break;
	  }
	memcpy (&spiData[0], &msg.Data[3], spiDataLen);
	tmp.data[2] = machine_SPI_Transmit (&hspi1, &spiData[0], spiDataLen, TIMEOUT_SPI1_MS);
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
#warning "Test SPI!" // TODO Test SPI
	break;
      }
    case AFECommand_setAD8402Value_byte_byMask:
      {
	uint8_t channels = msg.Data[2];
	uint8_t value = msg.Data[3];
	tmp.data[1] = get_byte_of_message_number (0, 1);
	tmp.data[2] = channels;
	tmp.data[3] = value;
	tmp.data[4] = 0x00; // masked error status
	tmp.dlc = 5;
	for (uint8_t channel = 0; channel < NUMBER_OF_AD8402_CHANNELS; ++channel)
	  {
	    if (channels & (1 << channel))
	      {
		tmp.data[4] |= (AD8402_Write (&hspi1, channel, value, TIMEOUT_SPI1_MS) == HAL_OK ? 0 : 1) << channel;
	      }
	  }
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_writeGPIO:
      {
	/*
	 * Data[2] - GPIO port enumerator
	 * Data[3] - GPIO pin number
	 * Data[4] - GPIO pin state
	 */
	GPIO_TypeDef *GPIOx = GetGPIOPortByEnumerator (msg.Data[2]);
	uint32_t GPIO_Pin = 1 << msg.Data[3]; // GPIO Pin mask
	uint8_t PinState = msg.Data[4];
	machine_GPIO_WritePin (GPIOx, GPIO_Pin, PinState);
	tmp.data[1] = get_byte_of_message_number (0, 1);
	tmp.data[2] = msg.Data[2];
	tmp.data[3] = msg.Data[3];
	tmp.data[4] = msg.Data[4];
	tmp.dlc = 5;
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }

      /* 0xC0 */
      /* Temperature loop runtime */
    case AFECommand_setTemperatureLoopForChannelState_byMask_asStatus: // start or stop [Data[3] temperature loop for channel [Data[2]]
      {
	uint8_t channels = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	uint8_t status = msg.Data[3];
	tmp.data[2] = channels;
	tmp.data[3] = status;
	tmp.dlc = 2 + 2;
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
	  {
	    if (channels & (1 << channel))
	      {
		regulatorSettings[channel].enabled = status != 0 ? 1 : 0;
	      }
	  }
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setDACValueRaw_bySubdeviceMask: // set [Data[3] temperature loop for channel [Data[2]]
      {
	uint8_t channels = msg.Data[2];
	uint16_t value = 0;
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	tmp.data[2] = channels;
	memcpy (&tmp.data[3], &msg.Data[3], sizeof(uint16_t));
	tmp.dlc = 3 + sizeof(uint16_t);
	memcpy (&value, &msg.Data[3], sizeof(uint16_t));
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
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setDACValueSi_bySubdeviceMask: // set [Data[3] temperature loop for channel [Data[2]]
      {
	float valueSi;
	uint8_t subdevice_mask = msg.Data[2];
	memcpy (&valueSi, &msg.Data[3], sizeof(float)); /* Copy SI value */
	memcpy (&tmp.data[0], &msg.Data[0], msg.DLC); /* Return the same msg */
	for (uint8_t subdev = 0; subdev < AFE_NUMBER_OF_SUBDEVICES; ++subdev)
	  {
	    if (subdevice_mask & (1 << subdev))
	      {
		uint16_t value = machine_DAC_convert_V_to_DAC_value (valueSi,
								      &regulatorSettings[subdev]);
		switch (subdev)
		  {
		  case AFECommandSubdevice_master:
		    {
		      machine_DAC_set (DAC_CHANNEL_1, value);
		      break;
		    }
		  case AFECommandSubdevice_slave:
		    {
		      machine_DAC_set (DAC_CHANNEL_2, value);
		      break;
		    }
		  default:
		    {
		      Error_Handler();
		      break;
		    }
		  }
	      }
	  }
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_stopTemperatureLoopForAllChannels: // stop temperature loop for all channels
      {
	for (uint8_t channel = 0; channel < 2; ++channel)
	  {
	    tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	    regulatorSettings[channel].enabled = 0;
	  }
	tmp.data[2] = 1;
	tmp.dlc = 2 + 1;
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setDAC_bySubdeviceMask: // Start or stop DAC
      {
	uint8_t channel_mask = msg.Data[2];
	uint8_t value = msg.Data[3];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	tmp.data[2] = channel_mask; // Copy mask
	tmp.data[3] = msg.Data[3]; // Copy value
	tmp.data[4] = 0x00; // Clear error status mask
	if (value > 1)
	  {
	    break;
	  }
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
	  {
	    if (channel_mask & (1 << channel))
	      {
		tmp.data[4] |= machine_DAC_switch ((channel == 0) ? DAC_CHANNEL_1 : DAC_CHANNEL_2,
						   value) << channel;
	      }
	  }
	tmp.dlc = 5;
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setDACRampOneBytePerMillisecond_ms:
      {
#warning "Implement and test this"
	break;
      }

      /*********** 0xD0 ************/
      /* Set setting [Data[0]] for adc channel [Data[2]] by uint32_t value[Data[3:7]]  */
    case AFECommand_setAveragingMode_byMask: // set averagingSettings.averaging_method
      {
	uint8_t channelMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channelMask & (1 << channel))
	      {
		memcpy (&channelSettings[channel].averaging_method, &msg.Data[3],
			sizeof(e_average));
		update_buffer_by_channelSettings (&channelSettings[channel]);
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setAveragingAlpha_byMask: // set averagingSettings.alpha
      {
	uint8_t channelMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channelMask & (1 << channel))
	      {
		memcpy (&channelSettings[channel].alpha, &msg.Data[3], sizeof(float));
		update_buffer_by_channelSettings (&channelSettings[channel]);
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setAveragingBufferSize_byMask: // set averagingSettings.buffer_size
      {
	uint8_t channelMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channelMask & (1 << channel))
	      {
		memcpy (&channelSettings[channel].buffer_ADC->buffer_size, &msg.Data[3],
			sizeof(uint32_t));
		update_buffer_by_channelSettings (&channelSettings[channel]);
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setChannel_dt_ms_byMask:
      {
	uint8_t channelMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channelMask & (1 << channel))
	      {
		memcpy (&channelSettings[channel].buffer_ADC->dt_ms, &msg.Data[3],
			sizeof(uint32_t));
		update_buffer_by_channelSettings (&channelSettings[channel]);
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setAveraging_max_dt_ms_byMask: // set averagingSettings.max_dt_ms
      {
	uint8_t channelMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channelMask & (1 << channel))
	      {
		memcpy (&channelSettings[channel].max_dt_ms, &msg.Data[3], sizeof(uint32_t));
		update_buffer_by_channelSettings (&channelSettings[channel]);
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setChannel_multiplicator_byMask: // set averagingSettings.multiplicator
      {
	uint8_t channelMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channelMask & (1 << channel))
	      {
		memcpy (&channelSettings[channel].multiplicator, &msg.Data[3], sizeof(float));
		update_buffer_by_channelSettings (&channelSettings[channel]);
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setAveragingSubdevice: // set averagingSettings.subdevice
      {
	Error_Handler();
	break;
      }
    case AFECommand_setChannel_a_byMask:
      {
	uint8_t channelMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channelMask & (1 << channel))
	      {
		memcpy (&channelSettings[channel].a, &msg.Data[3], sizeof(float));
		update_buffer_by_channelSettings (&channelSettings[channel]);
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setChannel_b_byMask:
      {
	uint8_t channelMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channelMask & (1 << channel))
	      {
		memcpy (&channelSettings[channel].b, &msg.Data[3], sizeof(float));
		update_buffer_by_channelSettings (&channelSettings[channel]);
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setChannel_period_ms_byMask:
      {
	uint8_t channelMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channelMask & (1 << channel))
	      {
		memcpy (&channelSettings[channel].period_ms, &msg.Data[3], sizeof(float));
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setRegulator_a_dac_byMask:
      {
	uint8_t subdevMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t subdev = 0; subdev < AFE_NUMBER_OF_SUBDEVICES; ++subdev)
	  {
	    if (subdevMask & (1 << subdev))
	      {
		memcpy (&regulatorSettings[subdev].a_dac, &msg.Data[3], sizeof(float));
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setRegulator_b_dac_byMask:
      {
	uint8_t subdevMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t subdev = 0; subdev < AFE_NUMBER_OF_SUBDEVICES; ++subdev)
	  {
	    if (subdevMask & (1 << subdev))
	      {
		memcpy (&regulatorSettings[subdev].b_dac, &msg.Data[3], sizeof(float));
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setRegulator_dV_dT_byMask:
      {
	uint8_t subdevMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t subdev = 0; subdev < AFE_NUMBER_OF_SUBDEVICES; ++subdev)
	  {
	    if (subdevMask & (1 << subdev))
	      {
		memcpy (&regulatorSettings[subdev].dV_dT, &msg.Data[3], sizeof(float));
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setRegulator_V_opt_byMask:
      {
	uint8_t subdevMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t subdev = 0; subdev < AFE_NUMBER_OF_SUBDEVICES; ++subdev)
	  {
	    if (subdevMask & (1 << subdev))
	      {
		memcpy (&regulatorSettings[subdev].V_opt, &msg.Data[3], sizeof(float));
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setRegulator_V_offset_byMask:
      {
	uint8_t subdevMask = msg.Data[2];
	tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	for (uint8_t subdev = 0; subdev < AFE_NUMBER_OF_SUBDEVICES; ++subdev)
	  {
	    if (subdevMask & (1 << subdev))
	      {
		memcpy (&regulatorSettings[subdev].V_offset, &msg.Data[3], sizeof(float));
	      }
	  }
	memcpy(&tmp.data[0],&msg.Data[0],msg.DLC);
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);
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
		adc_value_real = faxplusbcs (adc_val.adc_value, &channelSettings[channel]);
		CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, msg_index,
							     total_msg_count, 1<<channel,
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
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
	  {
	    if (channels & (1 << channel)) // loop over channel mask
	      {
//		get_n_latest_from_buffer (channelSettings[channel].buffer_ADC, 1, &adc_val);
		HAL_ADC_Start (&hadc);
		HAL_ADC_PollForConversion (&hadc, 1000);
		adc_val.adc_value = HAL_ADC_GetValue (&hadc);
		adc_value_real = faxplusbcs (adc_val.adc_value, &channelSettings[channel]);
//		CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, msg_index,
//							     total_msg_count, channel,
//							     &adc_value_real);
		CANCircularBuffer_enqueueMessage_data (&canTxBuffer, &tmp, msg_index,
						       total_msg_count, 1 << channel,
						       (uint8_t*) &adc_value_real, sizeof(float));
		++msg_index;
	      }
	  }
	CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, msg_index,
						       total_msg_count, tmp.timestamp);
	break;
      }
    default:
      {
	tmp.dlc = 8;
	tmp.data[0] = 0xFF;
	tmp.data[1] = 0x34;
	tmp.data[2] = 0x56;
	tmp.data[3] = 0x78;
	tmp.data[4] = 0x90;
	tmp.data[5] = 0xAB;
	tmp.data[6] = 0xCD;
	tmp.data[7] = 0xEF;
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
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
	  here[i0] = get_average_atSettings (&channelSettings[i0], timestamp_now);
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
      s_regulatorSettings *regulatorSettings_ptr = &regulatorSettings[subdev];
      if (!regulatorSettings_ptr->enabled)
	{
	  continue;
	}
      uint32_t timestamp_ms = HAL_GetTick ();
#if HARDWARE_CONTROL_TEMPERATURE_LOOP_DISABLED
#else // HARDWARE_CONTROL_TEMPERATURE_LOOP_DISABLED
      float average_Temperature = get_average_atSettings (regulatorSettings_ptr->temperature_channelSettings_ptr, timestamp_ms);
      if (isnan(average_Temperature))
	{
	  /* Skip if cannot calculate average */
	  continue;
	}

      if (fabsf (average_Temperature - regulatorSettings_ptr->T_old) >= regulatorSettings_ptr->dT)
	{
	  float voltage_for_SiPM = get_voltage_for_SiPM_x (average_Temperature,
							   regulatorSettings_ptr);
	  regulatorSettings_ptr->ramp_target_voltage_set_bits =
	      machine_DAC_convert_V_to_DAC_value (voltage_for_SiPM, regulatorSettings_ptr);
#if DEBUG_SEND_BY_CAN_MACHINE_CONTROL
	  if (regulatorSettings_ptr->ramp_target_voltage_set_bits
	      != regulatorSettings_ptr->ramp_target_voltage_set_bits_old)
	    {
	      CAN_Message_t tmp;
	      tmp.id = CAN_ID_IN_MSG;
	      tmp.timestamp = HAL_GetTick (); // for timeout
	      tmp.data[0] = AFECommand_debug_machine_control;
	      // Voltage
	      tmp.data[1] = get_byte_of_message_number (0, 4);
	      tmp.data[2] = subdev;
	      tmp.dlc = 2 + 1 + sizeof(float);
	      memcpy (&tmp.data[3], &voltage_for_SiPM, sizeof(float));
	      CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	      // Average temperature
	      tmp.data[1] = get_byte_of_message_number (1, 4);
	      tmp.data[2] = subdev;
	      tmp.dlc = 2 + 1 + sizeof(float);
	      memcpy (&tmp.data[3], &average_Temperature, sizeof(float));
	      CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	      // Old temperature
	      tmp.data[1] = get_byte_of_message_number (2, 4);
	      tmp.data[2] = subdev;
	      tmp.dlc = 2 + 1 + sizeof(float);
	      memcpy (&tmp.data[3], &regulatorSettings_ptr->T_old, sizeof(float));
	      CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	      // Timestamp
	      tmp.data[1] = get_byte_of_message_number (3, 4);
	      tmp.data[2] = subdev;
	      tmp.dlc = 2 + 1 + sizeof(uint32_t);
	      memcpy (&tmp.data[3], &timestamp_ms, sizeof(uint32_t));
	      CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	    }
	  regulatorSettings_ptr->ramp_target_voltage_set_bits_old = regulatorSettings_ptr->ramp_target_voltage_set_bits;
#endif // DEBUG_SEND_BY_CAN_MACHINE_CONTROL
	  regulatorSettings_ptr->T_old = average_Temperature;
	}
#endif // HARDWARE_CONTROL_TEMPERATURE_LOOP_DISABLED
#if HARDWARE_CONTROL_TEMPERATURE_LOOP_RAMP_BIT_DISABLED
#else // HARDWARE_CONTROL_TEMPERATURE_LOOP_RAMP_BIT_DISABLED
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
		      regulatorSettings_ptr->ramp_curent_voltage_set_bits +=
			  regulatorSettings_ptr->ramp_bit_step;
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
		      regulatorSettings_ptr->ramp_curent_voltage_set_bits -=
			  regulatorSettings_ptr->ramp_bit_step;
		    }
		}

	      if (regulatorSettings_ptr->ramp_curent_voltage_set_bits >= AFE_DAC_MAX)
		{
		  regulatorSettings_ptr->ramp_curent_voltage_set_bits = AFE_DAC_MAX;
		}
#if TEMPERATURE_LOOP_HARDWARE_CONTROL_DAC_DISABLED
	      switch (subdev)
		{
		case 0:
		  {
		    machine_DAC_set (DAC_CHANNEL_1,
				     regulatorSettings_ptr->ramp_curent_voltage_set_bits);
		    break;
		  }
		case 1:
		  {
		    machine_DAC_set (DAC_CHANNEL_2,
				     regulatorSettings_ptr->ramp_curent_voltage_set_bits);
		    break;
		  }
		default:
		  {
		    Error_Handler();
		    break;
		  }
		}
#endif // TEMPERATURE_LOOP_HARDWARE_CONTROL_DAC_DISABLED
	    }
	}
#endif // HARDWARE_CONTROL_TEMPERATURE_LOOP_RAMP_BIT_DISABLED
    }
}

static void __attribute__ ((optimize("-O3")))
machine_periodic_report (void)
{
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_CHANNELS; ++channel)
    {
      s_channelSettings *ptr = &channelSettings[channel];
      uint32_t timestamp = HAL_GetTick ();
      if (ptr->period_ms > 0)
	{
	  if ((timestamp - ptr->period_ms_last) >= ptr->period_ms)
	    {
	      float adc_value_real = 0.0;
	      ptr->period_ms_last = timestamp;
	      CAN_Message_t tmp;
	      tmp.id = CAN_ID_IN_MSG;
	      tmp.data[0] = AFECommand_getSensorDataSi_periodic;
	      tmp.timestamp = timestamp;
	      uint8_t channel_mask = 1 << channel;
	      s_ADC_Measurement adc_val;
	      uint8_t total_msg_count = 4;

	      /* Get last data */
	      get_n_latest_from_buffer (channelSettings[channel].buffer_ADC, 1, &adc_val);
	      adc_value_real = faxplusbcs (adc_val.adc_value, &channelSettings[channel]);
	      CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, 0, total_msg_count,
							   channel_mask, &adc_value_real);
	      /* Add last data timestamp */
	      CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, 1, total_msg_count,
							     adc_val.timestamp_ms);
	      /* Get average data */
	      adc_value_real = get_average_atSettings (ptr, timestamp);
	      CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, 2, total_msg_count,
							   channel_mask, &adc_value_real);
	      /* Add calculation timestamp */
	      CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, 3, total_msg_count,
							     timestamp);
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

	for (uint8_t i0 = 0; i0 < 2; ++i0)
	  {
	    HAL_Delay (50);
	    blink1 ();
	  }
	for (uint8_t i0 = 0; i0 < AFE_NUMBER_OF_CHANNELS; ++i0)
	  {
	    adc_dma_buffer[i0] = 0;
	  }
#if WATCHDOG_FOR_CAN_RECIEVER_ENABLED
	main_machine_soft_watchdog_timestamp_ms = HAL_GetTick();
#endif // WATCHDOG_FOR_CAN_RECIEVER_ENABLED
	break;
      }
    case e_machine_main_idle:
      {
#if WATCHDOG_FOR_CAN_RECIEVER_ENABLED
	if ((HAL_GetTick () - main_machine_soft_watchdog_timestamp_ms)
	    > main_machine_soft_watchdog_timeout_ms)
	  {
	    NVIC_SystemReset (); // Reset if no respond
	  }
#endif // WATCHDOG_FOR_CAN_RECIEVER_ENABLED
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
  _ADC_Measurement_HAL_ADC_ConvCpltCallback.timestamp_ms = HAL_GetTick();

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
