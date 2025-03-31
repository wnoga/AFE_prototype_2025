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
  e_adc_channel_DC_LEVEL_MEAS0=0,
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

void update_buffer_by_channelSettings(s_channelSettings *averagingSettings)
{
  averagingSettings->buffer_ADC->tail = averagingSettings->buffer_ADC->head = 0;
}

/***
 * tmp->data[0] and tmp->data[1] should be set before this function
 */
static void
CANCircularBuffer_enqueueMessage_and_update_channelSettings_byMsg (CANCircularBuffer_t *cb,
								   const s_can_msg_recieved *msg,
								   s_channelSettings *chs0,
								   CAN_Message_t *tmp,
								   const size_t offset,
								   const uint8_t size)
{
  uint8_t channels = msg->Data[2];
  tmp->data[2] = channels;
  memcpy (&tmp->data[3], &msg->Data[3], size);
  tmp->dlc = 2 + 1 + size;
  for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
    {
      if (0x01 & (channels >> channel))
	{
	  memcpy (
	      (void*) ((size_t) &chs0[channel] + (size_t) (channel * sizeof(s_channelSettings))
		  + offset),
	      &msg->Data[3], size);
	  update_buffer_by_channelSettings (&chs0[channel]);
	}
    }
  CANCircularBuffer_enqueueMessage (cb, tmp);
}

static void enqueueSensorDataSIandTimestamp(CANCircularBuffer_t *cb, uint8_t command, uint8_t channel,float adc_value_real, uint32_t timestamp)
{
	CAN_Message_t tmp;
	tmp.id = CAN_ID_IN_MSG;
	tmp.timestamp = HAL_GetTick();
	tmp.data[0] = command;
	tmp.data[1] = 0x21;
	tmp.data[2] = channel;
	tmp.dlc = 2 + 1 + sizeof(float);
	memcpy (&tmp.data[3], &adc_value_real, sizeof(float));
	CANCircularBuffer_enqueueMessage (cb, &tmp);
	tmp.data[0] = command;
	tmp.data[1] = 0x22;
	tmp.data[2] = channel;
	tmp.dlc = 2 + 1 + sizeof(uint32_t);
	memcpy (&tmp.data[3], &timestamp, sizeof(float));
	CANCircularBuffer_enqueueMessage (cb, &tmp);
}

static void enqueueSensorDataSI(CANCircularBuffer_t *cb, uint8_t command, uint8_t channel,float adc_value_real)
{
	CAN_Message_t tmp;
	tmp.id = CAN_ID_IN_MSG;
	tmp.timestamp = HAL_GetTick();
	tmp.data[0] = command;
	tmp.data[1] = 0x11;
	tmp.data[2] = channel;
	tmp.dlc = 2 + 1 + sizeof(float);
	memcpy (&tmp.data[3], &adc_value_real, sizeof(float));
	CANCircularBuffer_enqueueMessage (cb, &tmp);
}

static void
send_SensorDataSi_average (uint8_t msg_id, uint8_t command, CANCircularBuffer_t *cb,
			   s_channelSettings *ch)
{
  float adc_value_real;
  CAN_Message_t tmp;
  tmp.id = msg_id;
  tmp.timestamp = HAL_GetTick (); // for timeout
  adc_value_real = get_average_atSettings (ch,tmp.timestamp);
  tmp.data[0] = command;
  tmp.data[1] = get_byte_of_message_number (0, 1);
  tmp.data[2] = ch->channel_nr;
  tmp.dlc = 2 + 1 + sizeof(float);
  memcpy (&tmp.data[3], &adc_value_real, sizeof(float));
  CANCircularBuffer_enqueueMessage (cb, &tmp);
}
static void
send_SensorDataSiAndTimestamp_average (uint8_t msg_id, uint8_t command, CANCircularBuffer_t *cb,
				       s_channelSettings *ch, uint32_t timestamp)
{
  float adc_value_real;
  CAN_Message_t tmp;
  tmp.id = msg_id;
  tmp.timestamp = HAL_GetTick (); // for timeout
  adc_value_real = get_average_atSettings (ch, tmp.timestamp);
  tmp.data[0] = command;
  tmp.data[1] = get_byte_of_message_number (0, 2);
  tmp.data[2] = 1 << ch->channel_nr;
  tmp.dlc = 2 + 1 + sizeof(float);
  memcpy (&tmp.data[3], &adc_value_real, sizeof(float));
  CANCircularBuffer_enqueueMessage (cb, &tmp);
  tmp.data[0] = command;
  tmp.data[1] = get_byte_of_message_number (1, 2);
  tmp.data[2] = 1 << ch->channel_nr;
  tmp.dlc = 2 + 1 + sizeof(uint32_t);
  memcpy (&tmp.data[3], &timestamp, sizeof(float));
  CANCircularBuffer_enqueueMessage (cb, &tmp);
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
  uint16_t data = ((channel & 0x03) << 8) | (value & 0xFF);  // 10-bit format
  return machine_SPI_Transmit (hspi, (uint8_t*) &data, 1, timeout);
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
  if (voltage < V_min)
    {
      voltage = V_min;
    }
  if (voltage > V_max)
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
static uint16_t
machine_DAC_convert_mv_to_dac_value (float mV)
{
#warning "FIXME Create conversion from float to DAC uint16_t value"
  return mV;
}
void __attribute__ ((cold, optimize("-Os")))
machine_main_init_0 (void)
{
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

  for (uint8_t i0 = 0; i0 < 2; ++i0)
    {
      regulatorSettings[i0].a = AFE_REGULATOR_DEFAULT_a;
      regulatorSettings[i0].T_0 = AFE_REGULATOR_DEFAULT_T0;
      regulatorSettings[i0].U_0 = AFE_REGULATOR_DEFAULT_U0;
      regulatorSettings[i0].U_offset = AFE_REGULATOR_DEFAULT_U_offset;
      regulatorSettings[i0].dT = AFE_REGULATOR_DEFAULT_dT;
//      regulatorSettings[i0].dU = AFE_REGULATOR_DEFAULT_dT;
      regulatorSettings[i0].T_old = regulatorSettings[i0].T_0;
      regulatorSettings[i0].enabled = 0;
      regulatorSettings[i0].ramp_bit_step = 1;
      regulatorSettings[i0].ramp_bit_step_every_ms = 100;
      regulatorSettings[i0].ramp_bit_step_timestamp_old_ms = 0;
      regulatorSettings[i0].ramp_curent_voltage_set_bits = AFE_DAC_START;
      regulatorSettings[i0].ramp_target_voltage_set_bits = AFE_DAC_START;
    }

  /* Append ADC channel to regulator settings */
  regulatorSettings[0].temperature_channelSettings_ptr = &channelSettings[e_ADC_CHANNEL_TEMP_LOCAL];
  regulatorSettings[1].temperature_channelSettings_ptr = &channelSettings[e_ADC_CHANNEL_TEMP_EXT];
}

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
	tmp.dlc = 2 + 1;
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
	    if (0x01 & (channels >> channel)) // loop over channel mask
	      {
		get_n_latest_from_buffer (channelSettings[channel].buffer_ADC, 1, &adc_val);
		adc_value_real = faxplusb(adc_val.adc_value, &channelSettings[channel]);
		CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, msg_index,
							     total_msg_count, channel,
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
	    if (0x01 & (channels >> channel)) // loop over channel mask
	      {
		adc_value_real = get_average_atSettings (&channelSettings[channel], tmp.timestamp);
		CANCircularBuffer_enqueueMessage_data_float (&canTxBuffer, &tmp, msg_index,
							     total_msg_count, channel,
							     &adc_value_real);
		++msg_index;
	      }
	  }
	CANCircularBuffer_enqueueMessage_timestamp_ms (&canTxBuffer, &tmp, msg_index,
						       total_msg_count, tmp.timestamp);
	break;
      }
      /* Send SI data [Data[0]] from ADC channel [Data[2]] as float average value[Data[3:7]]
       * and timestamp on next message */
    case AFECommand_getSensorDataSiAndTimestamp_average_byMask:
      {
	uint8_t channels = msg.Data[2];
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
	  {
	    if (0x01 & (channels >> channel))
	      {
		send_SensorDataSiAndTimestamp_average (tmp.id, command, &canTxBuffer,
						       &channelSettings[channel], HAL_GetTick ());
	      }
	  }
	break;
      }
    case AFECommand_getSensorDataSi_all_periodic_average:
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
	tmp.dlc = 2 + 1;
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
	tmp.data[1] = get_byte_of_message_number(0, 1);
	tmp.data[2] = channels;
	tmp.data[3] = value;
	tmp.data[4] = 0x00; // masked error status
	tmp.dlc = 5;
	for (uint8_t channel = 0; channel < NUMBER_OF_AD8402_CHANNELS; ++channel)
	  {
	    if (0x01 & (channels >> channel))
	      {
		tmp.data[4] |= AD8402_Write (&hspi1, channel, value, TIMEOUT_SPI1_MS) << channel;
	      }
	  }
	CANCircularBuffer_enqueueMessage(&canTxBuffer,&tmp);
	break;
      }
    case AFECommand_writeGPIO:
      {
	GPIO_TypeDef *GPIOx = GetGPIOPortByEnumerator (msg.Data[2]);
	uint32_t GPIO_Pin = 1 << msg.Data[3];
	uint8_t PinState = msg.Data[4];
	machine_GPIO_WritePin (GPIOx, GPIO_Pin, PinState);
	tmp.data[2] = msg.Data[2];
	tmp.data[3] = msg.Data[3];
	tmp.data[4] = msg.Data[4];
	tmp.dlc = 5;
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }

      /* 0xC0 */
      /* Temperature loop runtime */
    case AFECommand_setTemperatureLoopForChannelState_byMask_asMask: // start or stop [Data[3] temperature loop for channel [Data[2]]
      {
	uint8_t channels = msg.Data[2];
	uint8_t status = msg.Data[3];
	tmp.data[2] = channels;
	tmp.data[3] = status;
	tmp.dlc = 2 + 2;
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
	  {
	    if (0x01 & (channels >> channel))
	      {
		regulatorSettings[channel].enabled = 0x01 & (status >> channel);
	      }
	  }
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setDACValueRaw_bySubdeviceMask: // set [Data[3] temperature loop for channel [Data[2]]
      {
	uint8_t channels = msg.Data[2];
	uint16_t value = 0;
	memcpy (&value, &msg.Data[3], sizeof(uint16_t));
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
	  {
	    if (0x01 & (channels >> channel))
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
	memcpy (&tmp.data[3], &value, sizeof(uint16_t));
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setDACValueSi_bySubdeviceMask: // set [Data[3] temperature loop for channel [Data[2]]
      {
	uint8_t channel = msg.Data[2];
	regulatorSettings[channel].enabled = msg.Data[3];
	float valueSi = 0.0;
	memcpy (&valueSi, &msg.Data[3], sizeof(float));
	/* FIXME Add function to convert DAC SI value to uint16_t */
	Error_Handler();
	uint16_t value = roundf (valueSi);
	switch (channel)
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
	  case AFECommandSubdevice_both:
	    {
	      machine_DAC_set (0x11, value);
	      break;
	    }
	  default:
	    break;
	  }
	tmp.data[2] = channel;
	tmp.data[3] = value;
	tmp.dlc = 2 + 1 + 1;
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_stopTemperatureLoopForAllChannels: // stop temperature loop for all channels
      {
	for (uint8_t channel = 0; channel < 2; ++channel)
	  {
	    regulatorSettings[channel].enabled = 0;
	  }
	tmp.data[2] = 1;
	tmp.dlc = 2 + 1;
	CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case AFECommand_setDAC_bySubdeviceMask_asMask: // Start or stop DAC
      {
	uint8_t channels = msg.Data[2];
	uint8_t status = msg.Data[3];
	tmp.data[2] = channels; // Copy mask
	tmp.data[3] = 0x00; // Clear status
	for (uint8_t channel = 0; channel < AFE_NUMBER_OF_SUBDEVICES; ++channel)
	  {
	    if (0x01 & (channels >> channel))
	      {
		tmp.data[3] |= (
		    machine_DAC_switch ((channel == 0) ? DAC_CHANNEL_1 : DAC_CHANNEL_2, status)
			== HAL_OK ? 1 : 0) << channel;
	      }
	  }
	tmp.dlc = 2 + 2;
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
	CANCircularBuffer_enqueueMessage_and_update_channelSettings_byMsg (
	    &canTxBuffer, &msg, &channelSettings[0], &tmp,
	    (size_t) &((s_channelSettings*) 0)->averaging_method, sizeof(e_average));
	break;
      }
    case AFECommand_setAveragingAlpha_byMask: // set averagingSettings.alpha
      {
	CANCircularBuffer_enqueueMessage_and_update_channelSettings_byMsg (
	    &canTxBuffer, &msg, &channelSettings[0], &tmp,
	    (size_t) &((s_channelSettings*) 0)->alpha, sizeof(float));
	break;
      }
    case AFECommand_setAveragingBufferSize_byMask: // set averagingSettings.buffer_size
      {
	CANCircularBuffer_enqueueMessage_and_update_channelSettings_byMsg (
	    &canTxBuffer, &msg, &channelSettings[0], &tmp,
	    (size_t) &((s_channelSettings*) 0)->buffer_ADC->buffer_size, sizeof(uint32_t));
	break;
      }
    case AFECommand_setChannel_dt_ms_byMask:
      {
	CANCircularBuffer_enqueueMessage_and_update_channelSettings_byMsg (
	    &canTxBuffer, &msg, &channelSettings[0], &tmp,
	    (size_t) &((s_channelSettings*) 0)->buffer_ADC->dt_ms, sizeof(uint32_t));
	break;
      }
    case AFECommand_setAveraging_max_dt_ms_byMask: // set averagingSettings.max_dt_ms
      {
	CANCircularBuffer_enqueueMessage_and_update_channelSettings_byMsg (
	    &canTxBuffer, &msg, &channelSettings[0], &tmp,
	    (size_t) &((s_channelSettings*) 0)->max_dt_ms, sizeof(uint32_t));
	break;
      }
    case AFECommand_setChannel_multiplicator_byMask: // set averagingSettings.multiplicator
      {
	CANCircularBuffer_enqueueMessage_and_update_channelSettings_byMsg (
	    &canTxBuffer, &msg, &channelSettings[0], &tmp,
	    (size_t) &((s_channelSettings*) 0)->multiplicator, sizeof(float));
	break;
      }
    case AFECommand_setAveragingSubdevice: // set averagingSettings.subdevice
      {
	update_buffer_by_channelSettings (&channelSettings[msg.Data[2]]);
	break;
      }
    case AFECommand_setChannel_a_byMask:
      {
	CANCircularBuffer_enqueueMessage_and_update_channelSettings_byMsg (
	    &canTxBuffer, &msg, &channelSettings[0], &tmp, (size_t) &((s_channelSettings*) 0)->a,
	    sizeof(float));
	break;
      }
    case AFECommand_setChannel_b_byMask:
      {
	CANCircularBuffer_enqueueMessage_and_update_channelSettings_byMsg (
	    &canTxBuffer, &msg, &channelSettings[0], &tmp, (size_t) &((s_channelSettings*) 0)->b,
	    sizeof(float));
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
	  here[i0] = get_average_atSettings(&channelSettings[i0],timestamp_now);
	}
    }
}

/***
 * Here is the temperature loop
 */
void __attribute__ ((optimize("-O3")))
machine_control (void)
{

  /* Calculate values for DAC -- active part of the Temperature Loop */
  for (uint8_t channel = 0; channel < 2; ++channel)
    {
      s_regulatorSettings *regulatorSettings_ptr = &regulatorSettings[channel];
      if (!regulatorSettings_ptr->enabled)
	{
	  continue;
	}
      uint32_t timestamp_ms = HAL_GetTick ();

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
	  regulatorSettings_ptr->T_old = average_Temperature;
#if DEBUG_SEND_BY_CAN_MACHINE_CONTROL
	  CAN_Message_t tmp;
	  tmp.id = CAN_ID_IN_MSG;
	  tmp.timestamp = HAL_GetTick (); // for timeout
	  tmp.data[0] = AFECommand_debug_machine_control;
	  // Voltage
	  tmp.data[1] = get_byte_of_message_number (0, 4);
	  tmp.data[2] = channel;
	  tmp.dlc = 2 + 1 + sizeof(float);
	  memcpy (&tmp.data[3], &voltage_for_SiPM, sizeof(float));
	  CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	  // Average temperature
	  tmp.data[1] = get_byte_of_message_number (1, 4);
	  tmp.data[2] = channel;
	  tmp.dlc = 2 + 1 + sizeof(float);
	  memcpy (&tmp.data[3], &average_Temperature, sizeof(float));
	  CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	  // Old temperature
	  tmp.data[1] = get_byte_of_message_number (2, 4);
	  tmp.data[2] = channel;
	  tmp.dlc = 2 + 1 + sizeof(float);
	  memcpy (&tmp.data[3], &regulatorSettings_ptr->T_old, sizeof(float));
	  CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	  // Timestamp
	  tmp.data[1] = get_byte_of_message_number (3, 4);
	  tmp.data[2] = channel;
	  tmp.dlc = 2 + 1 + sizeof(uint32_t);
	  memcpy (&tmp.data[3], &timestamp_ms, sizeof(uint32_t));
	  CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
#endif
	  regulatorSettings_ptr->ramp_target_voltage_set_bits =
	      machine_DAC_convert_mv_to_dac_value (voltage_for_SiPM);
	}
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
		  regulatorSettings_ptr->ramp_curent_voltage_set_bits +=
		      regulatorSettings_ptr->ramp_bit_step;
		}
	      else
		{
		  regulatorSettings_ptr->ramp_curent_voltage_set_bits -=
		      regulatorSettings_ptr->ramp_bit_step;
		}
	      if (regulatorSettings_ptr->ramp_curent_voltage_set_bits >= AFE_DAC_MAX)
		{
		  regulatorSettings_ptr->ramp_curent_voltage_set_bits = AFE_DAC_MAX;
		}
	      switch (channel)
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
	    }
	}
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
	      adc_value_real = get_average_atSettings (ptr, timestamp);
	      enqueueSensorDataSIandTimestamp (&canTxBuffer,
					       AFECommand_getSensorDataSi_all_periodic_average,
					       channel, adc_value_real, timestamp);
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

//	/* Start ADC in DMA mode */
	HAL_ADC_Start_DMA (&hadc, (uint32_t*) &adc_dma_buffer[0], 8);
	HAL_TIM_Base_Start (&htim1);
//	htim1.Instance->ARR = 50000-1;
	htim1.Instance->ARR = 10000 - 1;

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
	machine_periodic_report();
	break;
      }
    default:
      Error_Handler();
      break;
    }
}

static volatile s_ADC_Measurement _ADC_Measurement_HAL_ADC_ConvCpltCallback;
/* DMA Transfer Complete Callback */
void  __attribute__ ((optimize("-O3")))
HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
//    __DSB();  // Data Synchronization Barrier (ensures previous memory accesses complete)

    /* Store timestamp */
    _ADC_Measurement_HAL_ADC_ConvCpltCallback.timestamp_ms = HAL_GetTick();

    for (uint8_t i = 0; i < AFE_NUMBER_OF_CHANNELS; ++i)
    {
        /* Ensure the latest ADC value is read from memory (not a cached value) */
//        __DMB();  // Data Memory Barrier (ensures correct ordering of memory operations)

        _ADC_Measurement_HAL_ADC_ConvCpltCallback.adc_value = adc_dma_buffer[i];

        /* Ensure value is stored before proceeding */
//        __DSB();

        add_to_buffer(&bufferADC[i], &_ADC_Measurement_HAL_ADC_ConvCpltCallback);
    }

    /* Ensure ISR execution order is correct before exiting */
//    __ISB();  // Instruction Synchronization Barrier (ensures all instructions complete before next)
}

