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
uint16_t adc_dma_buffer[ADC_NUMBER_OF_CHANNELS];
s_ADC_Measurement adc_measurement_raw[ADC_NUMBER_OF_CHANNELS][ADC_MEASUREMENT_RAW_SIZE_MAX];

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

s_BufferADC bufferADC[ADC_NUMBER_OF_CHANNELS];

s_channelSettings channelSettings[ADC_NUMBER_OF_CHANNELS];

s_regulatorSettings regulatorSettings[2]; // Regulator settings for master and slave

typedef enum
{
  e_machine_main_init = 0,
  e_machine_main_idle

} e_machine_main;

volatile e_machine_main machine_main_status = e_machine_main_init;

uint32_t periodic_send_info_period_ms = 1500;
uint32_t periodic_send_info_start_ms = 0;

int8_t machnie_flag_averaging_enabled[ADC_NUMBER_OF_CHANNELS];
int8_t machine_temperatureLoop_enabled[2];

void update_buffer_by_averagingSettings(s_channelSettings *averagingSettings)
{
  averagingSettings->buffer_ADC->buffer_size = averagingSettings->buffer_size;
  averagingSettings->buffer_ADC->dt_ms = averagingSettings->dt_ms;
  averagingSettings->buffer_ADC->tail = averagingSettings->buffer_ADC->head = 0;
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

/***
 * Keep this to avoid mistake from values
 * channel 0x00 -> DAC_CHANNEL_1
 * channel 0x10 -> DAC_CHANNEL_2
 * channel 0x11 -> both
 */
static void
machine_DAC_switch (uint8_t channel, uint8_t enable)
{
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
}

/***
 * FIXME Create conversion from float to DAC uint16_t value
 */
static uint16_t
machine_DAC_convert_mv_to_dac_value (float mV)
{
  return mV;
}

void
machine_main_init_0 (void)
{
  machine_main_status = e_machine_main_init;
  for (uint8_t i0 = 0; i0 < ADC_NUMBER_OF_CHANNELS; ++i0)
    {
      channelSettings[i0].averaging_method = e_average_NONE;
      channelSettings[i0].alpha = 1.0;
      channelSettings[i0].buffer_ADC = &bufferADC[i0];
      channelSettings[i0].buffer_size = ADC_MEASUREMENT_RAW_SIZE_MAX;
      channelSettings[i0].max_dt_ms = 3600 * 1000; // 1 hour
      channelSettings[i0].multiplicator = 1.0;
      channelSettings[i0].max_N = ADC_MEASUREMENT_RAW_SIZE_MAX;

      machnie_flag_averaging_enabled[i0] = 0;

      init_buffer (&bufferADC[i0], &adc_measurement_raw[i0][0], channelSettings[i0].buffer_size,
		   channelSettings[i0].dt_ms);
    }

  for (uint8_t i0 = 0; i0 < 2; ++i0)
    {
      regulatorSettings[i0].T_0 = 25.0;
      regulatorSettings[i0].U_0 = 0.0;
      regulatorSettings[i0].U_cor = 0.0;
      regulatorSettings[i0].dT = 1.0;
      regulatorSettings[i0].dU = 1.0;
      regulatorSettings[i0].T_old = regulatorSettings[i0].T_0;
      regulatorSettings[i0].timestamp_ms_old = 0;
      regulatorSettings[i0].enabled = 0;
    }

  /* Append ADC channel to regulator setings */
  regulatorSettings[0].temperature_channelSettings_ptr = &channelSettings[7];
  regulatorSettings[1].temperature_channelSettings_ptr = &channelSettings[6];
}

static uint8_t
get_byte_of_message_number(uint8_t msg_index, uint8_t msg_index_max)
{
  if((msg_index > 15) | (msg_index_max > 15)) return 0;
  return (0xF0 & (msg_index_max << 4)) | ((msg_index+1) & 0x0F); // 0xF0 - total nr of msgs, 0x0F msg nr
}

static uint32_t value0;
void
can_execute (s_can_msg_recieved msg, CAN_HandleTypeDef *hcan)
{
  //  uint8_t tmp[8];
  CAN_Message_t tmp;
  tmp.timestamp = HAL_GetTick ();
  tmp.id = (1 << 10) | (CAN_ID << 2); // Master bit and own receiver ID
  tmp.data[0] = msg.Data[0]; // Standard reply [function]
  tmp.data[1] = 0x11; // Standard number of messages 1/1
		      // (0xF0 >> 4) = total number of messages in queue
		      // (0x0F) = message number (in the queue)
  tmp.dlc = 8;
//  blink1();
  switch ((uint8_t)msg.Data[0])
    {
    case 0x00: // getSerialNumber
      {
	tmp.id |= e_CANIdFunctionCode_multipleRead & 0b11;
	tmp.dlc = 2 + 4;
	for (uint8_t i0 = 0; i0 < 3; ++i0)
	  {
	    tmp.data[1] = get_byte_of_message_number(i0, 3);
	    // Copy the UID data, byte-by-byte
	    memcpy (&tmp.data[2], (uint8_t*) &UID[i0], sizeof(uint32_t));
	    CAN_EnqueueMessage (&canTxBuffer, &tmp);
	  }
	break;
      }
    case 0x01: // getVersion
      {
	tmp.dlc = 2 + verArrLen;
	memcpy (&tmp.data[2], &verArr[0], verArrLen);
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case 0x02: // setValue0
      {
	tmp.dlc = 2 + 1;
	value0 = *((uint32_t*) &msg.Data[2]);
	tmp.data[2] = 1;
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case 0x03: // resetAll
      {
	NVIC_SystemReset();
	break;
      }
      /* Send SI data [Data[0]] from ADC channel [Data[2]] as float value[Data[3:7]]  */
    case 0x30:
      {
	uint8_t channel = msg.Data[2];
	s_ADC_Measurement adc_val;
	get_n_latest_from_buffer(channelSettings[channel].buffer_ADC, 1, &adc_val);
	float adc_value_real = adc_val.adc_value * channelSettings[channel].multiplicator;
	tmp.data[2] = channel;
	tmp.dlc = 2 + 1 + sizeof(float);
	memcpy (&tmp.data[3], &adc_value_real, sizeof(float));
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
      }
      /* Send SI data [Data[0]] from ADC channel [Data[2]] as float average value[Data[3:7]]  */
    case 0x31:
      {
	uint8_t channel = msg.Data[2];
	float adc_value_real;
	get_average_atSettings (&channelSettings[channel], &adc_value_real);
	tmp.data[2] = channel;
	tmp.dlc = 2 + 1 + sizeof(float);
	memcpy (&tmp.data[3], &adc_value_real, sizeof(float));
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
      }
      /* Send SI data [Data[0]] from all channels as float average value[Data[3:7]]  */
    case 0x32:
      {
	for (uint8_t channel = 0; channel < ADC_NUMBER_OF_CHANNELS; ++channel)
	  {
	    float adc_value_real;
	    get_average_atSettings (&channelSettings[channel], &adc_value_real);
	    tmp.data[0] = 0x32;
	    tmp.data[1] = get_byte_of_message_number (channel, ADC_NUMBER_OF_CHANNELS);
	    tmp.data[2] = channel;
	    tmp.dlc = 2 + 1 + sizeof(float);
	    memcpy (&tmp.data[3], &adc_value_real, sizeof(float));
	    CAN_EnqueueMessage (&canTxBuffer, &tmp);
	  }
      }
      /* Temperature loop runtime */
    case 0xC0: // start or stop [Data[3] temperature loop for channel [Data[2]]
      {
	uint8_t channel = msg.Data[2];
	uint8_t status =  msg.Data[3];
//	regulatorSettings[channel].enabled = msg.Data[3];
	switch (channel)
	  {
	  case 0x00:
	    {
	      regulatorSettings[0].enabled = status;
//	      machine_DAC_switch (DAC_CHANNEL_1, regulatorSettings[channel].enabled);
	      break;
	    }
	  case 0x01:
	    {
	      regulatorSettings[1].enabled = status;
//	      machine_DAC_switch (DAC_CHANNEL_2, regulatorSettings[channel].enabled);
	      break;
	    }
	  case 0x03:
	    {
	      regulatorSettings[0].enabled = status;
	      regulatorSettings[1].enabled = status;
//	      machine_DAC_switch (0x11, regulatorSettings[channel].enabled);
	      break;
	    }
	  default:
	    break;
	  }
	tmp.data[2] = channel;
	tmp.data[3] = status;
	tmp.dlc = 2 + 1 + 1;
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case 0xC2: // set [Data[3] temperature loop for channel [Data[2]]
      {
	uint8_t channel = msg.Data[2];
	regulatorSettings[channel].enabled = msg.Data[3];
	uint16_t value = 0;
	memcpy (&value, &msg.Data[3], sizeof(uint16_t));
	switch (channel)
	  {
	  case 0x00:
	    {
	      machine_DAC_set (DAC_CHANNEL_1, value);
	      break;
	    }
	  case 0x01:
	    {
	      machine_DAC_set (DAC_CHANNEL_2, value);
	      break;
	    }
	  case 0x03:
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
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case 0xC3: // stop temperature loop for all channels
      {
	for (uint8_t channel = 0; channel < 2; ++channel)
	  {
	    regulatorSettings[channel].enabled = 0;
	  }
	tmp.data[2] = 1;
	tmp.dlc = 2 + 1;
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case 0xC4: // Start or stop DAC
      {
	uint8_t channel = msg.Data[2];
	uint8_t status =  msg.Data[3];
	switch (channel)
	  {
	  case 0x00:
	    {
	      machine_DAC_switch (DAC_CHANNEL_1, status);
	      break;
	    }
	  case 0x01:
	    {
	      machine_DAC_switch (DAC_CHANNEL_2, status);
	      break;
	    }
	  case 0x03:
	    {
	      machine_DAC_switch (DAC_CHANNEL_1, status);
	      machine_DAC_switch (DAC_CHANNEL_2, status);
	      break;
	    }
	  default:
	    break;
	  }
	tmp.data[2] = channel;
	tmp.data[3] = status;
	tmp.dlc = 2 + 1 + 1;
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
      /* Set setting [Data[0]] for adc channel [Data[2]] by uint32_t value[Data[3:7]]  */
    case 0xD0: // set averagingSettings.averaging_method
      {
	uint8_t channel = msg.Data[2];
	channelSettings[channel].averaging_method = (e_average) msg.Data[3];
	update_buffer_by_averagingSettings (&channelSettings[channel]);
	tmp.dlc = 2 + 1 + 1;
	tmp.data[2] = channel;
	tmp.data[3] = (uint8_t) channelSettings[channel].averaging_method;
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case 0xD1: // set averagingSettings.alpha
      {
	uint8_t channel = msg.Data[2];
	memcpy (&channelSettings[channel].alpha, &msg.Data[3], sizeof(float));
	update_buffer_by_averagingSettings(&channelSettings[channel]);
	tmp.dlc = 2 + 1 + 4;
	tmp.data[2] = channel;
	memcpy (&tmp.data[3], &channelSettings[channel].alpha, sizeof(float));
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case 0xD2: // set averagingSettings.buffer_size
      {
	uint8_t channel = msg.Data[2];
	memcpy (&channelSettings[channel].buffer_size, &msg.Data[3], sizeof(uint32_t));
	update_buffer_by_averagingSettings(&channelSettings[channel]);
	tmp.dlc = 2 + 1 + 4;
	tmp.data[2] = channel;
	memcpy (&tmp.data[3], &channelSettings[channel].buffer_size, 4);
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case 0xD3: // set averagingSettings.dt_ms
      {
	uint8_t channel = msg.Data[2];
	memcpy (&channelSettings[channel].dt_ms, &msg.Data[3], sizeof(uint32_t));
	update_buffer_by_averagingSettings(&channelSettings[channel]);
	tmp.dlc = 2 + 1 + 4;
	tmp.data[2] = channel;
	memcpy (&tmp.data[3], &channelSettings[channel].dt_ms, 4);
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case 0xD4: // set averagingSettings.max_dt_ms
      {
	uint8_t channel = msg.Data[2];
	memcpy (&channelSettings[channel].max_dt_ms, &msg.Data[3], sizeof(uint32_t));
	update_buffer_by_averagingSettings(&channelSettings[channel]);
	tmp.dlc = 2 + 1 + 4;
	tmp.data[2] = channel;
	memcpy (&tmp.data[3], &channelSettings[channel].max_dt_ms, 4);
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case 0xD5: // set averagingSettings.multiplicator
      {
	uint8_t channel = msg.Data[2];
	memcpy (&channelSettings[channel].multiplicator, &msg.Data[3], sizeof(uint32_t));
	update_buffer_by_averagingSettings(&channelSettings[channel]);
	tmp.dlc = 2 + 1 + 4;
	tmp.data[2] = channel;
	memcpy (&tmp.data[3], &channelSettings[channel].multiplicator, 4);
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    case 0xD6: // set averagingSettings.subdevice
      {
	update_buffer_by_averagingSettings(&channelSettings[msg.Data[2]]);
//	averagingSettings[msg.Data[2]].subdevice = (e_subdevice)msg.Data[3];
	break;
      }
    default:
      {
	tmp.dlc = 8;
	tmp.data[0] = 0x12;
	tmp.data[1] = 0x34;
	tmp.data[2] = 0x56;
	tmp.data[3] = 0x78;
	tmp.data[4] = 0x90;
	tmp.data[5] = 0xAB;
	tmp.data[6] = 0xCD;
	tmp.data[7] = 0xEF;
	CAN_EnqueueMessage (&canTxBuffer, &tmp);
	break;
      }
    }
}

void
machine_calculate_averageValues (float *here)
{
  uint32_t timestamp_now = HAL_GetTick ();
  for (uint8_t i0 = 0; i0 < ADC_NUMBER_OF_CHANNELS; ++i0)
    {
      if (machnie_flag_averaging_enabled[i0])
	{
	  here[i0] = get_average_atSettings(&channelSettings[i0], &here[i0]);
	}
    }
}

/***
 * Here is the temperature loop
 */
void
machine_control (void)
{
  uint32_t timestamp_ms = HAL_GetTick ();

  /* Calculate values for DAC -- active part of the Temperature Loop */
  for (uint8_t channel = 0; channel < 2; ++channel)
    {
      s_regulatorSettings *regulatorSettings_ptr = &regulatorSettings[channel];
      if (!regulatorSettings_ptr->enabled)
	{
	  continue;
	}

      float average_Temperature;
      if(get_average_atSettings(regulatorSettings_ptr->temperature_channelSettings_ptr, &average_Temperature) == 0)
	{
	  /* Skip if cannot calculate average */
	  continue;
	}

      if (fabsf (average_Temperature - regulatorSettings_ptr->T_old) >= regulatorSettings_ptr->dT)
	{
	  float voltage_for_SiPM = get_voltage_for_SiPM_x (average_Temperature,
							   regulatorSettings_ptr);
	  switch (channel)
	    {
	    case 0:
	      {
		machine_DAC_set ( DAC_CHANNEL_1,
				 machine_DAC_convert_mv_to_dac_value (voltage_for_SiPM));

		break;
	      }
	    case 1:
	      {
		machine_DAC_set ( DAC_CHANNEL_2,
				 machine_DAC_convert_mv_to_dac_value (voltage_for_SiPM));
		break;
	      }
	    default:
	      break;
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

	for (uint8_t i0 = 0; i0 < 20; ++i0)
	  {
	    HAL_Delay (50);
	    blink1 ();
	  }
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
	    can_execute (can_msg_received, &hcan);
	  }

	machine_control ();
	if((HAL_GetTick() - last_blink) > 250)
	  {
	    last_blink = HAL_GetTick();
	    blink1();
	  }
	break;
      }
    default:
      Error_Handler();
      break;
    }
}

static s_ADC_Measurement _ADC_Measurement_HAL_ADC_ConvCpltCallback;
/* DMA Transfer Complete Callback */
void
HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
  /* Append ADC values to buffers */
  _ADC_Measurement_HAL_ADC_ConvCpltCallback.timestamp_ms = HAL_GetTick ();
  for (uint8_t i = 0; i < ADC_NUMBER_OF_CHANNELS; ++i)
    {
      _ADC_Measurement_HAL_ADC_ConvCpltCallback.adc_value = adc_dma_buffer[i];
      add_to_buffer (&bufferADC[i], &_ADC_Measurement_HAL_ADC_ConvCpltCallback);
    }
}

