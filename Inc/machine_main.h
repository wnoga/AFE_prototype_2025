/*
 * machine_main.h
 *
 *  Created on: Feb 18, 2025
 *      Author: blondier94
 */

#ifndef MACHINE_MAIN_H_
#define MACHINE_MAIN_H_

#include "settings.h"
#include "AFE_functions.h" // For s_channelSettings, s_regulatorSettings, etc.

#include "can_functions.h"

typedef enum
{
  e_machine_main_init = 0,
  e_machine_main_idle

} e_machine_main;

// Expose globals for testing purposes
extern volatile e_machine_main machine_main_status;
extern s_channelSettings afe_channelSettings[AFE_NUMBER_OF_CHANNELS];
extern s_regulatorSettings afe_regulatorSettings[AFE_NUMBER_OF_SUBDEVICES];
extern s_BufferADC bufferADC[AFE_NUMBER_OF_CHANNELS];
extern int8_t machnie_flag_averaging_enabled[AFE_NUMBER_OF_CHANNELS];
extern s_ADC_Measurement adc_measurement_raw[AFE_NUMBER_OF_CHANNELS][ADC_MEASUREMENT_RAW_SIZE_MAX];

void enqueueSubdeviceStatus (CAN_Message_t *reply, uint8_t masked_channel);
void machine_main_init_0 (void);
void machine_main (void);


#endif /* MACHINE_MAIN_H_ */
