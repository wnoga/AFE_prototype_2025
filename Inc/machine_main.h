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

void config_adc_channels (ADC_HandleTypeDef *hadc);

void enqueueSubdeviceStatus (CAN_Message_t *reply, uint8_t masked_channel);
void machine_main_init_0 (void);
void machine_main (void);


#endif /* MACHINE_MAIN_H_ */
