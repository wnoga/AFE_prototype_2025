/*
 * machine_main.h
 *
 *  Created on: Feb 18, 2025
 *      Author: blondier94
 */

#ifndef MACHINE_MAIN_H_
#define MACHINE_MAIN_H_

#include "can_functions.h"

#define ADC_MEASUREMENT_RAW_SIZE_MAX 64
#define ADC_NUMBER_OF_CHANNELS 8

void machine_main_init_0 (void);
void machine_main (void);

#endif /* MACHINE_MAIN_H_ */
