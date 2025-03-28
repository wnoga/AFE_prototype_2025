/*
 * machine_main.h
 *
 *  Created on: Feb 18, 2025
 *      Author: blondier94
 */

#ifndef MACHINE_MAIN_H_
#define MACHINE_MAIN_H_

#include "can_functions.h"

#define TIMEOUT_SPI1_MS 5000

#define USE_SHIFTING_IN_SetPinFromArray 0

#define DEBUG_SEND_BY_CAN_MACHINE_CONTROL 1
#define DEBUG_HARDWARE_CONTROL_DISABLED 0
#define HARDWARE_CONTROL_DAC_DISABLED 1
#define HARDWARE_CONTROL_SPI_DISABLED 1
#define HARDWARE_CONTROL_GPIO_DISABLED 0

uint8_t get_byte_of_message_number(uint8_t msg_index, uint8_t msg_index_max);

void machine_main_init_0 (void);
void machine_main (void);


#endif /* MACHINE_MAIN_H_ */
