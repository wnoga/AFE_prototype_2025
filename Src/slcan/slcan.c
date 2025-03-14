/*
 * slcan_interface.c
 *
 *  Created on: Apr 2, 2016
 *      Author: Vostro1440
 */

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"
#include "slcan.h"
#include "slcan_additional.h"

// internal slcan_interface state
static uint8_t state = STATE_CONFIG;

extern CAN_HandleTypeDef hcan;

void
StartCan (void)
{

  HAL_NVIC_DisableIRQ (CEC_CAN_IRQn);
  state = STATE_CONFIG;

  if (state == STATE_CONFIG) slcanSetCANBaudRate (CAN_BR_100K);

  hcan.Init.Mode = CAN_MODE_NORMAL;

  if (CANInit () == HAL_OK)
    {
      HAL_NVIC_EnableIRQ (CEC_CAN_IRQn);
      state = STATE_OPEN;
    }
}






