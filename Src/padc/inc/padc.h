#ifndef __PADC_H__
#define __PADC_H__

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_adc.h"

// channels
//#define PADC_CH0_Uout  0
//#define PADC_CH1_Uout  1
#define PADC_CH0_Usipm 2
#define PADC_CH1_Usipm 3
//#define PADC_CH0_I     4
//#define PADC_CH1_I     5
#define PADC_CH1_TEMP  6
#define PADC_CH0_TEMP  7

// exported functions
void PADC_init(ADC_HandleTypeDef *hadc);

#endif // __PADC_H__
