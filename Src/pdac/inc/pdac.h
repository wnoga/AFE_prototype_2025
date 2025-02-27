#ifndef __PDAC_H__
#define __PDAC_H__

#include "nica.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_dac.h"

#define PDAC_CH_NR 2
#define PDAC_WORD_CONF DAC_ALIGN_12B_R
#define PDAC_DEFAULT_VAL 4095

// exported functions
void PDAC_init(DAC_HandleTypeDef *hdac);
void PDAC_startCh(DAC_HandleTypeDef *hdac, uint32_t ch);
void PDAC_startChAll(DAC_HandleTypeDef *hdac);
void PDAC_setValCh(DAC_HandleTypeDef *hdac, uint32_t ch, uint32_t data);
void PDAC_setValChAll(DAC_HandleTypeDef *hdac, sipmCtrlLoop_t *sipmCtrlLoop);

#endif // __PDAC_H__
