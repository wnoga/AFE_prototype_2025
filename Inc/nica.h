/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NICA_H__
#define __NICA_H__

#include "stm32f0xx_hal.h"

// main structure for control loop
#define TEMPFIL_BUF_LEN 128
typedef struct
{
    uint16_t vSet;
    uint16_t vDelta;
    uint16_t tDelta;
    uint16_t filLen;
    uint16_t tempBuf[TEMPFIL_BUF_LEN];
    uint16_t vSetAct;
    uint16_t tSetAct;
    uint32_t sumFil;
    uint16_t avgFil;
    uint16_t filNewPtr;
    uint16_t filOldPtr;
} sipmCtrlLoop_t;

#endif
