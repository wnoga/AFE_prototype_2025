#include "inc/pdac.h"

//extern DAC_HandleTypeDef hdac;

static const uint32_t PDAC_CH_ADDR[PDAC_CH_NR] = {
		                                           DAC_CHANNEL_1,
		                                           DAC_CHANNEL_2
                                                 };

//INIT
void PDAC_init(DAC_HandleTypeDef *hdac)
{
    DAC_ChannelConfTypeDef sConfig = {0};

    hdac->Instance = DAC;
    if (HAL_DAC_Init(hdac) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

    if (HAL_DAC_ConfigChannel(hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_DAC_ConfigChannel(hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    // set default value
    PDAC_setValCh(hdac, DAC_CHANNEL_1, PDAC_DEFAULT_VAL);
    PDAC_setValCh(hdac, DAC_CHANNEL_2, PDAC_DEFAULT_VAL);

}

void PDAC_startCh(DAC_HandleTypeDef *hdac, uint32_t ch)
{
    if (HAL_DAC_Start(hdac, PDAC_CH_ADDR[ch]) != HAL_OK)
    {
        Error_Handler();
    }
}

void PDAC_startChAll(DAC_HandleTypeDef *hdac)
{
	for (uint32_t i = 0; i < PDAC_CH_NR; i++)
	{
		PDAC_startCh(hdac, i);
	}
}

void PDAC_setValCh(DAC_HandleTypeDef *hdac, uint32_t ch, uint32_t data)
{
	if (HAL_DAC_SetValue(hdac, PDAC_CH_ADDR[ch], PDAC_WORD_CONF, data) != HAL_OK)
	{
		Error_Handler();
	}
}

void PDAC_setValChAll(DAC_HandleTypeDef *hdac, sipmCtrlLoop_t *structPtr)
// use HAL_DACEx_DualSetValue -- RM0091 (p. 288)
{
	for (uint32_t i = 0; i < PDAC_CH_NR; i++)
	{
		PDAC_setValCh(hdac, i, (uint32_t) (structPtr+i)->vSet);
	}
}

// for get also use HAL_DACEx_DualGetValue

