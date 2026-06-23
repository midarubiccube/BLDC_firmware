#include "adc.h"


void init_adc(void) {
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    HAL_OPAMP_SelfCalibrate(&hopamp1);
	HAL_OPAMP_SelfCalibrate(&hopamp2);
	HAL_OPAMP_SelfCalibrate(&hopamp3);

    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

	HAL_ADCEx_MultiModeStart_DMA(&hadc1, dma_adc_buf, 2);
}