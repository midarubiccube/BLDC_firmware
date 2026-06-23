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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc == &hadc1) {
		disable_irq_nest();
		raw_currents[0] = (uint16_t)(dma_adc_buf[0] & 0xFFFF);
		raw_currents[1] = (uint16_t)((dma_adc_buf[0] >> 16) & 0xFFFF);
		raw_currents[2] = (uint16_t)((dma_adc_buf[1] >> 16) & 0xFFFF);
		enable_irq_nest();
	}
}