#include "main_app.h"

#include "main.h"


void main_setup(void){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); // Example: Turn off onboard LED
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    HAL_OPAMP_SelfCalibrate(&hopamp1);
	HAL_OPAMP_SelfCalibrate(&hopamp2);
	HAL_OPAMP_SelfCalibrate(&hopamp3);

    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

	HAL_ADCEx_MultiModeStart_DMA(&hadc1, dma_adc_buf, 2);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_1, 0.5f * PWM_PERIOD_COUNTS);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_2, 0.5f * PWM_PERIOD_COUNTS);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_4, 0.5f * PWM_PERIOD_COUNTS +  10);
	HAL_Delay(1); // 0.1秒待機
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_4, 0);
}

void main_loop(void){}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc == &hadc1) {
		disable_irq_nest();
		raw_currents[0] = (uint16_t)(dma_adc_buf[0] & 0xFFFF);
		raw_currents[1] = (uint16_t)((dma_adc_buf[0] >> 16) & 0xFFFF);
		raw_currents[2] = (uint16_t)((dma_adc_buf[1] >> 16) & 0xFFFF);
		enable_irq_nest();
	}
}

void disable_irq_nest() {
	if (irq_cnt == 0) __disable_irq();
	if (irq_cnt < UINT32_MAX) irq_cnt++;
}

void enable_irq_nest() {
	if (irq_cnt > 0) irq_cnt--;
	if (irq_cnt == 0) __enable_irq();
}

