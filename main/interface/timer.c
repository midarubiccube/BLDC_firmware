#include "timer.h"

#include "tim.h"

void init_timer(void) {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {
		esc_callControlTask();
	} else if (htim == &htim2) {
		disable_irq_nest();
		us_upper32++;
		enable_irq_nest();
	} else if (htim == &htim3) {
		if (task_1kHz != NULL) task_1kHz();
	}
}





