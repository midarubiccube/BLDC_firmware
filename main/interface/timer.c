#include "timer.h"


static void (*controll_task)() = NULL;


void init_timer(void) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim1);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

void set_timer_task(void (*task)()) {
    controll_task = task;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {
        controll_task();
	} else if (htim == &htim3) {
		
	}
}





