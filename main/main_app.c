#include "main_app.h"

#include "main.h"

#include "interface/adc.h"
#include "interface/timer.h"


void main_setup(void){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); // Example: Turn off onboard LED

	init_adc();
	init_timer();

	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_1, 0.5f * PWM_PERIOD_COUNTS);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_2, 0.5f * PWM_PERIOD_COUNTS);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_4, 0.5f * PWM_PERIOD_COUNTS +  10);
	HAL_Delay(1); // 0.1秒待機
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_4, 0);

}


void disable_irq_nest() {
	if (irq_cnt == 0) __disable_irq();
	if (irq_cnt < UINT32_MAX) irq_cnt++;
}

void enable_irq_nest() {
	if (irq_cnt > 0) irq_cnt--;
	if (irq_cnt == 0) __enable_irq();
}

