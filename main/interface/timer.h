#pragma once

#include "tim.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void init_timer(void);
void set_timer_task(void (*task)());