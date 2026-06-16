#pragma once

#include <stdint.h>

#define PWM_PERIOD_COUNTS 4250.0f

void main_setup(void);
void main_loop(void);

void disable_irq_nest();
void enable_irq_nest();

static uint32_t irq_cnt = 0;

static volatile uint16_t raw_currents[3];
static uint32_t dma_adc_buf[2];
static float adc_current_offsets[3] = {0.0f, 0.0f, 0.0f};
static volatile uint8_t adc2_rank_index = 0;