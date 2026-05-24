/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bldc_controller.h
  * @brief          : BLDC Motor Commutation Control Header
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BLDC_CONTROLLER_H__
#define __BLDC_CONTROLLER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* BLDC Control Definitions */
#define BLDC_PWM_FREQUENCY     20000  // 20kHz
#define BLDC_MIN_PWM           0
#define BLDC_MAX_PWM           1000
#define SINE_TABLE_SIZE        256    // 256-step sine table

/* BLDC Controller Structure */
typedef struct {
    uint16_t pwm_duty;              // PWM Duty Cycle (0-1000)
    uint8_t angle;                  // Motor electrical angle (0-255 = 0-360°)
    uint8_t direction;              // 0: Forward, 1: Reverse
    uint8_t enabled;                // 0: Disabled, 1: Enabled
} BLDC_Controller_t;

/* Function Prototypes */
void BLDC_Init(void);
void BLDC_SetSpeed(uint16_t speed_percent);
void BLDC_SetDirection(uint8_t direction);
void BLDC_SetAngle(uint8_t angle);        // Set electrical angle (0-255 = 0-360°)
void BLDC_SetAngleIncrement(uint8_t inc); // Set angle increment per commutation period
void BLDC_Enable(void);
void BLDC_Disable(void);

extern BLDC_Controller_t bldc_controller;

#ifdef __cplusplus
}
#endif

#endif /* __BLDC_CONTROLLER_H__ */
