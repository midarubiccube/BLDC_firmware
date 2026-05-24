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
#define BLDC_MAX_PWM           500

/* Commutation Sequence States (120° commutation) */
typedef enum {
    COMMUTATION_STATE_1 = 1,  // U phase: PWM, V phase: GND, W phase: Floating
    COMMUTATION_STATE_2 = 2,  // U phase: PWM, W phase: GND, V phase: Floating
    COMMUTATION_STATE_3 = 3,  // V phase: PWM, W phase: GND, U phase: Floating
    COMMUTATION_STATE_4 = 4,  // V phase: PWM, U phase: GND, W phase: Floating
    COMMUTATION_STATE_5 = 5,  // W phase: PWM, U phase: GND, V phase: Floating
    COMMUTATION_STATE_6 = 6   // W phase: PWM, V phase: GND, U phase: Floating
} BLDC_CommutationState_t;

/* BLDC Controller Structure */
typedef struct {
    uint16_t pwm_duty;              // PWM Duty Cycle (0-65535)
    uint16_t commutation_period;    // Period between commutations (ms)
    BLDC_CommutationState_t state;  // Current commutation state
    uint8_t direction;              // 0: Forward, 1: Reverse
    uint8_t enabled;                // 0: Disabled, 1: Enabled
} BLDC_Controller_t;

/* Function Prototypes */
void BLDC_Init(void);
void BLDC_SetSpeed(uint16_t speed_percent);
void BLDC_SetDirection(uint8_t direction);
void BLDC_Enable(void);
void BLDC_Disable(void);
void BLDC_Commutate(BLDC_CommutationState_t state);
void BLDC_ForceCommutation(void);
void BLDC_SetCommutationPeriod(uint16_t period_ms);
BLDC_CommutationState_t BLDC_GetCurrentState(void);

extern BLDC_Controller_t bldc_controller;

#ifdef __cplusplus
}
#endif

#endif /* __BLDC_CONTROLLER_H__ */
