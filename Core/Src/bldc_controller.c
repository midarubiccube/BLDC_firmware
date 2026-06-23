/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bldc_controller.c
  * @brief          : BLDC Motor Commutation Control Implementation
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bldc_controller.h"

/* Private variables ---------------------------------------------------------*/
BLDC_Controller_t bldc_controller = {
    .pwm_duty = 0,
    .commutation_period = 100,
    .state = COMMUTATION_STATE_1,
    .direction = 0,  // Forward
    .enabled = 0     // Disabled
};

static volatile uint32_t commutation_counter = 0;

/* Private function prototypes -----------------------------------------------*/
static void BLDC_ApplyCommutationState(BLDC_CommutationState_t state);
static void BLDC_SetPWMOutput(uint16_t channel, uint16_t duty);

/* Exported Functions -------------------------------------------------------*/

/**
  * @brief  Initialize BLDC Controller
  * @retval None
  */
void BLDC_Init(void)
{
       /* Initialize Timer 6 for commutation timing */
    HAL_TIM_Base_Start_IT(&htim6);
    
    /* Set initial commutation state */
    BLDC_Commutate(COMMUTATION_STATE_1);
    
    bldc_controller.enabled = 0;
}

/**
  * @brief  Set motor speed by PWM duty cycle
  * @param  speed_percent: Speed percentage (0-100%)
  * @retval None
  */
void BLDC_SetSpeed(uint16_t speed_percent)
{
    if (speed_percent > 100) {
        speed_percent = 100;
    }
    
    bldc_controller.pwm_duty = (speed_percent * BLDC_MAX_PWM) / 100;
}

/**
  * @brief  Set motor rotation direction
  * @param  direction: 0 = Forward, 1 = Reverse
  * @retval None
  */
void BLDC_SetDirection(uint8_t direction)
{
    bldc_controller.direction = direction ? 1 : 0;
}

/**
  * @brief  Enable BLDC motor
  * @retval None
  */
void BLDC_Enable(void)
{
    bldc_controller.enabled = 1;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

/**
  * @brief  Disable BLDC motor
  * @retval None
  */
void BLDC_Disable(void)
{
    bldc_controller.enabled = 0;
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}

/**
  * @brief  Execute commutation to specified state
  * @param  state: Target commutation state
  * @retval None
  */
void BLDC_Commutate(BLDC_CommutationState_t state)
{
    bldc_controller.state = state;
    BLDC_ApplyCommutationState(state);
}

/**
  * @brief  Force immediate commutation (manual commutation)
  * @retval None
  */
void BLDC_ForceCommutation(void)
{
    if (!bldc_controller.enabled) {
        return;
    }
    
    BLDC_CommutationState_t next_state;
    
    if (bldc_controller.direction == 0) {
        // Forward direction: 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 1
        next_state = (bldc_controller.state % 6) + 1;
    } else {
        // Reverse direction: 1 -> 6 -> 5 -> 4 -> 3 -> 2 -> 1
        next_state = (bldc_controller.state == 1) ? 6 : (bldc_controller.state - 1);
    }
    
    BLDC_Commutate(next_state);
}

/**
  * @brief  Set commutation period
  * @param  period_ms: Period in milliseconds
  * @retval None
  */
void BLDC_SetCommutationPeriod(uint16_t period_ms)
{
    bldc_controller.commutation_period = period_ms;
}

/**
  * @brief  Get current commutation state
  * @retval Current BLDC commutation state
  */
BLDC_CommutationState_t BLDC_GetCurrentState(void)
{
    return bldc_controller.state;
}

/* Private Functions --------------------------------------------------------*/

/**
  * @brief  Apply commutation state by controlling PWM and GPIO
  * @param  state: Target commutation state
  * @retval None
  */
static void BLDC_ApplyCommutationState(BLDC_CommutationState_t state)
{
    if (!bldc_controller.enabled) {
        return;
    }
    
    uint16_t duty = bldc_controller.pwm_duty;
    
    /* 120-degree commutation sequence (CH1, CH2, CH4) */
    switch (state) {
        case COMMUTATION_STATE_1:
            // U-phase: PWM, V-phase: GND, W-phase: OFF
            BLDC_SetPWMOutput(TIM_CHANNEL_1, duty);
            BLDC_SetPWMOutput(TIM_CHANNEL_2, 0);
            BLDC_SetPWMOutput(TIM_CHANNEL_4, 0);
            break;
            
        case COMMUTATION_STATE_2:
            // U-phase: PWM, W-phase: GND, V-phase: OFF
            BLDC_SetPWMOutput(TIM_CHANNEL_1, duty);
            BLDC_SetPWMOutput(TIM_CHANNEL_2, 0);
            BLDC_SetPWMOutput(TIM_CHANNEL_4, 0);
            break;
            
        case COMMUTATION_STATE_3:
            // V-phase: PWM, W-phase: GND, U-phase: OFF
            BLDC_SetPWMOutput(TIM_CHANNEL_1, 0);
            BLDC_SetPWMOutput(TIM_CHANNEL_2, duty);
            BLDC_SetPWMOutput(TIM_CHANNEL_4, 0);
            break;
            
        case COMMUTATION_STATE_4:
            // V-phase: PWM, U-phase: GND, W-phase: OFF
            BLDC_SetPWMOutput(TIM_CHANNEL_1, 0);
            BLDC_SetPWMOutput(TIM_CHANNEL_2, duty);
            BLDC_SetPWMOutput(TIM_CHANNEL_4, 0);
            break;
            
        case COMMUTATION_STATE_5:
            // W-phase: PWM, U-phase: GND, V-phase: OFF
            BLDC_SetPWMOutput(TIM_CHANNEL_1, 0);
            BLDC_SetPWMOutput(TIM_CHANNEL_2, 0);
            BLDC_SetPWMOutput(TIM_CHANNEL_4, duty);
            break;
            
        case COMMUTATION_STATE_6:
            // W-phase: PWM, V-phase: GND, U-phase: OFF
            BLDC_SetPWMOutput(TIM_CHANNEL_1, 0);
            BLDC_SetPWMOutput(TIM_CHANNEL_2, 0);
            BLDC_SetPWMOutput(TIM_CHANNEL_4, duty);
            break;
            
        default:
            break;
    }
}

/**
  * @brief  Set PWM output duty cycle for specified channel
  * @param  channel: Timer channel (TIM_CHANNEL_1, TIM_CHANNEL_2, etc.)
  * @param  duty: PWM duty cycle (0 - ARR)
  * @retval None
  */
static void BLDC_SetPWMOutput(uint16_t channel, uint16_t duty)
{
    __HAL_TIM_SET_COMPARE(&htim3, channel, duty);
}

/* Timer Interrupt Handler for Commutation --------------------------------*/

/**
  * @brief  Timer 8 Period Elapsed Callback (Commutation Trigger)
  * @param  htim: Timer handle
  * @retval None
  */
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        commutation_counter++;
        
        // Trigger commutation at specified period
        if (commutation_counter >= bldc_controller.commutation_period) {
            if (bldc_controller.enabled) {
                BLDC_ForceCommutation();
            }
            commutation_counter = 0;
        }
    }
}*/
