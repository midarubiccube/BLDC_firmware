/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bldc_controller.c
  * @brief          : BLDC Motor Sinusoidal Commutation Control Implementation
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bldc_controller.h"
#include <math.h>

/* Sine Wave Lookup Table (256 steps, 0-1000 amplitude) */
static const uint16_t sine_table[256] = {
    500,  503,  506,  509,  512,  515,  518,  521,  524,  527,  530,  533,  536,  539,  542,  545,
    548,  551,  554,  557,  560,  563,  566,  569,  572,  575,  578,  581,  584,  587,  590,  593,
    596,  599,  602,  605,  608,  611,  614,  617,  620,  623,  626,  628,  631,  634,  637,  640,
    643,  646,  648,  651,  654,  657,  660,  662,  665,  668,  670,  673,  676,  678,  681,  683,
    686,  689,  691,  694,  696,  698,  701,  703,  706,  708,  710,  713,  715,  717,  719,  722,
    724,  726,  728,  730,  732,  734,  736,  738,  740,  742,  743,  745,  747,  749,  750,  752,
    754,  755,  757,  758,  760,  761,  762,  764,  765,  766,  768,  769,  770,  771,  772,  773,
    774,  775,  776,  777,  778,  778,  779,  780,  780,  781,  781,  782,  782,  782,  783,  783,
    783,  783,  783,  782,  782,  782,  781,  781,  780,  780,  779,  778,  778,  777,  776,  775,
    774,  773,  772,  771,  770,  769,  768,  766,  765,  764,  762,  761,  760,  758,  757,  755,
    754,  752,  750,  749,  747,  745,  743,  742,  740,  738,  736,  734,  732,  730,  728,  726,
    724,  722,  719,  717,  715,  713,  710,  708,  706,  703,  701,  698,  696,  694,  691,  689,
    686,  683,  681,  678,  676,  673,  670,  668,  665,  662,  660,  657,  654,  651,  648,  646,
    643,  640,  637,  634,  631,  628,  626,  623,  620,  617,  614,  611,  608,  605,  602,  599,
    596,  593,  590,  587,  584,  581,  578,  575,  572,  569,  566,  563,  560,  557,  554,  551,
    548,  545,  542,  539,  536,  533,  530,  527,  524,  521,  518,  515,  512,  509,  506,  503
};

/* Private variables ---------------------------------------------------------*/
BLDC_Controller_t bldc_controller = {
    .pwm_duty = 0,
    .angle = 0,           // 0-255 = 0-360°
    .direction = 0,       // Forward
    .enabled = 0          // Disabled
};

static volatile uint8_t angle_increment = 0;  // Angle step per commutation period

/* Private function prototypes -----------------------------------------------*/
static uint16_t BLDC_GetSineValue(uint8_t angle);
static void BLDC_UpdateSineWavePWM(void);

/* Exported Functions -------------------------------------------------------*/

/**
  * @brief  Initialize BLDC Controller
  * @retval None
  */
void BLDC_Init(void)
{
    /* Initialize Timer 3 for PWM output (CH1, CH2, CH4) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    
    /* Initialize Timer 6 for timing interrupt */
    HAL_TIM_Base_Start_IT(&htim6);
    
    /* Initialize angle to 0 */
    bldc_controller.angle = 0;
    bldc_controller.enabled = 0;
    
    /* Update PWM with initial angle */
    BLDC_UpdateSineWavePWM();
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
  * @brief  Set motor electrical angle
  * @param  angle: Electrical angle (0-255 = 0-360°)
  * @retval None
  */
void BLDC_SetAngle(uint8_t angle)
{
    bldc_controller.angle = angle;
    BLDC_UpdateSineWavePWM();
}

/**
  * @brief  Set angle increment per commutation period
  * @param  inc: Angle increment (0-255)
  * @retval None
  */
void BLDC_SetAngleIncrement(uint8_t inc)
{
    angle_increment = inc;
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
    
    /* Set all PWM to 0 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

/* Private Functions --------------------------------------------------------*/

/**
  * @brief  Get sine value from lookup table
  * @param  angle: Angle (0-255 = 0-360°)
  * @retval Sine value (0-1000)
  */
static uint16_t BLDC_GetSineValue(uint8_t angle)
{
    uint16_t value = sine_table[angle];
    
    /* Apply duty cycle scaling */
    value = (value * bldc_controller.pwm_duty) / 1000;
    
    /* Convert to PWM compare value (0-1000) */
    value = (value * 8500) / 1000;
    
    return value;
}

/**
  * @brief  Update PWM outputs with sinusoidal commutation
  * @retval None
  */
static void BLDC_UpdateSineWavePWM(void)
{
    if (!bldc_controller.enabled) {
        return;
    }
    
    uint8_t angle = bldc_controller.angle;
    
    /* Calculate angles for 3-phase (120° phase shift) */
    uint8_t angle_u = angle;                    // U-phase at angle
    uint8_t angle_v = angle + 85;               // V-phase at angle + 120° (85 ≈ 256/3)
    uint8_t angle_w = angle + 170;              // W-phase at angle + 240° (170 ≈ 2*256/3)
    
    /* Apply direction */
    if (bldc_controller.direction == 1) {
        // Reverse: negate phase sequence
        angle_v = angle - 85;
        angle_w = angle - 170;
    }
    
    /* Get sine values and set PWM */
    uint16_t pwm_u = BLDC_GetSineValue(angle_u);
    uint16_t pwm_v = BLDC_GetSineValue(angle_v);
    uint16_t pwm_w = BLDC_GetSineValue(angle_w);
    
    /* Limit to maximum compare value */
    if (pwm_u > 1000) pwm_u = 1000;
    if (pwm_v > 1000) pwm_v = 1000;
    if (pwm_w > 1000) pwm_w = 1000;
    
    /* Update PWM compare registers */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_v);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_w);
}

/* Interrupt Callback -------------------------------------------------------*/

/**
  * @brief  Timer interrupt callback - Called by HAL_TIM_PeriodElapsedCallback
  *         Increment angle automatically when enabled
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        if (bldc_controller.enabled && angle_increment > 0) {
            /* Update angle */
            bldc_controller.angle += angle_increment;
            
            /* Update PWM with new angle */
            BLDC_UpdateSineWavePWM();
        }
    }
}
