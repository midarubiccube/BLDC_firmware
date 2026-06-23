/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bldc_controller.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "main_app.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_FULL_SCALE_COUNTS 4095.0f
#define ADC_REFERENCE_VOLTAGE 3.3f
#define SHUNT_RESISTANCE_OHMS 0.01f
#define OPAMP_GAIN 64.0f




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart4,(uint8_t *)ptr,len,10);
  return len;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void FOC_SVPWM_Update(float V_alpha, float V_beta) {
    
    // 1. 逆Clarke変換 (alpha, beta -> U, V, W)
    // Va = Valpha
    // Vb = -0.5 * Valpha + (sqrt(3)/2) * Vbeta
    // Vc = -0.5 * Valpha - (sqrt(3)/2) * Vbeta
    
    // sqrt(3)/2 = 0.8660254f
    float Va = V_alpha;
    float Vb = -0.5f * V_alpha + 0.8660254f * V_beta;
    float Vc = -0.5f * V_alpha - 0.8660254f * V_beta;

    // 2. Min-Max法によるSVPWM (コモンモード電圧の注入)
    // 3相の中で一番大きい電圧と小さい電圧を見つける
    float V_max = fmaxf(Va, fmaxf(Vb, Vc));
    float V_min = fminf(Va, fminf(Vb, Vc));
    
    // 鞍型にするためのオフセット電圧
    float V_common = -0.5f * (V_max + V_min);

    // 3. オフセットを加算してデューティ比に変換
    // 入力(-1.0~1.0) -> CCR(0~ARR)
    // Center Alignedなので、duty 0.0 が中心、-1.0が0、+1.0がARRになるようにマップする
    // 式: CCR = ( (V + V_common) + 1.0 ) * 0.5 * ARR
    
    // クリップ処理 (過変調防止)
    // ※厳密には正規化が必要ですが、テストなので簡易リミッタで
    float u_out = Va + V_common;
    float v_out = Vb + V_common;
    float w_out = Vc + V_common;
    
    // レジスタ書き込み
    TIM3->CCR1 = (uint32_t)((u_out + 1.0f) * 0.5f * PWM_PERIOD_COUNTS);
    TIM3->CCR2 = (uint32_t)((v_out + 1.0f) * 0.5f * PWM_PERIOD_COUNTS);
    TIM3->CCR4 = (uint32_t)((w_out + 1.0f) * 0.5f * PWM_PERIOD_COUNTS);

    TIM1->CCR1 = (uint32_t)((u_out + 1.0f) * 0.5f * PWM_PERIOD_COUNTS);
    TIM1->CCR2 = (uint32_t)((v_out + 1.0f) * 0.5f * PWM_PERIOD_COUNTS);
    TIM1->CCR4 = (uint32_t)((w_out + 1.0f) * 0.5f * PWM_PERIOD_COUNTS);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_FDCAN2_Init();
  MX_TIM8_Init();
  MX_I2C3_Init();
  MX_UART4_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
  main_setup();
  float theta = 0.0f;
  float voltage_amp = 0.05f; // 電圧振幅 0.8 (最大1.0だが安全マージン)
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // 1. 角度を進める (50Hz程度で回転)
    theta += 0.1f; 
    if (theta > 6.283185f) theta -= 6.283185f;

    // 2. 電圧ベクトル生成 (逆Park変換の簡易版)
    // 本来は Id, Iq から計算しますが、テストなので直接 Alpha, Beta を生成
    float valpha = voltage_amp * cosf(theta);
    float vbeta  = voltage_amp * sinf(theta);

    // 3. SVPWM実行
    FOC_SVPWM_Update(valpha, vbeta);

    // 4. 結果確認 (CCRレジスタの値をプロット)
    // ExcelやSerialPlotterで波形を確認してください
    HAL_Delay(1); // 適当なウェイト
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
