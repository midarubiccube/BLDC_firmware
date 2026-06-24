#include "main_app.h"

#include "main.h"
#include <math.h>

#include "interface/adc.h"
#include "interface/timer.h"

float theta = 0.0f;
float voltage_amp = 0.05f; // 電圧振幅 0.8 (最大1.0だが安全マージン)

void FOC_SVPWM_Update() {
    theta += 0.01f; 
    if (theta > 6.283185f) theta -= 6.283185f;

    // 2. 電圧ベクトル生成 (逆Park変換の簡易版)
    // 本来は Id, Iq から計算しますが、テストなので直接 Alpha, Beta を生成
    float valpha = voltage_amp * cosf(theta);
    float vbeta  = voltage_amp * sinf(theta);
    
    // 1. 逆Clarke変換 (alpha, beta -> U, V, W)
    // Va = Valpha
    // Vb = -0.5 * Valpha + (sqrt(3)/2) * Vbeta
    // Vc = -0.5 * Valpha - (sqrt(3)/2) * Vbeta
    
    // sqrt(3)/2 = 0.8660254f
    float Va = valpha;
    float Vb = -0.5f * valpha + 0.8660254f * vbeta;
    float Vc = -0.5f * valpha - 0.8660254f * vbeta;

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

void main_setup(void){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); // Example: Turn off onboard LED


	init_adc();
	init_timer();
	set_timer_task(FOC_SVPWM_Update);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_1, 0.5f * PWM_PERIOD_COUNTS);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_2, 0.5f * PWM_PERIOD_COUNTS);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_4, 0.5f * PWM_PERIOD_COUNTS +  10);
	HAL_Delay(1); // 0.1秒待機
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_1, 0.5f * PWM_PERIOD_COUNTS);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_2, 0.5f * PWM_PERIOD_COUNTS);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_4, 0.5f * PWM_PERIOD_COUNTS);


}


void disable_irq_nest() {
	if (irq_cnt == 0) __disable_irq();
	if (irq_cnt < UINT32_MAX) irq_cnt++;
}

void enable_irq_nest() {
	if (irq_cnt > 0) irq_cnt--;
	if (irq_cnt == 0) __enable_irq();
}

