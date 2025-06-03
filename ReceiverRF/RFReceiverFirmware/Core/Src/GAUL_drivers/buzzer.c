/*
 * buzzer.c
 *
 *  Created on: Jun 1, 2025
 *      Author: victo
 */
#include "GAUL_drivers/buzzer.h"

#include "stm32f1xx_hal.h"  // Adjust if you use a different STM32 series

// Static variables to store the buzzer’s timer handle and channel
static TIM_HandleTypeDef *buzzer_timer = NULL;
static TIM_HandleTypeDef *buzzer_watcher = NULL;
static uint32_t         buzzer_channel = 0;
static uint32_t			watcher_channel = 0;
static uint32_t time;


void buzzer_init(TIM_HandleTypeDef *htim_buz, uint32_t buz_channel, TIM_HandleTypeDef *htim_watch, uint32_t watch_channel)
{
    buzzer_timer = htim_buz;
    buzzer_channel = buz_channel;
    buzzer_watcher = htim_watch;
    watcher_channel = watch_channel;

    TIM_OC_InitTypeDef sConfigOC = {0};

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;  // 0% duty initially
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(buzzer_timer, &sConfigOC, buzzer_channel);
    HAL_TIM_PWM_Start(buzzer_timer, buzzer_channel);
}

void buzzer_beep(uint32_t frequency, uint32_t duration_ms)
{
    if (buzzer_timer == NULL) return; // not initialized

    uint32_t timer_clock = HAL_RCC_GetPCLK2Freq();  // usually APB2 for TIM1
    if (buzzer_timer->Instance != TIM1) {
        timer_clock = HAL_RCC_GetPCLK1Freq();       // fallback for other timers
    }

    // Calculate period and prescaler
    uint32_t prescaler = (timer_clock / (frequency * 1000)) - 1;
    uint32_t period = 1000 - 1;  // Fixed 1 kHz timer base, to simplify duty control

    buzzer_timer->Instance->PSC = prescaler;
    buzzer_timer->Instance->ARR = period;
    buzzer_timer->Instance->CCR1 = period / 2;  // 50% duty (for CH1; adjust if CH2,3,4)

    __HAL_TIM_SET_COMPARE(buzzer_timer, buzzer_channel, period / 2);
    __HAL_TIM_SET_AUTORELOAD(buzzer_timer, period);
    __HAL_TIM_SET_PRESCALER(buzzer_timer, prescaler);

    HAL_TIM_PWM_Start(buzzer_timer, buzzer_channel);

    //start counter
    __HAL_TIM_SET_COMPARE(buzzer_watcher, watcher_channel, duration_ms);
    HAL_TIM_OC_Stop_IT(buzzer_watcher, watcher_channel);
    __HAL_TIM_SET_COUNTER(buzzer_watcher, 0);
	HAL_TIM_OC_Start_IT(buzzer_watcher, watcher_channel);
}

