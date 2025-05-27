/*
 * Pulse_pin.c
 *
 *  Created on: May 15, 2025
 *      Author: victo
 */
#include "GAUL_drivers/Pulse_pin.h"


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
		HAL_TIM_OC_Stop_IT(htim, TIM_CHANNEL_1);
	}
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
		HAL_TIM_OC_Stop_IT(htim, TIM_CHANNEL_1);
	}
	if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
		HAL_TIM_OC_Stop_IT(htim, TIM_CHANNEL_1);
	}
}


void PulsePin_init(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4, uint16_t time)
{
	  __HAL_TIM_SET_COMPARE(htim2, TIM_CHANNEL_1, time);
	  __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_1, time);
	  __HAL_TIM_SET_COMPARE(htim4, TIM_CHANNEL_1, time);
	  HAL_TIM_OC_Stop_IT(htim2, TIM_CHANNEL_1);
	  HAL_TIM_OC_Stop_IT(htim3, TIM_CHANNEL_1);
	  HAL_TIM_OC_Stop_IT(htim4, TIM_CHANNEL_1);
}

void PulsePin(uint8_t pin, TIM_HandleTypeDef *htim)
{
	if (pin == 0)
	{
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
		  __HAL_TIM_SET_COUNTER(htim, 0);
		  HAL_TIM_OC_Start_IT(htim, TIM_CHANNEL_1);
	}
	if (pin == 1)
	{
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
		  __HAL_TIM_SET_COUNTER(htim, 0);
		  HAL_TIM_OC_Start_IT(htim, TIM_CHANNEL_1);
	}
	if (pin == 2)
	{
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
		  __HAL_TIM_SET_COUNTER(htim, 0);
		  HAL_TIM_OC_Start_IT(htim, TIM_CHANNEL_1);
	}
}
