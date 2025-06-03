/*
 * buzzer.h
 *
 *  Created on: Jun 1, 2025
 *      Author: victo
 */

#ifndef INC_GAUL_DRIVERS_BUZZER_H_
#define INC_GAUL_DRIVERS_BUZZER_H_

#include "stm32f1xx_hal.h"



void buzzer_init(TIM_HandleTypeDef *htim_buz, uint32_t buz_channel, TIM_HandleTypeDef *htim_watch, uint32_t watch_channel);
void buzzer_beep(uint32_t frequency, uint32_t duration_ms);


#endif /* INC_GAUL_DRIVERS_BUZZER_H_ */
