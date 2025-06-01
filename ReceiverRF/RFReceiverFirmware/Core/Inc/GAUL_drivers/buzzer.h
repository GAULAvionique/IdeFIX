/*
 * buzzer.h
 *
 *  Created on: Jun 1, 2025
 *      Author: victo
 */

#ifndef INC_GAUL_DRIVERS_BUZZER_H_
#define INC_GAUL_DRIVERS_BUZZER_H_

#include "stm32f1xx_hal.h"



void buzzer_init(TIM_HandleTypeDef *htim, uint32_t channel);
void buzzer_start(uint32_t frequency, uint32_t duration_ms);
void buzzer_stop();


#endif /* INC_GAUL_DRIVERS_BUZZER_H_ */
