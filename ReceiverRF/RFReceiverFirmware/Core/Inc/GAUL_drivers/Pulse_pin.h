/*
 * Pulse_pin.h
 *
 *  Created on: May 15, 2025
 *      Author: victo
 */

#ifndef INC_GAUL_DRIVERS_PULSE_PIN_H_
#define INC_GAUL_DRIVERS_PULSE_PIN_H_

#include "stm32f1xx_hal.h"

void PulsePin_init(TIM_HandleTypeDef *buz_htim, uint32_t buz_channel, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4, uint16_t time);
void PulsePin(uint8_t pin, TIM_HandleTypeDef *htim);


#endif /* INC_GAUL_DRIVERS_PULSE_PIN_H_ */
