/*
 * Pulse_pin.h
 *
 *  Created on: May 15, 2025
 *      Author: victo
 */

#ifndef INC_GAUL_DRIVERS_PULSE_PIN_H_
#define INC_GAUL_DRIVERS_PULSE_PIN_H_

#include "stm32f1xx_hal.h"

void PulsePin_init(uint8_t pin_id);
void pulse_pin(uint8_t pin_id, uint16_t time);


#endif /* INC_GAUL_DRIVERS_PULSE_PIN_H_ */
