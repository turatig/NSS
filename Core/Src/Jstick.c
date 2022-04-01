/*
 * Jstick.c
 *
 *  Created on: Mar 29, 2022
 *      Author: jack
 */

#include "Jstick.h"
#include "stm32f4xx.h"

void jstickInit(Jstick *inst,ADC_HandleTypeDef *hadc){
	inst->hadc=hadc;
}

JstickDir jstickGetDirection(Jstick *inst){
	uint16_t raw_val;
	raw_val=HAL_ADC_GetValue(inst->hadc);

	if(raw_val>4000) return LEFT;
	if(raw_val<100) return RIGHT;

	return CENTER;
}

