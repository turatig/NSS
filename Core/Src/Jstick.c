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

uint8_t jstickIsLeft(Jstick *inst){
	HAL_StatusTypeDef status;
	uint16_t raw_val;
	uint8_t res=0;

	HAL_ADC_Start(inst->hadc);
	status=HAL_ADC_PollForConversion(inst->hadc,2000);

	if(status==HAL_OK){
		raw_val=HAL_ADC_GetValue(inst->hadc);
		if(raw_val>4000) res=1;
	}

	return res;
}

uint8_t jstickIsRight(Jstick *inst){
	HAL_StatusTypeDef status;
	uint16_t raw_val;
	uint8_t res=0;

	HAL_ADC_Start(inst->hadc);
	status=HAL_ADC_PollForConversion(inst->hadc,2000);

	if(status==HAL_OK){
		raw_val=HAL_ADC_GetValue(inst->hadc);
		if(raw_val<100) res=1;
	}

	return res;

}
