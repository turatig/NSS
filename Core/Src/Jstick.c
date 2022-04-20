/*
 * Jstick.c
 *
 *  Created on: Mar 29, 2022
 *      Author: jack
 */

#include "Jstick.h"
#include "stm32f4xx.h"
#include "utils.h"


void jstickInit(Jstick *inst,ADC_HandleTypeDef *hadc,uint16_t errpin,GPIO_TypeDef *errport){
	inst->hadc=hadc;
	inst->errpin=errpin;
	inst->errport=errport;

}


JstickDir jstickGetDirection(Jstick *inst){
	uint16_t raw_val;
	HAL_StatusTypeDef status;

	status=HAL_ADC_PollForConversion(inst->hadc,50);

	_FL_DEBUG(status,inst->errport,inst->errpin);
	if(status==HAL_OK){
		raw_val=HAL_ADC_GetValue(inst->hadc);

		if(raw_val>4000) return LEFT;
		if(raw_val<100) return RIGHT;
	}

	return CENTER;
}

