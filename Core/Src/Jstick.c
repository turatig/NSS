/*
 * Jstick.c
 *
 *  Created on: Mar 29, 2022
 *      Author: jack
 */

#include "Jstick.h"
#include "stm32f4xx.h"


void initJstick(Jstick *inst,ADC_HandleTypeDef *hadc,uint16_t errpin,GPIO_TypeDef *errport){
	inst->hadc=hadc;
	inst->errpin=errpin;
	inst->errport=errport;

}


JstickDir jstickGetDirPoll(Jstick *inst){
	uint16_t raw_val;
	HAL_StatusTypeDef status;

	status=HAL_ADC_PollForConversion(inst->hadc,50);

	if(status==HAL_OK){
		raw_val=HAL_ADC_GetValue(inst->hadc);

		if(raw_val>4000) return RIGHT;
		if(raw_val<100) return LEFT;
	}

	return CENTER;
}

