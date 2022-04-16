/*
 * Jstick.c
 *
 *  Created on: Mar 29, 2022
 *      Author: jack
 */

#include "Jstick.h"
#include "stm32f4xx.h"


void jstickInit(Jstick *inst,ADC_HandleTypeDef *hadc,uint16_t led_pin,GPIO_TypeDef *gp){
	inst->hadc=hadc;
	inst->led_pin=led_pin;
	inst->gp=gp;

}


JstickDir jstickGetDirection(Jstick *inst){
	uint16_t raw_val;
	HAL_StatusTypeDef status;

	status=HAL_ADC_PollForConversion(inst->hadc,50);

	if(status==HAL_OK){
		inst->gp->ODR&=~inst->led_pin;
		raw_val=HAL_ADC_GetValue(inst->hadc);

		if(raw_val>4000) return LEFT;
		if(raw_val<100) return RIGHT;
	}
	/*Notify error by flash error led*/
	else{
		inst->gp->ODR|=inst->led_pin;
	}

	return CENTER;
}

