/*
 * Step.h
 *
 *  Created on: Mar 28, 2022
 *      Author: jack
 */

#ifndef INC_STEP_H_
#define INC_STEP_H_

#include "stm32f4xx.h"

/* Step motor driver library*/
typedef struct{
	/*Driver pin masks*/
	uint16_t pins[4];
	/*Address of gpio port*/
	GPIO_TypeDef *port;
	/*Angle: measured in step resolution unit with respect to an initial value*/
}Step;

/*Init motor data structure from pin numbers and gpio port specification*/
void stepInit(Step *inst,uint16_t p0,uint16_t p1,uint16_t p2,uint16_t p3,GPIO_TypeDef *port);

/*Perform a step in wave_step mode -- dir: boolean directions (forward/backward)*/
void stepWave(Step *inst,uint8_t dir);

#endif /* INC_STEP_H_ */
