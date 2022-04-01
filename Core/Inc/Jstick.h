/*
 * Jstick.h
 *
 *  Created on: Mar 28, 2022
 *      Author: jack
 */

#ifndef INC_JSTICK_H_
#define INC_JSTICK_H_

#include "stm32f4xx.h"

/*Mono directional joystick data structure*/
typedef struct{
	ADC_HandleTypeDef *hadc;
}Jstick;

typedef enum{
	CENTER=0x0,
	LEFT=0x1,
	RIGHT=0x2
}JstickDir;

/*hadc in single conversion - single channel mode*/
void jstickInit(Jstick *inst,ADC_HandleTypeDef *hadc);
JstickDir jstickGetDirection(Jstick *inst);

#endif /* INC_JSTICK_H_ */