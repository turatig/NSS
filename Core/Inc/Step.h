/*
 * Step.h
 *
 *  Created on: Mar 28, 2022
 *      Author: jack
 */

#ifndef INC_STEP_H_
#define INC_STEP_H_

#include "stm32f4xx.h"

/*Angular resolution of BYJ28 motor*/
#define ANG_RES (float)11.25

typedef enum{
	WAVE=0x0
}StepMode;

/* Step motor driver library*/
typedef struct{
	/*Driver pin masks*/
	uint16_t pins[4];
	/*Address of gpio port*/
	GPIO_TypeDef *port;
	/*current step idx in the driving pattern*/
	uint8_t cur_step;
	/*Angle index in resolution step: will be incremented/decremented by step comands*/
	uint8_t ang_idx;
}Step;

/*Init motor data structure from pin numbers and gpio port specification*/
void initStep(Step *inst,uint16_t p0,uint16_t p1,uint16_t p2,uint16_t p3,GPIO_TypeDef *port);

/*Reset gpio pins to 0*/
void rstPins(Step *inst);
/*Reset angle index to 0*/
void rstAngle(Step *inst);
/*Perform a step in wave_step mode -- dir: boolean directions (forward/backward)*/
void waveStep(Step *inst,uint8_t dir);
void fullStep(Step *inst,uint8_t dir);

#endif /* INC_STEP_H_ */
