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
	WAVE=0x0,
	FULL=0x1,
	HALF=0x2
}StepMode;

/* Step motor driver library*/
typedef struct{
	/*Driver pin masks*/
	uint16_t pins[4];
	/*Address of gpio port*/
	GPIO_TypeDef *port;

	/*stepper motor drive mode*/
	StepMode mode;

	/*current step idx in the driving pattern*/
	uint8_t cur_step;
	/*Current angle index: will be incremented/decremented by step comands*/
	int ang_idx;
	/*Angle resolution
	 * - WAVE/FULL drive: 360/2038
	 */
	float res;

	/*Timer to move in interrupt mode*/
	TIM_HandleTypeDef *htim;
	/*Boolean flag to lock the stepper motor while it's moving to a destination in interrupt/polling mode*/
	uint8_t move_lock;
	/*Destination angle: set by the API when moveToIt*/
	float destination_it;
}Step;

/*Init motor data structure from pin numbers and gpio port specification*/
void initStep(Step *inst,uint16_t p0,uint16_t p1,uint16_t p2,uint16_t p3,GPIO_TypeDef *port,StepMode mode,TIM_HandleTypeDef *htim);
/*Init pattern data structures according to given driver mode*/

void initMode(Step *inst,StepMode mode);
/*Reset gpio pins to 0*/
void rstPins(Step *inst);
/*Reset angle index to 0*/
void rstAngle(Step *inst);
/*External stepper motor interface to step*/
void step(Step *inst,uint8_t dir);
void stepIt(Step *inst);
void moveToPoll(Step *inst,float angle);
void moveToIt(Step *inst,float angle);

#endif /* INC_STEP_H_ */
