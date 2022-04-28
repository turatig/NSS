/*
 * Step.c
 *
 *  Created on: Mar 28, 2022
 *      Author: jack
 */
#include "Step.h"
#include "utils.h"
#include "stm32f4xx.h"

void initStep(Step *inst,uint16_t p0,uint16_t p1,uint16_t p2,uint16_t p3,GPIO_TypeDef *port){
	inst->pins[0]=p0;
	inst->pins[1]=p1;
	inst->pins[2]=p2;
	inst->pins[3]=p3;

	inst->port=port;
	inst->cur_step=0;
	rstPins(inst);
	rstAngle(inst);
}

void rstPins(Step *inst){
	inst->port->ODR&=~( inst->pins[0] | inst->pins[1] | inst->pins[2] | inst->pins[3] );
}

void rstAngle(Step *inst){
	inst->ang_idx=0;
}

/*if dir activate pins backward[pin 4-0] else forward[pin 0-4] in wave step mode*/
void waveStep(Step *inst,uint8_t dir){

	/*Reset current step pin*/
	inst->port->ODR&= ~(inst->pins[ inst->cur_step ]);

	/*Update current step and angular index according to given direction*/
	if(dir){
		inst->cur_step=(inst->cur_step + 1) & 0x3;
		inst->ang_idx ++;
	}
	else{
		inst->cur_step=(inst->cur_step - 1) & 0x3;
		inst->ang_idx --;
	}

	/*Set updated current step pin*/
	inst->port->ODR|= inst->pins[ inst->cur_step ];
}

