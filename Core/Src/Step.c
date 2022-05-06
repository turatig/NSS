/*
 * Step.c
 *
 *  Created on: Mar 28, 2022
 *      Author: jack
 */
#include "Step.h"
#include "utils.h"
#include "stm32f4xx.h"

/*Init stepper motor data structure*/
void initStep(Step *inst,uint16_t p0,uint16_t p1,uint16_t p2,uint16_t p3,GPIO_TypeDef *port,StepMode mode,TIM_HandleTypeDef *htim){

	/*Assign gpio pins and port*/
	inst->pins[0]=p0;
	inst->pins[1]=p1;
	inst->pins[2]=p2;
	inst->pins[3]=p3;

	inst->port=port;

	initMode(inst,mode);
	rstAngle(inst);

	inst->move_lock=0;
	inst->htim=htim;
}


void initMode(Step *inst,StepMode mode){
	inst->mode=mode;
	inst->cur_step=0;
	rstPins(inst);

	/*BYJ-24 shaft (considering also torque factor) performs a complete rotation in 2038 steps*/
	if(mode==WAVE || mode==FULL)
		inst->res=360.0/2038.0;
}
void rstPins(Step *inst){
	inst->port->ODR&=~( inst->pins[0] | inst->pins[1] | inst->pins[2] | inst->pins[3] );
}

void rstAngle(Step *inst){
	inst->ang_idx=0;
}


/*
	WAVE DRIVE PATTERN
	 p1 p2 p3 p4
    |1  0  0  0|
    |0  1  0  0|
    |0  0  1  0|
    |0  0  0  1|
 */
void waveStep(Step *inst,uint8_t dir){

	/*Reset current step pin*/
	inst->port->ODR&= ~(inst->pins[ inst->cur_step ]);

	/*Update current step and angular index according to given direction*/
	if(dir){
		inst->cur_step=(inst->cur_step + 1) & 0x3;
		inst->ang_idx ++;
	}
	else{
		inst->cur_step= inst->cur_step ? (inst->cur_step - 1) : 0x3;
		inst->ang_idx --;
	}

	/*Set updated current step pin*/
	inst->port->ODR|= inst->pins[ inst->cur_step ];
}

/*
	FULL DRIVE PATTERN
	 p1 p2 p3 p4
    |1  1  0  0|
    |0  1  1  0|
    |0  0  1  1|
    |1  0  0  1|
 */
void fullStep(Step *inst,uint8_t dir){

	/*Update current step and angular index according to given direction*/
	if(dir){
		/*Reset current step pin*/
		inst->port->ODR&= ~(inst->pins[ inst->cur_step ]);

		inst->cur_step=(inst->cur_step + 1) & 0x3;
		inst->ang_idx ++;


	}
	else{
		/*Reset current step pin*/
		inst->port->ODR&= ~(inst->pins[ (inst->cur_step + 1) & 0x3 ]);
		inst->cur_step= inst->cur_step ? (inst->cur_step - 1) : 0x3;
		inst->ang_idx --;
	}

	/*Set updated current step pin*/
	inst->port->ODR|= ( inst->pins[ inst->cur_step ] | inst->pins[ (inst->cur_step + 1) & 0x3] );
}

/*
	HALF DRIVE PATTERN
	 p1 p2 p3 p4
    |1  0  0  0|
    |1  1  0  0|
    |0  1  0  0|
    |0  1  1  0|
    |0  0  1  0|
    |0  0  1  1|
    |0  0  0  1|
    |1  0  0  1|

 */
void halfStep(Step *inst,uint8_t dir){
	__NOP();
}

/*Perform one step according to drive mode*/
void step(Step *inst,uint8_t dir){
	switch(inst->mode){
	case WAVE:
		waveStep(inst,dir);
	case FULL:
		fullStep(inst,dir);
	}
}


/*Move motor from a starting position to a destination expressed in angle (degree)*/
void moveToPoll(Step *inst,float angle){

	if(!inst->move_lock){
		inst->move_lock=1;

		if( angle > inst->ang_idx * inst->res ){
			while( angle > ( inst->ang_idx + 1 ) * inst->res ){
				step(inst,1);
				HAL_Delay(1);
			}
		}
		else{
			while( angle < ( inst->ang_idx - 1 ) * inst->res ){
				step(inst,0);
				HAL_Delay(1);
			}
		}

		inst->move_lock=0;
	}
}

/*
 * Perform one step in interrupt mode
 * this function is meant to be called inside instance timer PeriodElapsed callback
 */
void stepIt(Step *inst){
	if( inst->destination_it > inst->ang_idx * inst->res ){

		if( inst->destination_it > ( inst->ang_idx + 1 ) * inst->res )
			step(inst,1);
		else{
			HAL_TIM_Base_Stop_IT(inst->htim);
			inst->move_lock=0;
		}
	}

	else{
		if( inst->destination_it < ( inst->ang_idx - 1 ) * inst->res )
			step(inst,0);
		else{
			HAL_TIM_Base_Stop_IT(inst->htim);
			inst->move_lock=0;
		}
	}
}


/*Move motor from a starting position to a destination expressed in angle (degree)*/
void moveToIt(Step *inst,float angle){

	if(!inst->move_lock){
		inst->move_lock=1;

		inst->destination_it=angle;
		HAL_TIM_Base_Start_IT( inst->htim );
	}
}



