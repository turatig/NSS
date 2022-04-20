/*
 * Step.c
 *
 *  Created on: Mar 28, 2022
 *      Author: jack
 */
#include "Step.h"
#include "utils.h"
#include "stm32f4xx.h"

void stepInit(Step *inst,uint16_t p0,uint16_t p1,uint16_t p2,uint16_t p3,GPIO_TypeDef *port){
	inst->pins[0]=p0;
	inst->pins[1]=p1;
	inst->pins[2]=p2;
	inst->pins[3]=p3;

	inst->port=port;
	inst->port->ODR&=~( inst->pins[0] | inst->pins[1] | inst->pins[2] | inst->pins[3] );

}

/*if dir activate pins backward[pin 4-0] else forward[pin 0-4] in wave step mode*/
void stepWave(Step *inst,uint8_t dir){

	inst->port->ODR&=~( inst->pins[0] | inst->pins[1] | inst->pins[2] | inst->pins[3] );
	inst->port->ODR|=inst->pins[0];

	for(int i=0;i<4;i++){
		inst->port->ODR&=~inst->pins[ dir ? (4-i) & 3 : i ];
		inst->port->ODR|=inst->pins[ dir ? 3-i : (i+1) & 3 ];

		//delayUS(500);
		HAL_Delay(2);
	}
}
