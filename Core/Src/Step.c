/*
 * Step.c
 *
 *  Created on: Mar 28, 2022
 *      Author: jack
 */
#include "Step.h"
#include "stm32f4xx.h"

void stepInit(Step *inst,uint16_t p1,uint16_t p2,uint16_t p3,uint16_t p4,GPIO_TypeDef *gp){
	inst->pins[0]=p1;
	inst->pins[1]=p2;
	inst->pins[2]=p3;
	inst->pins[3]=p4;

	inst->gpio_port=gp;
	inst->gpio_port->ODR&=~( inst->pins[0] | inst->pins[1] | inst->pins[2] | inst->pins[4] );
}

/*if dir activate pins backward[pin 4-0] else forward[pin 0-4] in wave step mode*/
void stepWave(Step *inst,uint8_t dir){

	for(int i=0;i<4;i++){
		inst->gpio_port->ODR&=~inst->pins[ dir ? (4-i) & 3 : i ];
		inst->gpio_port->ODR|=inst->pins[ dir ? 3-i : (i+1) & 3 ];

		HAL_Delay(1);
	}
}
