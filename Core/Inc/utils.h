/*
 * utils.h
 *
 *  Created on: Mar 29, 2022
 *      Author: jack
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

/*Flash error led debug macro*/
#define _FL_DEBUG(status,port,pin) if(status!=HAL_OK){\
		port->ODR|=pin;\
	}\
	else{\
		port->ODR&=~pin;\
	}
#define _FL_EQ_DEBUG(status,value,port,pin) if(status==value){\
		port->ODR|=pin;\
	}

#include "stm32f4xx.h"

//Delay microseconds implementation
void delayUS(uint32_t us);

#endif /* INC_UTILS_H_ */
