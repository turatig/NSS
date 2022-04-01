/*
 * utils.c
 *
 *  Created on: Mar 29, 2022
 *      Author: jack
 */


#include "stm32f4xx.h"
#include "utils.h"

#pragma GCC push_options
#pragma GCC optimize ("O3")

void delayUS(uint32_t us) {
	DWT->CTRL|=1;
	DWT->CYCCNT=0;

	volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while(DWT->CYCCNT - start < cycles);
}

#pragma GCC pop_options
