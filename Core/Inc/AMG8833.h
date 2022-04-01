/*
 * AMG8833.h
 *
 *  Created on: Feb 22, 2022
 *      Author: jack
 * This is the AMG8833 (8x8 pixel,3V) infrared array sensor driver module.
 * The API provides functions to read temperature measures in
 * 	- polling
 * 	- interrupt
 * 	- dma
 * 	mode and manage its power configurations
 */

#ifndef INC_AMG8833_H_
#define INC_AMG8833_H_

#include "stm32f4xx.h"

#define AMG8833_I2C_BASE_ADR 0x68

/*
 * AMG8833 registers
 */
#define AMG8833_PWR_CTRL 0x00
#define AMG8833_RST 0x01
/*Fps can be set to 1 or 10*/
#define AMG883_FPS 0x02
/*Temperature registers 01/64 low/high - first/last pixel low/high*/
#define AMG8833_T01L 0x80
#define AMG8833_T64H 0xFF
/*Matrix data size in byte*/
#define AMG8833_DS 128

typedef struct{
	/*I2C address of the sensor*/
	uint16_t adri2c;
	/*I2C handle data structure that keeps I2C config*/
	I2C_HandleTypeDef *hi2c;
}AMG8833;

/*
 * -uint8_t ad_sel: address select pin connected to ground/3.3V (false/true).
 */
void amg8833Init(AMG8833 *inst, I2C_HandleTypeDef *hi2c,uint8_t ad_sel);
/*Test if there's a ready device at I2C slave address inst->adri2c*/
uint8_t amg8833IsReady(AMG8833 *inst);
/*
 * Read 128 bytes from register T01L to T64H.
 */
HAL_StatusTypeDef amg8833Reset(AMG8833 *inst);
HAL_StatusTypeDef amg8833ReadPoll(AMG8833 *inst, uint8_t *data);
/* Read one frame in DMA mode*/
HAL_StatusTypeDef amg8833ReadDMA(AMG8833 *inst,uint8_t *data);

#endif /* INC_AMG8833_H_ */
