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

/*Chip reset register*/
#define AMG8833_RST 0x01


/*Reset mode values*/
#define AMG8833_INIT_RST 0x3F
#define AMG8833_FLAG_RST 0x30

/*Fps register*/
#define AMG883_FPS 0x02

/*Interrupt control register*/
#define AMG8833_INTC 0x03

/*Interrupt mode values*/
#define AMG8833_INTEN 0X01
#define AMG8833_INT_MOD_FLAG ( 0x01 << 1 )

/*
 * Status register
 */
#define AMG8833_STAT 0X04
#define AMG8833_INT_FLAG ( 0x01 << 1 )
/*
 * Status clear registers
 */
#define AMG8833_SCLR 0X05
#define AMG8833_INT_CLR (0X01 << 1)

/*
 * Interrupt table registers
 */
#define AMG8833_INT_TAB 0X10
#define AMG8833_INT_TAB_SZ 8

/*Threshold registers*/
#define AMG8833_INTHL 0x08
#define AMG8833_INTHH 0x09

#define AMG8833_INTLL 0X0A
#define AMG8833_INTLH 0X0B

#define AMG8833_IHYSL 0x0C
#define AMG8833_IHYSH 0X0D

/*Temperature registers 01/64 low/high - first/last pixel low/high*/
#define AMG8833_T01L 0x80
#define AMG8833_T64H 0xFF

/*Matrix data size in byte*/
#define AMG8833_DS 128
/*Interrupt matrix data size*/
#define AMG8833_INT_TAB_SZ 8

typedef struct{
	/*I2C address of the sensor*/
	uint16_t adri2c;
	/*I2C handle data structure that keeps I2C config*/
	I2C_HandleTypeDef *hi2c;
}AMG8833;

/*
 * -uint8_t ad_sel: address select pin connected to ground/3.3V (false/true).
 */
void amg8833Init (AMG8833 *inst, I2C_HandleTypeDef *hi2c,uint8_t ad_sel);
/*Test if there's a ready device at I2C slave address inst->adri2c*/
uint8_t amg8833IsReady( AMG8833 *inst, uint8_t max_retry );

/*Reset amg8833 software by writing in register 0x1*/
HAL_StatusTypeDef amg8833Reset( AMG8833 *inst, uint8_t max_retry );
/*Reset interrupt flag and interrupt table*/
HAL_StatusTypeDef amg8833FlagReset( AMG8833 *inst, uint8_t max_retry );

/*
 * Read 128 bytes from register T01L to T64H.
 */
HAL_StatusTypeDef amg8833ReadPoll( AMG8833 *inst, uint8_t *data, uint8_t max_retry );
/*
 * Read interrupt table into data buffer
 */
HAL_StatusTypeDef amg8833GetIntTab( AMG8833 *inst, uint8_t *data, uint8_t max_retry );
/* Read one frame in DMA mode*/
HAL_StatusTypeDef amg8833ReadDMA( AMG8833 *inst,uint8_t *data, uint8_t max_retry );

/*Enable interrupt*/
HAL_StatusTypeDef amg8833IntEn( AMG8833 *inst, uint8_t max_retry );
/*Disable interrupt*/
HAL_StatusTypeDef amg8833IntDis( AMG8833 *inst, uint8_t max_retry );

/*Read control register value*/
HAL_StatusTypeDef amg8833GetIntCtrl( AMG8833 *inst,uint8_t *data, uint8_t max_retry );
/*Get interrupt internal flag*/
uint8_t amg8833GetIntFlag( AMG8833 *inst, uint8_t max_retry );
/*Clear interrupt flag*/
HAL_StatusTypeDef amg8833IntClr( AMG8833 *inst,uint8_t max_retry);


/*Set interrupt generation with hysteresis/absolute value (0/1)*/
HAL_StatusTypeDef amg8833SetIntMode( AMG8833 *inst, uint8_t mode,uint8_t max_retry );

/*Set absolute value threshold for interrupt*/
HAL_StatusTypeDef amg8833SetAbsHighThrs(AMG8833 *inst,uint16_t thrs, uint8_t max_retry );
uint16_t amg8833GetAbsHighThrs(AMG8833 *inst, uint8_t max_retry);

HAL_StatusTypeDef amg8833SetAbsLowThrs(AMG8833 *inst,uint16_t thrs, uint8_t max_retry );
uint16_t amg8833GetAbsLowThrs(AMG8833 *inst, uint8_t max_retry);

/*Set hysteresis value threshold for interrupt*/
HAL_StatusTypeDef amg8833SetHysThrs(AMG8833 *inst,uint16_t thrs, uint8_t max_retry );
uint16_t amg8833GetHysThrs(AMG8833 *inst, uint8_t max_retry);

#endif /* INC_AMG8833_H_ */
