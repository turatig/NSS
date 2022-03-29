/*
 * AMG8833.c
 *
 *  Created on: Feb 23, 2022
 *      Author: jack
 */


#include "AMG8833.h"
#include "stm32f4xx.h"

void amg8833Init(AMG8833 *inst,I2C_HandleTypeDef *hi2c,uint8_t ad_sel){
	inst->adri2c= ad_sel ? ( AMG8833_I2C_BASE_ADR + 1 ) << 1 :
								AMG8833_I2C_BASE_ADR << 1;
	inst->hi2c=hi2c;
}

uint8_t amg8833IsReady(AMG8833 *inst){
	HAL_StatusTypeDef status=HAL_I2C_IsDeviceReady(inst->hi2c,inst->adri2c,5,10);
	return (uint8_t)(status==HAL_OK);
}

uint8_t amg8833IsDMAEnabled(AMG8833 *inst){
	return (uint8_t)(inst->hi2c->hdmarx!=NULL);
}

HAL_StatusTypeDef amg8833ReadPoll(AMG8833 *inst,uint8_t *data){
	HAL_StatusTypeDef status;
	status=HAL_I2C_Mem_Read(inst->hi2c,(uint16_t)inst->adri2c,AMG8833_T01L,
			1,data,128,HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef amg8833ReadDMA(AMG8833 *inst,uint8_t *data){
	HAL_StatusTypeDef status;

	status=HAL_I2C_Mem_Read_DMA(inst->hi2c,(uint16_t)inst->adri2c,AMG8833_T01L,
			1,data,128);

	return status;
}
