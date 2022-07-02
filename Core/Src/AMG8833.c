/*
 * AMG8833.c
 *
 *  Created on: Feb 23, 2022
 *      Author: jack
 */


#include "AMG8833.h"
#include "stm32f4xx.h"

void amg8833Init(AMG8833 *inst,I2C_HandleTypeDef *hi2c, uint8_t ad_sel){

	inst->adri2c= ad_sel ? ( AMG8833_I2C_BASE_ADR + 1 ) << 1 :
								AMG8833_I2C_BASE_ADR << 1;
	inst->hi2c=hi2c;
}

/*
 * Test if the sensor reply on configured I2C line
 */
uint8_t amg8833IsReady(AMG8833 *inst,uint8_t max_retry){

	HAL_StatusTypeDef status=HAL_I2C_IsDeviceReady(inst->hi2c,inst->adri2c, max_retry ,10);
	return (uint8_t)(status==HAL_OK);
}

uint8_t amg8833IsDMAEnabled(AMG8833 *inst){
	return (uint8_t)(inst->hi2c->hdmarx!=NULL);
}

/*
 * Utils to read/write on I2C line with automatic retry in case of failure
 */
HAL_StatusTypeDef I2C_Read(AMG8833 *inst, uint8_t reg,uint8_t *data,uint16_t dim,uint16_t max_retry){

	uint8_t try=0;
	HAL_StatusTypeDef status;

	do{
		try++;
		status=HAL_I2C_Mem_Read( inst->hi2c,(uint16_t)inst->adri2c,reg,
				1,data,dim,2000 );
	}while( status!=HAL_OK && try <= max_retry );

	return status;

}

HAL_StatusTypeDef I2C_Write(AMG8833 *inst, uint8_t reg,uint8_t *data,uint16_t dim,uint16_t max_retry){

	uint8_t try=0;
	HAL_StatusTypeDef status;

	do{
		try++;
		status=HAL_I2C_Mem_Write( inst->hi2c,(uint16_t)inst->adri2c,reg,
				1,data,dim,2000 );
	}while( status!=HAL_OK && try <= max_retry );

	return status;

}

HAL_StatusTypeDef I2C_ReadDMA(AMG8833 *inst, uint8_t reg,uint8_t *data,uint16_t dim,uint16_t max_retry){

	uint8_t try=0;
	HAL_StatusTypeDef status;

	do{
		try++;
		status=HAL_I2C_Mem_Read_DMA( inst->hi2c,(uint16_t)inst->adri2c,reg,
				1,data,dim );
	}while( status!=HAL_OK && try <= max_retry );

	return status;

}
/*
 * Reset to initial settings
 */
HAL_StatusTypeDef amg8833Reset( AMG8833 *inst, uint8_t max_retry ){

	uint8_t data=AMG8833_INIT_RST;
	return I2C_Write( inst,AMG8833_RST,&data,1,max_retry );


}

/*
 * Reset interrupt flags and interrupt table
 */
HAL_StatusTypeDef amg8833FlagReset(AMG8833 *inst, uint8_t max_retry ){

	uint8_t data=AMG8833_FLAG_RST;
	return I2C_Write( inst,AMG8833_RST,&data,1,max_retry );


}

/*
 * Read temperature matrix in polling mode
 */
HAL_StatusTypeDef amg8833ReadPoll(AMG8833 *inst,uint8_t *data, uint8_t max_retry){

	return I2C_Read( inst, AMG8833_T01L,data,AMG8833_DS,max_retry );
}

/*
 * Read temperature matrix in DMA mode
 */
HAL_StatusTypeDef amg8833ReadDMA(AMG8833 *inst,uint8_t *data, uint8_t max_retry){

	return I2C_ReadDMA( inst,AMG8833_T01L,data,AMG8833_DS,max_retry );
}
/*
 * Read interrupt table into data buffer
 */
HAL_StatusTypeDef amg8833GetIntTab( AMG8833 *inst, uint8_t *data, uint8_t max_retry ){
	return I2C_Read( inst, AMG8833_INT_TAB, data, AMG8833_INT_TAB_SZ, max_retry );
}

/*
 * Enable interrupt triggered by by pixels above hysteresis/absolute  threshold
 */
HAL_StatusTypeDef amg8833IntEn(AMG8833 *inst, uint8_t max_retry ){

	HAL_StatusTypeDef status;
	uint8_t data;

	status=amg8833GetIntCtrl( inst,&data, max_retry );

	if( status!=HAL_OK )
		return status;

	data|=AMG8833_INTEN;

	return I2C_Write( inst,AMG8833_INTC,&data,1,max_retry );

}

/*
 * Disable interrupt triggered by by pixels above hysteresis/absolute  threshold
 */
HAL_StatusTypeDef amg8833IntDis(AMG8833 *inst, uint8_t max_retry ){

	HAL_StatusTypeDef status;
	uint8_t data;

	status=amg8833GetIntCtrl( inst,&data, max_retry );

	if( status!=HAL_OK )
		return status;

	data&=~AMG8833_INTEN;

	return I2C_Write( inst,AMG8833_INTC,&data,1,max_retry );

}


/*
 * Get interrupt control register value
 */
HAL_StatusTypeDef amg8833GetIntCtrl(AMG8833 *inst,uint8_t *data, uint8_t max_retry){

	return I2C_Read( inst,AMG8833_INTC,data,1,max_retry );
}

/*
 * Set interrupt in hysteresis/absolute ( 0/1 ) threshold mode
 */
HAL_StatusTypeDef amg8833SetIntMode(AMG8833 *inst, uint8_t mode, uint8_t max_retry){

	HAL_StatusTypeDef status;
	uint8_t data;

	status=amg8833GetIntCtrl( inst,&data, max_retry );

	if( status!=HAL_OK )
		return status;

	/*
	 * Toggle bit of absolute threshold interrupt
	 */
	if(!mode)
		/*
		 * Set hysteresis mode
		 */
		data&=~AMG8833_INT_MOD_FLAG;
	else
		/*
		 * Set absolute mode
		 */
		data|=AMG8833_INT_MOD_FLAG;

	return I2C_Write( inst,AMG8833_INTC,&data,1,max_retry );
}

/*
 * Clear interrupt status flag
 */
HAL_StatusTypeDef amg8833IntClr( AMG8833 *inst, uint8_t max_retry){

	uint8_t data=AMG8833_INT_CLR;
	return I2C_Write( inst, AMG8833_SCLR,&data,1, max_retry);
}

/*
 * This function return 1/0 for interrupt set/reset state or 0xff in case of error
 */
uint8_t amg8833GetIntFlag( AMG8833 *inst, uint8_t max_retry ){

	HAL_StatusTypeDef status;
	uint8_t data;

	status=I2C_Read( inst,AMG8833_STAT,&data,1,max_retry);
	if( status!= HAL_OK ){
		/*
		 *
		 */
		data=0xFF;
		return data;
	}

	/*
	 * Mask interrupt flag
	 */
	data&=AMG8833_INT_FLAG;
	/*
	 * Shift right by 1 bit - will return 0/1 for interrupt reset/set
	 */
	if( data!=0 )
		return 1;
	else
		return 0;
}

/*
 * Set 12 bit absolute threshold value if interrupt in absolute mode is selected
 */
HAL_StatusTypeDef amg8833SetAbsHighThrs(AMG8833 *inst,uint16_t thrs, uint8_t max_retry){

	HAL_StatusTypeDef status;
	uint8_t data;

	/*Lower byte*/
	data=(uint8_t)( thrs & 0x00FF );

	status=I2C_Write( inst,AMG8833_INTHL,&data,1,max_retry);

	if(status!=HAL_OK){
		return status;
	}

	/*Higher byte (only lower 4-bit)*/
	data=(uint8_t)( ( thrs & 0x0F00 ) >> 8 );
	return I2C_Write( inst,AMG8833_INTHH,&data,1,max_retry);
}


/*
 * Read absolute threshold value from AMG8833_INTHL and AMG8833_INTHH on uint16_t
 */
uint16_t amg8833GetAbsHighThrs(AMG8833 *inst, uint8_t max_retry){

	HAL_StatusTypeDef status;
	uint16_t res=0;
	uint8_t data[2];

	status=I2C_Read( inst,AMG8833_INTHL,data,2,max_retry);

	if(status!=HAL_OK){
		return 0xFFFF;
	}

	res=res | data[0];
	/*
	 * Keep only lower 4 bits of the higher byte (12 bit number)
	 */
	res=res | ( ( data[1] & 0x0F) << 8);
	return res;
}

/*
 * Set 12 bit absolute threshold value if interrupt in absolute mode is selected
 */
HAL_StatusTypeDef amg8833SetAbsLowThrs(AMG8833 *inst,uint16_t thrs, uint8_t max_retry){

	HAL_StatusTypeDef status;
	uint8_t data;

	/*Lower byte*/
	data=(uint8_t)( thrs & 0x00FF );

	status=I2C_Write( inst,AMG8833_INTLL,&data,1,max_retry);

	if(status!=HAL_OK){
		return status;
	}

	/*Higher byte (only lower 4-bit)*/
	data=(uint8_t)( ( thrs & 0x0F00 ) >> 8 );
	return I2C_Write( inst,AMG8833_INTLH,&data,1,max_retry);
}

/*
 * Read absolute threshold value from AMG8833_INTHL and AMG8833_INTHH on uint16_t
 */
uint16_t amg8833GetAbsLowThrs(AMG8833 *inst, uint8_t max_retry){

	HAL_StatusTypeDef status;
	uint16_t res=0;
	uint8_t data[2];

	status=I2C_Read( inst,AMG8833_INTLL,data,2,max_retry);

	if(status!=HAL_OK){
		return 0xFFFF;
	}

	res=res | data[0];
	/*
	 * Keep only lower 4 bits of the higher byte (12 bit number)
	 */
	res=res | ( ( data[1] & 0x0F) << 8);
	return res;
}

/*
 * Set 12 bit absolute threshold value if interrupt in absolute mode is selected
 */
HAL_StatusTypeDef amg8833SetHysThrs(AMG8833 *inst,uint16_t thrs, uint8_t max_retry){

	HAL_StatusTypeDef status;
	uint8_t data;

	/*Lower byte*/
	data=(uint8_t)( thrs & 0x00FF );

	status=I2C_Write( inst,AMG8833_IHYSL,&data,1,max_retry);

	if(status!=HAL_OK){
		return status;
	}

	/*Higher byte (only lower 4-bit)*/
	data=(uint8_t)( ( thrs & 0x0F00 ) >> 8 );
	return I2C_Write( inst,AMG8833_IHYSH,&data,1,max_retry);
}

/*
 * Read absolute threshold value from AMG8833_INTHL and AMG8833_INTHH on uint16_t
 */
uint16_t amg8833GetHysThrs(AMG8833 *inst, uint8_t max_retry){

	HAL_StatusTypeDef status;
	uint16_t res=0;
	uint8_t data[2];

	status=I2C_Read( inst,AMG8833_IHYSL,data,2,max_retry);

	if(status!=HAL_OK){
		return 0xFFFF;
	}

	res=res | data[0];
	/*
	 * Keep only lower 4 bits of the higher byte (12 bit number)
	 */
	res=res | ( ( data[1] & 0x0F) << 8);
	return res;
}








