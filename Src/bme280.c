/*
 * bme280.c
 *
 *  Created on: Jan 7, 2020
 *      Author: DC
 */

#include <stdint.h>
#include "i2c.h"

#define DEVICE_ADDRESS 0x76; // use 0x76 even though data-sheet says 0x77


uint8_t readBme280reg(uint8_t reg)
{
	uint16_t shiftedAddress = DEVICE_ADDRESS;
	shiftedAddress = shiftedAddress << 1;
	uint8_t pData[100] = {0};
	pData[0] = reg; //register in question
	uint16_t Size = 1;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, shiftedAddress, pData, Size, 1000); //select register
	if(status != HAL_OK)
	{
		// TODO: log error
	}

	uint8_t value = 0;
	status = HAL_I2C_Master_Receive(&hi2c1, shiftedAddress, &value, 1, 1000); //read from register
	return value;
}
