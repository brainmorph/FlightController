/*
 * bme280.c
 *
 *  Created on: Jan 7, 2020
 *      Author: DC
 */

#include <stdint.h>
#include "i2c.h"

#define DEVICE_ADDRESS 0x76; // use 0x76 even though data-sheet says 0x77


uint8_t bme280ReadReg(uint8_t reg)
{
	uint16_t deviceAddress = DEVICE_ADDRESS;
	uint16_t shiftedAddress = deviceAddress << 1;
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

void bme280WriteReg(uint8_t reg, uint8_t value)
{
	uint16_t deviceAddress = DEVICE_ADDRESS;
	uint16_t shiftedAddress = deviceAddress << 1;
	uint8_t pData[100];
	pData[0] = reg; //register in question
	pData[1] = value; //value to write
	uint16_t Size = 2; //we need to send 2 bytes of data (check out mpu datasheet... write register operation is defined this way)
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, shiftedAddress, pData, Size, 1000); //select register and write to it all in one
	if(status != HAL_OK)
	{
		// TODO: log error
	}
}

uint16_t bme280ReadPressure()
{
	uint16_t shiftedAddress = DEVICE_ADDRESS;
	shiftedAddress = shiftedAddress << 1;
	uint8_t pData[10] = {0};
	pData[0] = 0xF7; // Register of MSB pressure value.  Plan is to read 2 more registers after this one for LSB and XSB
	uint16_t Size = 1;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, shiftedAddress, pData, Size, 1000); //select register
	if(status != HAL_OK)
	{
		// TODO: log error
	}

	uint8_t pressureBuffer[3] = {0};
	status = HAL_I2C_Master_Receive(&hi2c1, shiftedAddress, pressureBuffer, 3, 1000); //read from register

	uint16_t pressure = (((uint16_t)pressureBuffer[0]) << 8 ) | pressureBuffer[1];
	return pressure;
}
