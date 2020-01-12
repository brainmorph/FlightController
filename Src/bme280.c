/*
 * bme280.c
 *
 *  Created on: Jan 7, 2020
 *      Author: DC
 */

#include <stdint.h>
#include "bme280.h"
#include "i2c.h"

#define DEVICE_ADDRESS 0x76; // use 0x76 even though data-sheet says 0x77

static struct BMP280_cal_param_t cal_param;
static int32_t t_fine;

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

uint32_t bme280ReadPressure()
{
	uint16_t shiftedAddress = DEVICE_ADDRESS;
	shiftedAddress = shiftedAddress << 1;
	uint8_t pData[3] = {0};
	pData[0] = 0xF7; // Register of MSB pressure value.  Plan is to read 2 more registers after this one for LSB and XSB
	uint16_t Size = 1;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, shiftedAddress, pData, Size, 1000); //select register
	if(status != HAL_OK)
	{
		// TODO: log error
	}

	status = HAL_I2C_Master_Receive(&hi2c1, shiftedAddress, pData, 3, 1000); //read from selected register

	uint32_t pressure = (uint32_t)(pData[0] << 12 | pData[1] << 4 | pData[2] >> 4);
	return pressure;
}

void bme280ReadAllRaw(int32_t *UT, int32_t *UP, int32_t *UH)
{
	// Clear result values
	*UT = 0x80000;
	*UP = 0x80000;
	*UH = 0x8000;

	uint16_t shiftedAddress = DEVICE_ADDRESS;
	shiftedAddress = shiftedAddress << 1;
	uint8_t pData[8] = {0};
	pData[0] = 0xF7; // Register of MSB pressure value.  Plan is to read 2 more registers after this one for LSB and XSB
	uint16_t Size = 1;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, shiftedAddress, pData, Size, 1000); //select register
	if(status != HAL_OK)
	{
		// TODO: log error
	}

	status = HAL_I2C_Master_Receive(&hi2c1, shiftedAddress, pData, 3, 1000); //read from selected register

	*UP = (int32_t)((pData[0] << 12) | (pData[1] << 4) | (pData[2] >> 4));
	*UT = (int32_t)((pData[3] << 12) | (pData[4] << 4) | (pData[5] >> 4));
	*UH = (int32_t)((pData[6] <<  8) |  pData[7]);

}

// DATASHEET SUGGESTS TO LIFT THE FOLLOWING CONVERSION CODE STRAIGHT FROM BME280 DRIVER REPO
// Calculate pressure from raw value, resolution is 0.001 Pa
// input:
//   UP - raw pressure value
// return: pressure in Pa as unsigned 32-bit integer in Q24.8 format (24 integer and 8 fractional bits)
// note: output value of '24674867' represents 24674867/256 = 96386.2 Pa = 963.862 hPa
// note: BME280_CalcT must be called before calling this function
// note: using 64-bit calculations
// note: code from the BME280 datasheet (rev 1.1)
uint32_t BME280_CalcP(int32_t UP) {
	int32_t v1,v2;
	uint32_t p;

	v1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	v2 = (((v1 >> 2) * (v1 >> 2)) >> 11 ) * ((int32_t)cal_param.dig_P6);
	v2 = v2 + ((v1 * ((int32_t)cal_param.dig_P5)) << 1);
	v2 = (v2 >> 2) + (((int32_t)cal_param.dig_P4) << 16);
	v1 = (((cal_param.dig_P3 * (((v1 >> 2) * (v1 >> 2)) >> 13 )) >> 3) + ((((int32_t)cal_param.dig_P2) * v1) >> 1)) >> 18;
	v1 = (((32768 + v1)) * ((int32_t)cal_param.dig_P1)) >> 15;
	if (v1 == 0) return 0; // avoid exception caused by division by zero
	p = (((uint32_t)(((int32_t)1048576) - UP) - (v2 >> 12))) * 3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((uint32_t)v1);
	} else {
		p = (p / (uint32_t)v1) << 1;
	}
	v1 = (((int32_t)cal_param.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	v2 = (((int32_t)(p >> 2)) * ((int32_t)cal_param.dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((v1 + v2 + cal_param.dig_P7) >> 4));

	// Convert pressure to Q24.8 format (fractional part always be .000)
	p <<= 8;

	return (uint32_t)p;
}

// Calculate temperature from raw value, resolution is 0.01 degree
// input:
//   UT - raw temperature value
// return: temperature in Celsius degrees (value of '5123' equals '51.23C')
// note: code from the BME280 datasheet (rev 1.1)
int32_t BME280_CalcT(int32_t UT) {
	t_fine  = ((((UT >> 3) - ((int32_t)cal_param.dig_T1 << 1))) * ((int32_t)cal_param.dig_T2)) >> 11;
	t_fine += (((((UT >> 4) - ((int32_t)cal_param.dig_T1)) * ((UT >> 4) - ((int32_t)cal_param.dig_T1))) >> 12) * ((int32_t)cal_param.dig_T3)) >> 14;

	return ((t_fine * 5) + 128) >> 8;
}
