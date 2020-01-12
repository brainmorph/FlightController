/*
 * bme280.h
 *
 *  Created on: Jan 7, 2020
 *      Author: DC
 */

#ifndef BME280_H_
#define BME280_H_

uint8_t bme280ReadReg(uint8_t reg);
void bme280WriteReg(uint8_t reg, uint8_t value);
uint32_t bme280ReadPressure();
void bme280ReadAllRaw(int32_t *UT, int32_t *UP, int32_t *UH);
int32_t BME280_CalcT(int32_t UT);
uint32_t BME280_CalcP(int32_t UP);

// Structure for compensation parameters
struct BMP280_cal_param_t {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
};

#endif /* BME280_H_ */
