/*
 * flight_control.h
 *
 *  Created on: Dec 10, 2019
 *      Author: DC
 */

#ifndef FLIGHT_CONTROL_H_
#define FLIGHT_CONTROL_H_

#include <stdint.h>
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

void ExecuteFlightControlLoop();

typedef struct
{
	float deltaT;
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	float calcRoll;
	float calcPitch;
}LogData;




#define start_timer()    *((volatile uint32_t*)0xE0001000) = 0x40000001  // Enable CYCCNT register
#define stop_timer()   *((volatile uint32_t*)0xE0001000) = 0x40000000  // Disable CYCCNT register
#define get_timer()   *((volatile uint32_t*)0xE0001004)               // Get value from CYCCNT register
uint32_t it1, it2;      // start and stop flag
//stop_timer();               // If timer is not needed any more, stop

/*
 * Read specified register from MPU6050 module.
 *
 * Input: register address
 *
 * Output: return 8bit register value
 */
uint8_t readMPUreg(uint8_t reg);

/*
 * Write value to specified register
 *
 * Input: register address, value to write
 */
void writeMPUreg(uint8_t reg, uint8_t value);

#define accelRunningLen 4 // must be a power of 2
void accelRunningAverage(float* floatX, float* floatY, float* floatZ);

void readCurrentAccelerationValues(float* floatX, float* floatY, float* floatZ);

#define gyroRunningLen 4 // must be a power of 2
void gyroRunningAverage(float* floatX, float* floatY, float* floatZ);

void readCurrentGyroValues(float* floatX, float* floatY, float* floatZ);

void configMPUFilter();

extern float accelLpf;
extern float accelRollLpf;
extern float gyroLpf;

extern float accelXLPF;
extern float accelYLPF;
extern float accelZLPF;
extern float gyroXLPF;
extern float gyroYLPF;
extern float gyroZLPF;

void lpf(float* valueSoFar, float newSample, float lpfSetting);

// returns the PWM equivalent of RPM value
float pwm(float value);

void InitMPU();


#endif /* FLIGHT_CONTROL_H_ */
