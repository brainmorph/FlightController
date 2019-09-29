/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "circularbuffer.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t readMPUreg(uint8_t reg)
{
	uint16_t deviceAddress = 0x68;
	uint16_t shiftedAddress = deviceAddress << 1;
	uint8_t pData[100];
	pData[0] = reg; //register in question
	uint16_t Size = 1;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, shiftedAddress, pData, Size, 1000); //select register
	uint8_t value = 0;
	status = HAL_I2C_Master_Receive(&hi2c1, shiftedAddress, &value, 1, 1000); //read from register
	return value;
}
void writeMPUreg(uint8_t reg, uint8_t value)
{
	uint16_t deviceAddress = 0x68;
	uint16_t shiftedAddress = deviceAddress << 1;
	uint8_t pData[100];
	pData[0] = reg; //register in question
	pData[1] = value; //value to write
	uint16_t Size = 2; //we need to send 2 bytes of data (check out mpu datasheet... write register operation is defined this way)
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, shiftedAddress, pData, Size, 1000); //select register and write to it all in one
}

uint16_t cbDataLength(tCircularBuffer* cb)
{
	int32_t length = cb->write - cb->read;
	if(length > 0)
		return length;

	//otherwise
	length = cb->write + (length - cb->read);
	return length;
}

void cbBumpNewDataIn(tCircularBuffer* cb, float val)
{
	cb->read = (cb->read + 1) & (cb->size - 1);
	cb->buf[cb->write] = val;
	cb->write = (cb->write + 1) & (cb->size -1);
}

int cbWrite(tCircularBuffer* cb, float data)
{
	if(cbDataLength(cb) == (cb->size - 1))
	{
		//return -1; //buffer full
		cbBumpNewDataIn(cb, data);
	}

	//otherwise insert new data
	cb->buf[cb->write] = data;
	cb->write = (cb->write + 1) & (cb->size - 1); //this is possible because size is a power of 2
}

int cbRead(tCircularBuffer* cb, float* data)
{
	if(cbDataLength(cb) == 0)
		return -1; //empty buffer

	//otherwise return oldest data
	*data = cb->buf[cb->read];
	cb->read = (cb->read + 1) & (cb->size - 1); //this is possible because size is a power of 2
}

#define accelRunningLen 4 // must be a power of 2
void accelRunningAverage(float* floatX, float* floatY, float* floatZ)
{
	// These static declarations should only run one time
	static float x[accelRunningLen];
	static int xWrite = 0;
	static int xRead = accelRunningLen; //initialize to point to the end
	static float y[accelRunningLen];
	static int yWrite = 0;
	static int yRead = accelRunningLen; //initialize to point to the end
	static float z[accelRunningLen];
	static int zWrite = 0;
	static int zRead = accelRunningLen; //initialize to point to the end

	//insert new element
	x[xWrite] = *floatX;
	xWrite = (xWrite + 1) & (accelRunningLen - 1);
	xRead = (xRead + 1) & (accelRunningLen - 1);

	y[yWrite] = *floatY;
	yWrite = (yWrite + 1) & (accelRunningLen - 1);
	yRead = (yRead + 1) & (accelRunningLen - 1);

	z[zWrite] = *floatZ;
	zWrite = (zWrite + 1) & (accelRunningLen - 1);
	zRead = (zRead + 1) & (accelRunningLen - 1);

	float runAvgX = 0;
	float runAvgY = 0;
	float runAvgZ = 0;
	int readPtr = xRead;
	for(int i=0; i < accelRunningLen; i++)
	{
		readPtr = (xRead+i) & (accelRunningLen - 1); // wrap around after incrementing
		runAvgX += x[readPtr];
		runAvgY += y[readPtr];
		runAvgZ += z[readPtr];
	}

	runAvgX /= (float)accelRunningLen;
	runAvgY /= (float)accelRunningLen;
	runAvgZ /= (float)accelRunningLen;

	*floatX = runAvgX;
	*floatY = runAvgY;
	*floatZ = runAvgZ;
}

void readCurrentAccelerationValues(float* floatX, float* floatY, float* floatZ)
{
	// Read x,y,z acceleration registers. TODO: guarantee that these are from same sample
	volatile int16_t accelX = 0;
	accelX = readMPUreg(0x3B); //read accel X MSB value
	accelX = accelX << 8;
	accelX |= (0x00FF) & readMPUreg(0x3C); //read accel X LSB value

	volatile int16_t accelY = 0;
	accelY = readMPUreg(0x3D);
	accelY = accelY << 8;
	accelY |= (0x00FF) & readMPUreg(0x3E); // LSB

	volatile int16_t accelZ = 0;
	accelZ = readMPUreg(0x3F);
	accelZ = accelZ << 8;
	accelZ |= (0x00FF) & readMPUreg(0x40); // LSB

//	volatile int16_t temp = 0; // temperature
//	temp = readMPUreg(0x41); // MSB
//	temp = temp << 8;
//	temp |= (0x00FF) & readMPUreg(0x42); // LSB

	*floatX = (float)accelX * (float)(1.0/16384.0); //multiply reading with Full Scale value
	*floatY = (float)accelY * (float)(1.0/16384.0); //multiply reading with Full Scale value
	*floatZ = (float)accelZ * (float)(1.0/16384.0); //multiply reading with Full Scale value
	//*floatTemp = ((float)temp / 340.0) + 36.53;
}

#define gyroRunningLen 4 // must be a power of 2
void gyroRunningAverage(float* floatX, float* floatY, float* floatZ)
{
	// These static declarations should only run one time
	static float x[gyroRunningLen];
	static int xWrite = 0;
	static int xRead = gyroRunningLen; //initialize to point to the end
	static float y[gyroRunningLen];
	static int yWrite = 0;
	static int yRead = gyroRunningLen; //initialize to point to the end
	static float z[gyroRunningLen];
	static int zWrite = 0;
	static int zRead = gyroRunningLen; //initialize to point to the end

	//insert new element
	x[xWrite] = *floatX;
	xWrite = (xWrite + 1) & (gyroRunningLen - 1);
	xRead = (xRead + 1) & (gyroRunningLen - 1);

	y[yWrite] = *floatY;
	yWrite = (yWrite + 1) & (gyroRunningLen - 1);
	yRead = (yRead + 1) & (gyroRunningLen - 1);

	z[zWrite] = *floatZ;
	zWrite = (zWrite + 1) & (gyroRunningLen - 1);
	zRead = (zRead + 1) & (gyroRunningLen - 1);

	float runAvgX = 0;
	float runAvgY = 0;
	float runAvgZ = 0;
	int readPtr = xRead;
	for(int i=0; i < gyroRunningLen; i++)
	{
		readPtr = (xRead+i) & (gyroRunningLen - 1); // wrap around after incrementing
		runAvgX += x[readPtr];
		runAvgY += y[readPtr];
		runAvgZ += z[readPtr];
	}

	runAvgX /= (float)gyroRunningLen;
	runAvgY /= (float)gyroRunningLen;
	runAvgZ /= (float)gyroRunningLen;

	*floatX = runAvgX;
	*floatY = runAvgY;
	*floatZ = runAvgZ;
}

void readCurrentGyroValues(float* floatX, float* floatY, float* floatZ)
{
	// Read x,y,z acceleration registers. TODO: guarantee that these are from same sample
	volatile int16_t gyroX = 0;
	gyroX = readMPUreg(0x43); //read accel X MSB value
	gyroX = gyroX << 8;
	gyroX |= (0x00FF) & readMPUreg(0x44); //read gyro X LSB value

	volatile int16_t gyroY = 0;
	gyroY = readMPUreg(0x45);
	gyroY = gyroY << 8;
	gyroY |= (0x00FF) & readMPUreg(0x46); // LSB

	volatile int16_t gyroZ = 0;
	gyroZ = readMPUreg(0x47);
	gyroZ = gyroZ << 8;
	gyroZ |= (0x00FF) & readMPUreg(0x48); // LSB

	*floatX = (float)gyroX * (float)(1.0/131.0); //multiply reading with Full Scale value
	*floatY = (float)gyroY * (float)(1.0/131.0); //multiply reading with Full Scale value
	*floatZ = (float)gyroZ * (float)(1.0/131.0); //multiply reading with Full Scale value
}

float accelLpf = 0;
float accelRollLpf = 0;
float gyroLpf = 0;
float lpfSetting = 0.01; // how much of the new value to use
void lpf(float* valueSoFar, float newSample)
{
	float calc = *valueSoFar;
	calc = (1.0 - lpfSetting) * calc; // take a fraction of the value so far
	calc += lpfSetting * newSample; // add it to a fraction of the new sample

	*valueSoFar = calc;
}

// returns the PWM equivalent of RPM value
float pwm(float value)
{
	// map RPM value to pwm setting
	float min = 0.0; // RPM
	float max = 4000; // RPM

	float slope = (255)/(max-min) ; // rise/run

	return value * slope; // RPM * slope
}

uint32_t NOW_MS = 0;


//TODO: CLEAN UPVVVVVVVVVVVVVVVVVVVVVVVVVVV
#define start_timer()    *((volatile uint32_t*)0xE0001000) = 0x40000001  // Enable CYCCNT register
#define stop_timer()   *((volatile uint32_t*)0xE0001000) = 0x40000000  // Disable CYCCNT register
#define get_timer()   *((volatile uint32_t*)0xE0001004)               // Get value from CYCCNT register
uint32_t it1, it2;      // start and stop flag
//stop_timer();               // If timer is not needed any more, stop

//TODO: CLEAN UP^^^^^^^^^^^^^^^
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_TIM4_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_UART4_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0); //Enable the peripheral IRQ
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	HAL_TIM_Base_Start_IT(&htim6); //Start the timer

	HAL_Delay(500); // TODO: is this necessary?

	//let's try to read a register over I2C
	readMPUreg(0x75);
	readMPUreg(0x6B);
	writeMPUreg(0x6B, 0x00);
	readMPUreg(0x6B);
	readMPUreg(0x6B);

	readMPUreg(0x1C); //read accel config register



	uint32_t count=0;
	float avgAccelX=0, avgAccelY=0, avgAccelZ=0, gyroX=0, gyroY=0, gyroZ=0;
	float vX=0, vY=0, vZ=0;
	float aRoll=0, aPitch=0, aYaw=0; // angles with respect to each axis
	float deltaT = 0.010; // TODO: measure this with firmware timer

	float envAccelX=0, envAccelY=0, envAccelZ=0, envGyroX=0, envGyroY=0, envGyroZ = 0;

	// throw away a few samples at the beginning
	for(int i=0; i<20; i++)
	{
	  // read acceleration, filter with a running average
	  readCurrentAccelerationValues(&avgAccelX, &avgAccelY, &avgAccelZ);
	  accelRunningAverage(&avgAccelX, &avgAccelY, &avgAccelZ); // takes input and factors it into the running average for each variable
	  readCurrentGyroValues(&gyroX, &gyroY, &gyroZ);
	  gyroRunningAverage(&gyroX, &gyroY, &gyroZ);
	}

	// use the samples at the beginning as calibration
	envAccelX = avgAccelX;
	envAccelY = avgAccelY;
	envAccelZ = avgAccelZ;
	envGyroX = gyroX;
	envGyroY = gyroY;
	envGyroZ = gyroZ;

	//TODO: CLEAN UPVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
	start_timer();          // start the timer.
	it1 = get_timer();      // store current cycle-count in a local

	// do something

	it2 = get_timer() - it1;    // Derive the cycle-count difference
	it1 = it2;

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	int counter = 0;
	int counter2 = 0;

	void dumbFunction(void)
	{
	counter++;
	if(counter < 0)
	  counter = 0;
	if(counter2 < 20 || counter2 > 38)
	  counter2 = 20;

	it2 = get_timer();    // Derive the cycle-count difference
	it1 = it2 - it1;

	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);

	if(counter % 10 != 0) // slow this loop down
	  return;

	counter2++;

	//Just set the register directly
	htim4.Instance->CCR1 = counter2;
	htim4.Instance->CCR2 = counter2;
	htim4.Instance->CCR3 = counter2;
	htim4.Instance->CCR4 = counter2;

	it1 = it2;
	}

	// each setting represents motor throttle from 0 to 100%
	void setPWM(int motor1, int motor2, int motor3, int motor4)
	{
//		counter++;
//		if(counter < 0)
//			counter = 0;
//		if(counter % 10 != 0) // slow down PWM updates
//			return;

		// clip min/max motor output
		if(motor1 < 4)
			motor1 = 0;
		if(motor2 < 4)
			motor2 = 0;
		if(motor3 < 4)
			motor3 = 0;
		if(motor4 < 4)
			motor4 = 0;

		int motorMax = 25;
		if(motor1 > motorMax)
			motor1 = motorMax;
		if(motor2 > motorMax)
			motor2 = motorMax;
		if(motor3 > motorMax)
			motor3 = motorMax;
		if(motor4 > motorMax)
			motor4 = motorMax;


		// TODO: update min and max values to match new timer settings (I want higher resolution control)
		// TODO: new timer will be 80MHz with pre-scalar of 20 and counter period 80,000
		int max = 40; // based on 80MHz TIM4 with pre-scalar of 4000 and counter period 400
		int min = 20; // based on 80MHz TIM4 with pre-scalar of 4000 and counter period 400

		int range = max - min;

		// motor 1
		int setting = (motor1/100.0) * (float)range;
		setting += min; // add new value to minimum setting
		htim4.Instance->CCR1 = setting;

		// motor 2
		setting = (motor2/100.0) * (float)range;
		setting += min;
		htim4.Instance->CCR2 = setting;

		// motor 3
		setting = (motor3/100.0) * (float)range;
		setting += min;
		htim4.Instance->CCR3 = setting;

		// motor 4
		setting = (motor4/100.0) * (float)range;
		setting += min;
		htim4.Instance->CCR4 = setting;

		uint8_t uartData[70] = {0};
		snprintf(uartData, sizeof(uartData), "[%d, %d, %d, %d]   ",
				motor1, motor2, motor3, motor4);
		HAL_UART_Transmit(&huart4, uartData, 150, 5);
	}

	void mixPWM(float thrust, float roll, float pitch, float yaw)
	{
		float FR = thrust + yaw + pitch + roll;
		float FL = thrust - yaw + pitch - roll;
		float BR = thrust - yaw - pitch + roll;
		float BL = thrust + yaw - pitch - roll;

		setPWM(FL, FR, BR, BL);
	}
	//TODO: CLEAN UP ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	uint32_t PREVIOUS_MS = 0;
	float oldRollAngle = 0;
	float oldPitchAngle = 0;
	float thrustCmd = 0;
	uint8_t uartData[90] = {0}; // seems to make no significant time difference whether this happens here or inside the while loop
	while (1)
	{
	  	deltaT = (NOW_MS - PREVIOUS_MS)/1000.0;
	  	PREVIOUS_MS = NOW_MS;

//		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3); // scope this pin if you want to see main loop frequency

	  	// read raw accelerometer and gyro
	  	float aX = 0, aY = 0, aZ = 0, gX = 0, gY = 0, gZ = 0;
	  	readCurrentAccelerationValues(&aX, &aY, &aZ);
	  	readCurrentGyroValues(&gX, &gY, &gZ);

	  	// report raw values
		snprintf(uartData, sizeof(uartData), "<%ld, %+.3f, %+.2f, %+.2f, %+.2f, %+.2f, %+.2f, %+.2f>\r\n",
				count, deltaT, aX, aY, aZ, gX, gY, gZ);
		HAL_UART_Transmit(&huart4, uartData, 150, 5);

		if(NOW_MS < 12000)
		{
			mixPWM(0,0,0,0);
		}
		else
		{
			mixPWM(20,0,0,0);
		}

//		// read acceleration, filter with a running average
//	  	float oldAX = avgAccelX;
//	  	float oldAY = avgAccelY;
//	  	float oldAZ = avgAccelZ;
//
//		readCurrentAccelerationValues(&avgAccelX, &avgAccelY, &avgAccelZ);
//		accelRunningAverage(&avgAccelX, &avgAccelY, &avgAccelZ); // takes input and factors it into the running average for each variable
//
//		lpf(&oldAX, avgAccelX);
//		lpf(&oldAY, avgAccelY);
//		lpf(&oldAZ, avgAccelZ);
//
//		// at this point I should have the running average of floatX,Y,Z according to accelRunningAverage(...)
//
//		// for fun let's deadband accelZ.
////		float deadBandZ = (avgAccelZ - envAccelZ);
////		if(deadBandZ < 0.2 && deadBandZ > -0.2)
////		  deadBandZ = 0;
////
////		// derive velocity from acceleration
////		vX = (avgAccelX - envAccelX) * deltaT + vX; // subtract out calibration
////		vY = (avgAccelY - envAccelY) * deltaT + vY; // subtract out calibration
////		vZ = deadBandZ * deltaT + vZ; // subtract out calibration
////
////		volatile int dummy = 0;
////		dummy++;
////
////		// calculate error
////		float errorVZ = 0.0 - vZ; // my setpoint is 0 m/s for now.  + vZ because positive Z axis points down
////
////		// PID (P only for now)
////		float thrustRPM = 20000.0 * errorVZ;
////
////		// convert to PWM
////		int PWM = (int)pwm(thrustRPM);
////
////		if(PWM < 0)
////		  PWM = 0;
////		if(PWM > 255)
////		  PWM = 255;
//
//
//		// let's calculate gyro PID separately for now
//		float oldGX = gyroX;
//		float oldGY = gyroY;
//		float oldGZ = gyroZ;
//		readCurrentGyroValues(&gyroX, &gyroY, &gyroZ);
//		gyroRunningAverage(&gyroX, &gyroY, &gyroZ); // takes input and factors it into the running average for each variable
//
//		lpf(&oldGX, gyroX);
//		lpf(&oldGY, gyroY);
//		lpf(&oldGZ, gyroZ);
//
//		// try to Low Pass Filter
//		//lpf(&gyroLpf, gyroY); // lpf roll
//
//		// calculate roll angle from acceleration
//		float accelRoll = -1.0 * atan2f(oldAY, oldAZ); // sign flip to align with accelerometer orientation
//		accelRoll *= (180.0 / 3.1415); // convert to degrees
//
//		// calculate pitch angle from acceleration
//		float accelPitch = atan2f(oldAX, oldAZ);
//		accelPitch *= (180.0 / 3.1415); // convert to degrees
//
//		// complementary roll angle calculation
//		float partialAccelRoll = 0.03 * accelRoll; // take only 2% of acceleration calculated angle
//		float gyroRoll = -1.0 * oldGX * deltaT + oldRollAngle;
//		float partialGyroRoll = 0.97 * gyroRoll;
//		float calculatedRollAngle = partialAccelRoll + partialGyroRoll;
//		oldRollAngle = calculatedRollAngle;
//
//		// complementary pitch angle calculation
//		float partialAccelPitch = 0.03 * accelPitch;
//		float gyroPitch = -1.0 * oldGY * deltaT + oldPitchAngle;
//		float partialGyroPitch = 0.97 * gyroPitch;
//		float calculatedPitchAngle = partialAccelPitch + partialGyroPitch;
//		oldPitchAngle = calculatedPitchAngle;
//
//		float deadBandGX = gyroX - envGyroX;
//		float deadBandGY = gyroY - envGyroY;
//		float deadBandGZ = gyroZ - envGyroZ;
//
//		// deadband to get rid of additions from quad vibration
////		if(deadBandGX > -0.5 && deadBandGX < 0.5)
////			deadBandGX = 0;
////		if(deadBandGY > -0.5 && deadBandGY < 0.5)
////			deadBandGY = 0;
////		if(deadBandGZ > -0.5 && deadBandGZ < 0.5)
////			deadBandGZ = 0;
//
//		// derive orientation angle from angular velocity
//		aRoll = -deadBandGX * deltaT + aRoll;
//		aPitch = -deadBandGY * deltaT + aPitch;
//		aYaw = -deadBandGZ * deltaT + aYaw;
//
//		// calculate error terms
//		float errorARoll = 0.0 - calculatedRollAngle; // my setpoint is 0
//		float errorAPitch = 0.0 - calculatedPitchAngle; // my setpoint is 0
//		float errorAYaw = 0.0 - aYaw; // my setpoint is 0
//
//		// calculate angular command (proportional) terms
//		float kp = 0.01;
//		float rollCmd = kp * errorARoll;
//		float pitchCmd = kp * errorAPitch;
//		float yawCmd = 0; // TODO: calculate appropriate yaw command
//
//		if(NOW_MS < 12000)
//		{
//			thrustCmd = rollCmd = pitchCmd = yawCmd = 0;
//		}
//		else
//		{
//			thrustCmd = 20;
//		}
//
//		mixPWM(thrustCmd, rollCmd, pitchCmd, yawCmd);
//
////		uint8_t uartData[150] = {0};
////		snprintf(uartData, sizeof(uartData), "\r\n<%ld, %+.2f, %+.2f, %+.2f>\r\n",
////				count, deltaT, gyroRoll, calculatedRollAngle);
////		HAL_UART_Transmit(&huart4, uartData, 70, 0x00FF);
//
//		uint8_t uartReceive[2] = {0};
//		HAL_UART_Receive(&huart4, uartReceive, 1, 1);
//		if(uartReceive[0] == 'w')
//		{
//			//HAL_UART_Transmit(&huart4, uartReceive, 2, 0x00FF);
//			thrustCmd++;
//		}
//		if(uartReceive[0] == 's')
//		{
//			thrustCmd--;
//		}
//
//		if(thrustCmd < 0)
//			thrustCmd = 0;
//
//		uint8_t uartData[150] = {0};
//		snprintf(uartData, sizeof(uartData), "<%ld, %+.2f, %+.2f, %+.2f, %+.2f, %+.2f, %+.2f, %+.2f>\r\n",
//				count, deltaT, oldAX, oldAY, oldAZ, oldGX, oldGY, oldGZ);
//		HAL_UART_Transmit(&huart4, uartData, 150, 0x00FF);


	  count++;

	  //HAL_Delay(200);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 400;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 600;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 700;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 800;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 80;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void TIM6_DAC_IRQHandler(void) {
	// Pass the control to HAL, which processes the IRQ
	HAL_TIM_IRQHandler(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// This callback is automatically called by the HAL on the UEV event
	if(htim->Instance == TIM6)
	{
		NOW_MS++;

		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
