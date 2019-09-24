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

#define len 4 // must be a power of 2
void accelRunningAverage(float* floatX, float* floatY, float* floatZ)
{
	// These static declarations should only run one time
	static float x[len];
	static int xWrite = 0;
	static int xRead = len; //initialize to point to the end
	static float y[len];
	static int yWrite = 0;
	static int yRead = len; //initialize to point to the end
	static float z[len];
	static int zWrite = 0;
	static int zRead = len; //initialize to point to the end

	//insert new element
	x[xWrite] = *floatX;
	xWrite = (xWrite + 1) & (len - 1);
	xRead = (xRead + 1) & (len - 1);

	y[yWrite] = *floatY;
	yWrite = (yWrite + 1) & (len - 1);
	yRead = (yRead + 1) & (len - 1);

	z[zWrite] = *floatZ;
	zWrite = (zWrite + 1) & (len - 1);
	zRead = (zRead + 1) & (len - 1);

	float runAvgX = 0;
	float runAvgY = 0;
	float runAvgZ = 0;
	int readPtr = xRead;
	for(int i=0; i < len; i++)
	{
		readPtr = (xRead+i) & (len - 1);
		runAvgX += x[readPtr];
		runAvgY += y[readPtr];
		runAvgZ += z[readPtr];
	}

	runAvgX /= (float)len;
	runAvgY /= (float)len;
	runAvgZ /= (float)len;

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

//	volatile int16_t temp = 0;
//	temp = readMPUreg(0x41); // MSB
//	temp = temp << 8;
//	temp |= (0x00FF) & readMPUreg(0x42); // LSB

	*floatX = (float)accelX * (float)(1.0/16384.0); //multiply reading with Full Scale value
	*floatY = (float)accelY * (float)(1.0/16384.0); //multiply reading with Full Scale value
	*floatZ = (float)accelZ * (float)(1.0/16384.0); //multiply reading with Full Scale value
	//*floatTemp = ((float)temp / 340.0) + 36.53;
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
	  readCurrentGyroValues(&envGyroX, &envGyroY, &envGyroZ);
  }

  // use the samples at the beginning as calibration
  envAccelX = avgAccelX;
  envAccelY = avgAccelY;
  envAccelZ = avgAccelZ;

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
	counter++;
	if(counter < 0)
		counter = 0;
	if(counter % 10 != 0) // slow down PWM updates
		return;

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
}
	//TODO: CLEAN UP ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  while (1)
  {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);

		// read acceleration, filter with a running average
		readCurrentAccelerationValues(&avgAccelX, &avgAccelY, &avgAccelZ);
		accelRunningAverage(&avgAccelX, &avgAccelY, &avgAccelZ); // takes input and factors it into the running average for each variable


		// at this point I should have the running average of floatX,Y,Z according to accelRunningAverage(...)

		// for fun let's deadband accelZ.
		float deadBandZ = (avgAccelZ - envAccelZ);
		if(deadBandZ < 0.2 && deadBandZ > -0.2)
		  deadBandZ = 0;

		// derive velocity from acceleration
		vX = (avgAccelX - envAccelX) * deltaT + vX; // subtract out calibration
		vY = (avgAccelY - envAccelY) * deltaT + vY; // subtract out calibration
		vZ = deadBandZ * deltaT + vZ; // subtract out calibration

		volatile int dummy = 0;
		dummy++;

		// calculate error
		float errorVZ = 0.0 - vZ; // my setpoint is 0 m/s for now.  + vZ because positive Z axis points down

		// PID (P only for now)
		float thrustRPM = 20000.0 * errorVZ;

		// convert to PWM
		int PWM = (int)pwm(thrustRPM);

		if(PWM < 0)
		  PWM = 0;
		if(PWM > 255)
		  PWM = 255;

		// TODO: clean this mess up
		if(PWM > 0)
			PWM = 25;

		setPWM(PWM, PWM, PWM, PWM);

		// let's calculate gyro PID separately for now
		readCurrentGyroValues(&gyroX, &gyroY, &gyroZ);
		float deadBandGX = gyroX - envGyroX;
		float deadBandGY = gyroY - envGyroY;
		float deadBandGZ = gyroZ - envGyroZ;
		if(deadBandGX > -6 && deadBandGX < 6)
			deadBandGX = 0;
		if(deadBandGY > -6 && deadBandGY < 6)
			deadBandGY = 0;
		if(deadBandGZ > -6 && deadBandGZ < 6)
			deadBandGZ = 0;

		// derive orientation angle from angular velocity
		aRoll = -deadBandGX * deltaT + aRoll;
		aPitch = -deadBandGY * deltaT + aPitch;
		aYaw = -deadBandGZ * deltaT + aYaw;

		uint8_t uartData[150] = {0};
		snprintf(uartData, sizeof(uartData), "<%ld, %+.2f, %+.2f, %+.2f>\r\n", count, aRoll, aPitch, aYaw);
		HAL_UART_Transmit(&huart4, uartData, 70, 0x00FF);


//	  // send PWM to UART
//	  uint8_t uartData[70];
//	  snprintf(uartData, sizeof(uartData), "<%ld, %f.2,%3d, %3d, %3d, %3d, %d>", count, vZ, PWM, PWM, PWM, PWM, NOW_MS);
//	  HAL_UART_Transmit(&huart4, uartData, 70, 0x00FF);  //TODO: choose the correct number of bytes to send
//
//	 //print stuff out (a.k.a send to UART)
//	  uint8_t uartData[150];
//	  snprintf(uartData, sizeof(uartData), "<%ld, %+.2f, %+.2f, %+.2f, %+.2f, %+.2f, %+.2f>\r\n", count, avgAccelX, avgAccelY, avgAccelZ,
//			  vX, vY, vZ);
//	  HAL_UART_Transmit(&huart4, uartData, 150, 0x00FF);  //TODO: choose the correct number of bytes to send

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
