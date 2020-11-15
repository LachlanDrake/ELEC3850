/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Fonts/fonts.h"
#include "../BSP/openx07v_c_lcd.h"
#include <math.h>
#include "stm32f407xx.h"
#include "stdio.h"
#include "inttypes.h"
#include <string.h>
#include "stm32f4xx_hal.h"
#include "lsm9ds1_reg.h"
#include "../BSP/touch_panel.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//State Machine type definition
typedef void* (*func_t)();

//IMU typedefs
typedef union {
	int16_t i16bit[3];
	uint8_t u8bit[6];
} axis3bit16_t;

typedef struct {
	void *hbus;
	uint8_t i2c_address;
	uint8_t cs_port;
	uint8_t cs_pin;
} sensbus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUS hi2c1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define    BOOT_TIME            20 //ms
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//General PVs
uint32_t stopDist;
uint16_t track, trackLight;

const int LEFT = 2;
const int RIGHT = 1;
const int FORWARD = 0;
const int REVERSE = 1;
const int HALL_EFFECT = 32;
const int ULTRASONIC = 4;
const int DRIVESPEED = 300;
const int TRACKONE= 80, TRACKTWO=45,TRACKTHREE= 75, TRACKFOUR=30,TRACKFIVE= 30, TRACKSIX=30;

//ADC PVs
uint32_t ADC_VALUE = 2;
uint32_t headlight = 2;
volatile int batCap;
enum {
	ADC_BUFFER_LEN = 8192
};
uint32_t ADC1_BUFFER[ADC_BUFFER_LEN];
uint32_t ADC2_BUFFER[ADC_BUFFER_LEN];
//Blinker variables
int blinkerFlag = 0;
int blinkerOnR = 0;
int blinkerOnL = 0;
int stopFlag = 1;
int countLcd = 0;

//Hall effect PVs
uint32_t newTime = 0, oldTime = 0;
volatile float speed;
volatile uint32_t quarterRevs = 0;
uint32_t storequarterRevs;

//Ultrasonic PVs
uint32_t sensor_time;
volatile uint16_t distance;
const uint16_t distanceLim = 40;
char buffer[100];
volatile uint8_t ultraFlag = 0;
volatile uint32_t sensorTime;

//State Machine PV
volatile uint16_t statechange = 0;

//Hall effect PV
char string[7], string2[18];

int counter;

//IMU private variables
static sensbus_t mag_bus = { &SENSOR_BUS, LSM9DS1_MAG_I2C_ADD_H, 0, 0 };
static sensbus_t imu_bus = { &SENSOR_BUS, LSM9DS1_IMU_I2C_ADD_H, 0, 0 };
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t data_raw_magnetic_field;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
static uint8_t tx_buffer[1000];
static stmdev_ctx_t dev_ctx_imu;
static stmdev_ctx_t dev_ctx_mag;
static float averageGyroZ;

//LCD variables

uint16_t linenum = 0;
const uint16_t MAX_ENTRY_COUNT = 159;
static uint16_t entryCount = 0;
static uint16_t displayEntryX = 1;
static uint16_t displayEntryY = 4;
char str[100]; //for displaying answer on lcd
uint8_t *state1String = "Wait state          ";
uint8_t *state2String = "Forward state       ";
uint8_t *state3String = "Reverse state       ";
uint8_t *state4String = "Clockwise state     ";
uint8_t *state5String = "Anti-clockwise state";
uint8_t *state6String = "Traffic Light state";
uint8_t *stateString="Wait state         ";

//wifi receive variables
//uint16_t uart_rx_data_waiting = 0;
////uint16_t uart_tx_data_waiting = 0;
//uint8_t rx_data[1];
//uint8_t del = 2;
//uint8_t cn = 0;

//wifi receive variables
uint8_t rx_data[3];
int i;
char *time;
volatile char light;
char one;
char two;
volatile int trafficStop;
float distTravelled = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//General PFPs
void driveForward();
void reverse();
void turnLeft();
void turnRight();
void centreWheels(int direction);
void brake(int direction);
void stop(void);
uint32_t avoidObstacle(int direction);
uint8_t BSP_TP_Init(void);
uint8_t BSP_TP_GetDisplayPoint(Coordinate *pDisplay);

//LCD PFPs
void LCD_Print(uint8_t *stateString);
void CalculatorInit(void);
void CalculatorProcess(void);
void LCDEntry1(uint8_t *textEntry);
void LCDEntry2(uint8_t *textEntry, int length);
void LCDEntry(uint16_t displayEntryX, uint16_t displayEntryY,
		uint8_t *textEntry, uint16_t entryCount);
int distSelect();
int trackSelect();

//Hall Effect PFP
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

//Ultrasonic PFPs
uint32_t Read_HCSR04();
void getDistance(void);

//Servo PFPs
void servo_write(int);
int map(int, int, int, int, int);
void delay_us(uint16_t us);

//IMU PFPs
void IMU_Init(void);
void readIMU(void);
static int32_t platform_write_imu(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static int32_t platform_write_mag(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static int32_t platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static void platform_delay(uint32_t ms);

//State functions
void* state1();
void* state2();
void* state3();
void* state4();
void* state5();
void* state6();
volatile func_t state = state1;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//char buffer[100];
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	CalculatorInit();
	IMU_Init();
	HAL_UART_Receive_DMA(&huart2, rx_data, 3);
	HAL_ADC_Start_DMA(&hadc1, ADC1_BUFFER, ADC_BUFFER_LEN);
	HAL_ADC_Start_DMA(&hadc2, ADC2_BUFFER, ADC_BUFFER_LEN);

	//  // Display welcome message
	//  BSP_LCD_Clear (LCD_COLOR_WHITE);
	//  BSP_LCD_SetFont (&Font24);
	//  BSP_LCD_SetTextColor (LCD_COLOR_BLACK);

	//BSP_LCD_DisplayStringAtLine (2, (uint8_t*) "Hello World");
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim2);

	HAL_TIM_Base_Start(&htim3);               //Initialize stm32 timer 3
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim4);               //Initialize stm32 timer 3
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim5);
	stop();

	centreWheels(FORWARD);
	//servo_sweep();

	//lsm9ds1_read_data_polling();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
		(func_t) (*state)();

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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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

/* USER CODE BEGIN 4 */

//State Machine function definitions
//Wait state
void* state1() {
	//fill in with state instructions, eg drive forward
	stateString=state1String;
	stop();
	speed = 0;
	getDistance();
//	LCD_Print(state1String);
	readIMU();
	CalculatorProcess();
	return 0;
	//return nextState;
}

//Drive forward state
void* state2() {
	stateString=state2String;
	//fill in with state instructions, eg drive forward
	quarterRevs = 0;
	//sprintf(state2String, "Forward state- %d cm", (int)stopDist);
	centreWheels(FORWARD);
	driveForward();
	while (distance > (stopDist + 5)) {
		if (state != state2) {
			stop();
			return 0;
		}
		centreWheels(FORWARD);
		getDistance();
		readIMU();
//		LCD_Print(state2String);
	}
	brake(FORWARD);
	speed = 0;
//	LCD_Print(state2String);
	HAL_Delay(2000);
	state = state3;
	storequarterRevs = quarterRevs;
	quarterRevs = 0;
	stopDist = 0;
	return 0;

}

//Reverse State
void* state3() {
	stateString=state3String;
	centreWheels(REVERSE);
	reverse();
	while (quarterRevs < storequarterRevs) {
		//Add centre wheels for reverse mode?
		getDistance();
		readIMU();
		centreWheels(REVERSE);
//		LCD_Print(state3String);
	}
	brake(REVERSE);
	quarterRevs = 0;
	state = state1;
	return 0;

}

//Clockwise state
void* state4() {
	stateString=state4String;
//	LCD_Print(state4String);
	readIMU();
	getDistance();
	quarterRevs = 0;

	//First straight
	while (quarterRevs < 65) {
		driveForward();
		centreWheels(FORWARD);
		readIMU();
		getDistance();

		if (distance <= 30) {
			quarterRevs = avoidObstacle(RIGHT);

		}
//		LCD_Print((uint8_t*) "Clockwise state");
		if (state != state4) {
			return 0;
		}
	}
	brake(FORWARD);

	//First turn
	turnRight();
	quarterRevs = 0;
	while (quarterRevs < 34) {
		driveForward();
		readIMU();
		getDistance();
		while (distance <= 30) {
			stop();
			getDistance();
		}

//		LCD_Print((uint8_t*) "Clockwise state");
		if (state != state4) {
			return 0;
		}
	}
	blinkerFlag = 0;

	//Second straight
	centreWheels(FORWARD);
	quarterRevs = 0;
	while (quarterRevs < 15) {
		centreWheels(FORWARD);
	}
	brake(FORWARD);

	//Second turn
	turnRight();
	quarterRevs = 0;
	while (quarterRevs < 32) {
		driveForward();
		readIMU();
		getDistance();
		while (distance <= 30) {
			stop();
			getDistance();
		}

//		LCD_Print((uint8_t*) "Clockwise state");
		if (state != state4) {
			return 0;
		}
	}

	blinkerFlag = 0;

	//Third straight
	centreWheels(FORWARD);
	quarterRevs = 0;
	while (quarterRevs < 54) {
		driveForward();
		centreWheels(FORWARD);
		readIMU();
		getDistance();
		if (distance <= 30) {
			quarterRevs = avoidObstacle(RIGHT);

		}

		LCD_Print((uint8_t*) "Clockwise state");
		if (state != state4) {
			return 0;
		}
	}
	stop();
	brake(FORWARD);

	//Third turn
	turnRight();
	quarterRevs = 0;
	while (quarterRevs < 32) {
		driveForward();
		readIMU();
		getDistance();
		while (distance <= 30) {
			stop();
			getDistance();
		}

//		LCD_Print((uint8_t*) "Clockwise state");
		if (state != state4) {
			return 0;
		}
	}

	blinkerFlag = 0;

	centreWheels(FORWARD);
	quarterRevs = 0;
	while (quarterRevs < 16) {
		centreWheels(FORWARD);
	}

	brake(FORWARD);
	turnRight();
	quarterRevs = 0;
	while (quarterRevs < 32) {
		driveForward();
		readIMU();
		getDistance();
		while (distance <= 30) {
			stop();
			getDistance();
		}

//		LCD_Print((uint8_t*) "Clockwise state");
		if (state != state4) {
			return 0;
		}
	}

	blinkerFlag = 0;

	centreWheels(FORWARD);
	quarterRevs = 0;

	stop();
	state = state1;

	return 0;
}

//Anti-clockwise state
void* state5() {
	stateString=state5String;
//	LCD_Print(state5String);
	readIMU();
	getDistance();

	//First turn
	turnLeft();
	quarterRevs = 0;
	while (quarterRevs < 36) {
		driveForward();
		readIMU();
		getDistance();
		while (distance <= 30) {
			stop();
			getDistance();
		}

//		LCD_Print(state5String);
		if (state != state5) {
			return 0;
		}
	}

	blinkerFlag = 0;

	//First straight
	centreWheels(FORWARD);
	quarterRevs = 0;
	while (quarterRevs < 14) {
		centreWheels(FORWARD);
//		LCD_Print(state5String);
	}
	stop();
	brake(FORWARD);

	//Second turn
	turnLeft();
	quarterRevs = 0;
	while (quarterRevs < 31) {
		driveForward();
		readIMU();
		getDistance();
		while (distance <= 30) {
			stop();
			getDistance();
		}

//		LCD_Print(state5String);
		if (state != state5) {
			return 0;
		}
	}
	blinkerFlag = 0;

	//Second straight
	centreWheels(FORWARD);
	quarterRevs = 0;
	while (quarterRevs < 54) {
		driveForward();
		centreWheels(FORWARD);
		readIMU();
		getDistance();
		if (distance <= 30) {
			quarterRevs = avoidObstacle(LEFT);

		}
//		LCD_Print(state5String);
		if (state != state5) {
			return 0;
		}
	}
	stop();
	brake(FORWARD);

	//Third turn
	turnLeft();
	quarterRevs = 0;
	while (quarterRevs < 32) {
		driveForward();
		readIMU();
		getDistance();
		while (distance <= 30) {
			stop();
			getDistance();
		}

//		LCD_Print(state5String);
		if (state != state5) {
			return 0;
		}
	}
	blinkerFlag = 0;

	//Third straight
	centreWheels(FORWARD);
	quarterRevs = 0;
	while (quarterRevs < 11) {
		centreWheels(FORWARD);
		while (distance <= 30) {
			stop();
			getDistance();
		}
	}
	brake(FORWARD);

	//Fourth turn
	turnLeft();
	quarterRevs = 0;
	while (quarterRevs < 30) {
		driveForward();
		readIMU();
		getDistance();
		while (distance <= 30) {
			stop();
			getDistance();
		}

//		LCD_Print(state5String);
		if (state != state5) {
			return 0;
		}
	}
	blinkerFlag = 0;
	//Final straight
	while (quarterRevs < 82) {
		driveForward();
		centreWheels(FORWARD);
		readIMU();
		getDistance();

		if (distance <= 30) {
			quarterRevs = avoidObstacle(LEFT);

		}
//		LCD_Print(state5String);
		if (state != state5) {
			return 0;
		}
	}
	brake(FORWARD);
	stop();
	quarterRevs = 0;
	state = state1;
	return 0;

}

void* state6() {
	stateString=state6String;
	quarterRevs = 0;
//sprintf(state2String, "Forward state- %d cm", (int)stopDist);
	centreWheels(FORWARD);

	while (quarterRevs < trackLight) {               //traffic light distance
		while(distance<30){
			stop();
			getDistance();
		}
		driveForward();
		centreWheels(FORWARD);
		getDistance();
		readIMU();
//		LCD_Print(state2String);
	}
	if (trafficStop) { //not green
		brake(FORWARD);

		while (trafficStop) { //not green
//			LCD_Print(state2String);
			getDistance();
			readIMU();
		}
	}

	while (quarterRevs < track) { //Drive to end of track
		while(distance<30){

					stop();
					getDistance();
					HAL_Delay(100);
				}
		driveForward();
		centreWheels(FORWARD);
		getDistance();
		readIMU();
//		LCD_Print(state2String);
	}

	stop();
	brake(FORWARD);
state=state1;
	return 0;
}

//General functions

void driveForward() {
	stopFlag = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); //Sets the Direction
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	TIM3->CCR1 = DRIVESPEED; //Sets the speed
}
void reverse() {
	stopFlag = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); //Sets the Direction
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	TIM3->CCR1 = DRIVESPEED; //Sets the speed
}

void turnLeft() {

//add turn left blinker on
	servo_write(70);
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); //Sets the Direction
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	blinkerFlag = LEFT;
//add speed control

}
void turnRight() {

//add turn right blinker on
	servo_write(120);
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); //Sets the Direction
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	blinkerFlag = RIGHT;
//add speed control
}
void centreWheels(int direction) {
//servo_write(97);
	blinkerFlag = FORWARD;
	if (direction == FORWARD) {
		if (averageGyroZ > 1000) {
//htim1.Instance->CCR1 =158;
			servo_write(99);
		} else if (averageGyroZ < -1000) {
//htim1.Instance->CCR1 =156;
			servo_write(93);
		} else {
//htim1.Instance->CCR1 =157;
			servo_write(96);
		}
	} else {
		if (averageGyroZ > 1000) {
//htim1.Instance->CCR1 =158;
			servo_write(93);
		} else if (averageGyroZ < -1000) {
//htim1.Instance->CCR1 =156;
			servo_write(99);
		} else {
//htim1.Instance->CCR1 =157;
			servo_write(96);
		}
	}
}

void brake(int direction) {
	if (direction == FORWARD) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); //Sets the Direction
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); //Sets the Direction
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	}
	TIM3->CCR1 = 498;
	HAL_Delay(200);
	TIM3->CCR1 = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); //Sets the Direction
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
//Turns off blinkers and headlights
	TIM4->CCR1 = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	stopFlag = 1;

//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); //Sets the Direction
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
//			HAL_Delay(100);
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); //Sets the Direction
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

}

void stop(void) {
	stopFlag = 1;
	TIM3->CCR1 = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); //Sets the Direction
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	TIM4->CCR1 = 0;
}

uint32_t avoidObstacle(int direction) {
	uint16_t distanceLimit = 30;
	int reverseDist, equivDist, straightDist, rightDist, leftDist;
	storequarterRevs = quarterRevs;

	if (distanceLimit <= distanceLim) {
		brake(FORWARD);
	}
	quarterRevs = 0;
	reverse();
	//HAL_Delay(40);
	while (distance <= distanceLim) {
		getDistance();

	}
	brake(REVERSE);
	reverseDist = quarterRevs;
	quarterRevs = 0;
	if (direction == RIGHT) {
		turnRight();
		driveForward(); // for 5 seconds
		HAL_Delay(1500);
		rightDist = quarterRevs;
		quarterRevs = 0;
		turnLeft(); // for 5 seconds
		HAL_Delay(2200);
		leftDist = quarterRevs;
		quarterRevs = 0;
		turnRight();
		HAL_Delay(1540);
		centreWheels(FORWARD);
		straightDist = quarterRevs;
		//HAL_Delay(1600);
		brake(FORWARD);
		//HAL_Delay(20000);
	} else {
		turnLeft();
		driveForward(); // for 5 seconds
		HAL_Delay(1600);
		leftDist = quarterRevs;
		quarterRevs = 0;
		turnRight();
		rightDist = quarterRevs;
		quarterRevs = 0;
		HAL_Delay(2200);
		turnLeft();
		HAL_Delay(1540);
		centreWheels(FORWARD);
		straightDist = quarterRevs;
		//HAL_Delay(1600);
		brake(FORWARD);
		//HAL_Delay(20000);
	}

	equivDist = storequarterRevs - reverseDist + leftDist + rightDist
			+ straightDist;
	return equivDist;
}

//Hall effect sensor function definitions
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//Hall effect interrupt
	if (GPIO_Pin == HALL_EFFECT) {
		newTime = HAL_GetTick();
		speed = 49.0 / (newTime - oldTime);	//Calculate speed in m/s, quarter wheel circumference is 49mm
		oldTime = newTime;
		quarterRevs++;
	}

		if (!stopFlag) {
		//		if (headlight > 2000) {
		//			TIM4->CCR1 = 1000;
		//		} else {
		//			TIM4->CCR1 = 100;
		//		}
//				TIM4->CCR1=300*log(headlight);
			TIM4->CCR1= 110*exp(headlight/1000);

			}

	if (GPIO_Pin == ULTRASONIC) {
		ultraFlag = 0;
		int time = __HAL_TIM_GET_COUNTER(&htim2);
		distance = time / 60;
	}
	if ((quarterRevs % 4) == 0) {
		if (blinkerFlag == RIGHT && !blinkerOnR) {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
			blinkerOnR = 1;
		} else if (blinkerFlag == LEFT && !blinkerOnL) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
			blinkerOnL = 1;
		} else {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
			blinkerOnR = 0;
			blinkerOnL = 0;
		}
	}

}

//Ultrasonic function definitions
uint32_t Read_HCSR04() {
	uint32_t local_time = 0;
	uint32_t local_time1 = 0;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);	// pull the trig pin high
	delay_us(10);										// wait for 10 us
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);// pull the trig pin low

	// wait for the echo pin to go high

	while (!(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2))) {
		local_time1++;									// increment local time
		delay_us(1);									// every 1 us
		if (local_time1 > 60000) {
			return 60000;
		}
	}

	while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2))			// while the pin is high
	{
		local_time++;									// increment local time
		delay_us(1);									// every 1 us

	}

	return local_time;
}

//void getDistance_EXTI(){
//	if(!ultraFlag || (__HAL_TIM_GET_COUNTER(&htim2)>10000)){
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);	// pull the trig pin high
//		delay_us(10);										// wait for 10 us
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
//		__HAL_TIM_SET_COUNTER(&htim2, 0);	//reset microsecond counter to 0
//		ultraFlag = 1;
//	}
//
//}

void getDistance(void) {
	uint32_t sensor_time = Read_HCSR04();			// get the high time
	distance = sensor_time / 60;
	return;
}

void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		;  // wait for the counter to reach the us input in the parameter
}

//Servo function defs
void servo_write(int angle) {
	htim1.Instance->CCR1 = map(0, 180, 50, 250, angle);
}

int map(int st1, int fn1, int st2, int fn2, int value) {
	return (1.0 * (value - st1)) / ((fn1 - st1) * 1.0) * (fn2 - st2) + st2;
}

//IMU Functions
void IMU_Init(void) {

	/* Initialize inertial sensors (IMU) driver interface */
	dev_ctx_imu.write_reg = platform_write_imu;
	dev_ctx_imu.read_reg = platform_read_imu;
	dev_ctx_imu.handle = (void*) &imu_bus;

	/* Initialize magnetic sensors driver interface */
	dev_ctx_mag.write_reg = platform_write_mag;
	dev_ctx_mag.read_reg = platform_read_mag;
	dev_ctx_mag.handle = (void*) &mag_bus;

	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);
	if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID) {
		while (1) {
			/* manage here device not found */
			LCD_Print((uint8_t*) "IMU NOT FOUND");
		}
	}
	/* Restore default configuration */
	lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
	do {
		lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

	/* Set full scale */
	lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
	lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
	lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);

	/* Configure filtering chain - See datasheet for filtering chain details */
	/* Accelerometer filtering chain */
	lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
	lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
	lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
	/* Gyroscope filtering chain */
	lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
	lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
	lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);

	/* Set Output Data Rate / Power mode */
	lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
	lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

}

void readIMU() {
	lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

	if (reg.status_imu.xlda && reg.status_imu.gda) {
		/* Read imu data */
		memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
		memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

		lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration.u8bit);
		lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate.u8bit);

		acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(
				data_raw_acceleration.i16bit[0]);
		acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(
				data_raw_acceleration.i16bit[1]);
		acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(
				data_raw_acceleration.i16bit[2]);

		angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(
				data_raw_angular_rate.i16bit[0]);
		angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(
				data_raw_angular_rate.i16bit[1]);
		//angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);
		float gyroTotal = 0, gyroZValue;
		int i = 0, N = 10;
		for (i = 0; i < N; i++) {
			gyroZValue = lsm9ds1_from_fs2000dps_to_mdps(
					data_raw_angular_rate.i16bit[2]);
			gyroTotal += gyroZValue;

		}
		averageGyroZ = gyroTotal / N;

		sprintf((char*) tx_buffer,
				"IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
				acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
				angular_rate_mdps[0], angular_rate_mdps[1],
				angular_rate_mdps[2]);
		// tx_com(tx_buffer, strlen((char const*)tx_buffer));
	}

	if (reg.status_mag.zyxda) {
		/* Read magnetometer data */
		memset(data_raw_magnetic_field.u8bit, 0x00, 3 * sizeof(int16_t));

		lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field.u8bit);

		magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(
				data_raw_magnetic_field.i16bit[0]);
		magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(
				data_raw_magnetic_field.i16bit[1]);
		magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(
				data_raw_magnetic_field.i16bit[2]);

		sprintf((char*) tx_buffer, "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
				magnetic_field_mgauss[0], magnetic_field_mgauss[1],
				magnetic_field_mgauss[2]);
		// tx_com(tx_buffer, strlen((char const*)tx_buffer));
	}
	return;
}

/*
 * @brief  Write generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_imu(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	sensbus_t *sensbus = (sensbus_t*) handle;

	HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

	return 0;
}

/*
 * @brief  Write generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_mag(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	sensbus_t *sensbus = (sensbus_t*) handle;

	/* Write multiple command */
	reg |= 0x80;
	HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

	return 0;
}

/*
 * @brief  Read generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	sensbus_t *sensbus = (sensbus_t*) handle;

	HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

	return 0;
}

/*
 * @brief  Read generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	sensbus_t *sensbus = (sensbus_t*) handle;

	/* Read multiple command */
	reg |= 0x80;
	HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms) {
	HAL_Delay(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */

// Called when first half of buffer is filled
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {

}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	ADC_VALUE = ((float) ADC1_BUFFER[0] / 4095) * 9200;
	headlight = ((float) ADC2_BUFFER[0]);
	//HAL_ADC_Stop_DMA(&hadc1);

}

uint8_t BSP_TP_Init(void) {
	// Initialise the interface and calibrate
	TP_Init(); // This is an empty function since done by STM32CubeMX
	TouchPanel_Calibrate();

	return 0;
}

uint8_t BSP_TP_GetDisplayPoint(Coordinate *pDisplay) {
	Coordinate *pScreen;

	pScreen = Read_Ads7846();
	if (pScreen == NULL) {
		return 1; // Error reading the touch panel
	}
	if (getDisplayPoint(pDisplay, pScreen, &matrix) == DISABLE) {
		return 1; // Error in LCD
	}
	return 0;
}

//LCD Functions
void LCD_Print(uint8_t *stateString) {
	//countLcd++;
	BSP_LCD_DisplayStringAtLine(0, stateString);
	sprintf(string2, "Speed: %.4f m/s", speed);
	BSP_LCD_DisplayStringAt(160, 75, (uint8_t*) string2, 0);
	//HAL_ADC_Start_DMA(&hadc1, ADC_BUFFER, ADC_BUFFER_LEN);
	sprintf(buffer, "Bat. Capacity: %4d %%", (int) batCap);
	BSP_LCD_DisplayStringAt(160, 95, (uint8_t*) buffer, 0);
	sprintf(buffer, "Distance: %4d cm", distance);
	BSP_LCD_DisplayStringAt(160, 115, (uint8_t*) buffer, 0);
	//	sprintf(buffer, "GYR x [mdps]:%4.2f", angular_rate_mdps[0]);
	//	BSP_LCD_DisplayStringAt(160,175,(uint8_t*)buffer,0);
	//	sprintf(buffer, "GYR y [mdps]:%4.2f", angular_rate_mdps[1]);
	//	BSP_LCD_DisplayStringAt(160,135,(uint8_t*)buffer,0);
	sprintf(buffer, "GYR z [mdps]:%4.2f", averageGyroZ);
	BSP_LCD_DisplayStringAt(160, 155, (uint8_t*) buffer, 0);
//	if(countLcd == 43){
//		printf("Bat. Voltage: %4d mV GYR z [mdps]:%4.2f Distance: %4d cm  \n",
//				(int) ADC_VALUE,averageGyroZ,distance);
//		countLcd = 0;
//	}

	//printf("Hello\n");
	//	sprintf(buffer, "MAG x [mG]:%4.2f", magnetic_field_mgauss[0]);
	//	BSP_LCD_DisplayStringAtLine (5, (uint8_t*)buffer);
	//	sprintf(buffer, "MAG y [mG]:%4.2f", magnetic_field_mgauss[1]);
	//	BSP_LCD_DisplayStringAtLine (6, (uint8_t*)buffer);
	//	sprintf(buffer, "MAG z [mG]:%4.2f", magnetic_field_mgauss[2]);
	//	BSP_LCD_DisplayStringAtLine (7, (uint8_t*)buffer);
}

void CalculatorInit(void) {
	// STEPIEN: Assume horizontal display

	// Initialize and turn on LCD and calibrate the touch panel
	BSP_LCD_Init();
	BSP_LCD_DisplayOn();
	BSP_TP_Init();

	// Display welcome message
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	//BSP_LCD_DisplayStringAtLine (0, (uint8_t*) "JCNK Calculator");
	//BSP_LCD_DisplayStringAtLine (1, (uint8_t*) VER_STRING);
	//BSP_LCD_DisplayStringAtLine (2, (uint8_t*) "Calculator Example");

	// Create colour choices
	//  BSP_LCD_SetTextColor (LCD_COLOR_RED);
	//  BSP_LCD_FillRect (5, 200, 30, 30);
	//  BSP_LCD_SetTextColor (LCD_COLOR_BLUE);
	//  BSP_LCD_FillRect (40, 200, 30, 30);
	//  BSP_LCD_SetTextColor (LCD_COLOR_GREEN);
	//  BSP_LCD_FillRect (75, 200, 30, 30);
	//  BSP_LCD_SetTextColor (LCD_COLOR_BLACK);
	//  BSP_LCD_FillRect (110, 200, 30, 30);
	//  BSP_LCD_SetTextColor (LCD_COLOR_MAGENTA);
	//  BSP_LCD_FillRect (145, 200, 30, 30);
	//  BSP_LCD_SetTextColor (LCD_COLOR_ORANGE);
	//  BSP_LCD_FillRect (180, 200, 30, 30);
	//  BSP_LCD_SetTextColor (LCD_COLOR_CYAN);
	//  BSP_LCD_FillRect (215, 200, 30, 30);
	//  BSP_LCD_SetTextColor (LCD_COLOR_YELLOW);
	//  BSP_LCD_FillRect (250, 200, 30, 30);
	//

	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

	//LEFT HALF
	BSP_LCD_DrawHLine(2, 39, 150);
	BSP_LCD_DrawHLine(2, 59, 150);
	BSP_LCD_DrawHLine(2, 149, 150);
	BSP_LCD_DrawHLine(2, 169, 150);
	BSP_LCD_DrawHLine(2, 227, 150);

	//Draw Vertical Lines for Button Grid
	BSP_LCD_DrawVLine(52, 60, 89);
	BSP_LCD_DrawVLine(102, 60, 89);
	BSP_LCD_DrawHLine(2, 105, 150);

	//BUTTON
	BSP_LCD_DrawVLine(2, 39, 189);
	BSP_LCD_DrawVLine(151, 39, 189);

	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DisplayStringAt(23, 77, "1", 0);
	BSP_LCD_DisplayStringAt(23, 122, "4", 0);
	BSP_LCD_DisplayStringAt(73, 77, "2", 0);
	BSP_LCD_DisplayStringAt(73, 122, "5", 0);
	BSP_LCD_DisplayStringAt(123, 77, "3", 0);
	BSP_LCD_DisplayStringAt(123, 122, "6", 0);

	BSP_LCD_DisplayStringAt(4, 45, "States:", 0);
	BSP_LCD_DisplayStringAt(4, 155, "Travel Map:", 0);

	//  //RIGHT HALF
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_DrawHLine(158, 39, 159);
	BSP_LCD_DrawHLine(158, 227, 159);
	BSP_LCD_DrawHLine(158, 70, 159);
	BSP_LCD_DrawHLine(158, 110, 159);
	BSP_LCD_DrawHLine(158, 150, 159);
	BSP_LCD_DrawHLine(158, 190, 159);
	BSP_LCD_DrawHLine(158, 90, 159);
	BSP_LCD_DrawHLine(158, 130, 159);
	BSP_LCD_DrawHLine(158, 170, 159);
	BSP_LCD_DrawHLine(158, 210, 159);
	//
	//Draw Vertical Lines for status bar
	BSP_LCD_DrawVLine(158, 39, 189);
	BSP_LCD_DrawVLine(317, 39, 189);

	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

	BSP_LCD_SetFont(&Font16);
	BSP_LCD_DisplayStringAt(203, 49, "Status", 0);
	BSP_LCD_SetFont(&Font12);
	//BSP_LCD_DisplayStringAt(160,75,"Speed: 1.0987 m/s",0);
	//	BSP_LCD_DisplayStringAt(160,95,"Distance: 400cm",0);
	//	BSP_LCD_DisplayStringAt(160,175,"Motor Current: 2mA",0);
	//	BSP_LCD_DisplayStringAt(160,135,"Direction: North",0);
	//	BSP_LCD_DisplayStringAt(160,155,"Orientation:AClockwise",0);
	//	BSP_LCD_DisplayStringAt(160,175,"Bat. Voltage: 1234 mV",0);
	BSP_LCD_DisplayStringAt(160, 195, " ", 0);

	//BSP_LCD_SetFont (&Font8);
	//BSP_LCD_DisplayStringAt(1,228,"Battery Voltage value: ",0);
	//BSP_LCD_SetFont (&Font12);
}

void LCDEntry(uint16_t displayEntryX, uint16_t displayEntryY,
		uint8_t *textEntry, uint16_t entryCount) {
	const uint16_t MAX_ENTRY_COUNT = 159;
	//BSP_LCD_DisplayStringAt(displayEntryX,displayEntryY,"1",0);
	if ((displayEntryX >= 312)
			&& ((displayEntryY == 4) || (displayEntryY == 15))) {
		BSP_LCD_DisplayStringAt(displayEntryX, displayEntryY, textEntry, 0);
		displayEntryY += 11;
		displayEntryX = 1;
		entryCount++;
	} else if (entryCount == MAX_ENTRY_COUNT) {
		BSP_LCD_SetFont(&Font8);
		BSP_LCD_DisplayStringAt(1, 231, "Max commands entered!", 0);
	} else {
		BSP_LCD_DisplayStringAt(displayEntryX, displayEntryY, textEntry, 0);
		entryCount++;
		displayEntryX += 6;
	}
}
;

void LCDEntry1(uint8_t *textEntry) {

	//BSP_LCD_DisplayStringAt(displayEntryX,displayEntryY,"1",0);
	if ((displayEntryX >= 312)
			&& ((displayEntryY == 4) || (displayEntryY == 15))) {
		BSP_LCD_DisplayStringAt(displayEntryX, displayEntryY, textEntry, 0);
		displayEntryY += 11;
		displayEntryX = 1;
		entryCount++;
	} else if (entryCount == MAX_ENTRY_COUNT) {
		BSP_LCD_SetFont(&Font8);
		BSP_LCD_DisplayStringAt(1, 231, "Max commands entered!", 0);
		BSP_LCD_SetFont(&Font12);
	} else {
		BSP_LCD_DisplayStringAt(displayEntryX, displayEntryY, textEntry, 0);
		entryCount++;
		displayEntryX += 6;
	}
}
;

void LCDEntry2(uint8_t *textEntry, int length) {

	//BSP_LCD_DisplayStringAt(displayEntryX,displayEntryY,"1",0);
	if ((displayEntryX >= 312)
			&& ((displayEntryY == 4) || (displayEntryY == 15))) {
		BSP_LCD_DisplayStringAt(displayEntryX, displayEntryY, textEntry, 0);
		displayEntryY += 11;
		displayEntryX = 1;
		entryCount = entryCount + (length);
	} else if (entryCount == MAX_ENTRY_COUNT) {
		BSP_LCD_SetFont(&Font8);
		BSP_LCD_DisplayStringAt(1, 231, "Max commands entered!", 0);
	} else {
		BSP_LCD_DisplayStringAt(displayEntryX, displayEntryY, textEntry, 0);
		entryCount = entryCount + length;
		displayEntryX += 6 * (length);
	}
}
;

void CalculatorProcess(void) {
	static uint8_t token[159];//buffer to hold the current token being read (i.e. a digit with any dots), size 159 for max calculator input
	static uint8_t *tokenPtr;//tokenPtr used to allocate memory for the size of the input
	static uint16_t digitFlag = 0;	//flag to determine if a digit was read last
	static uint16_t digitCount = 0;		//count to increment the index of token

	static uint8_t **tokens;				//array of pointers to strings
	static uint16_t tokenCount = 0;			//count of how many tokens are read
	static uint16_t resetFlag = 0;

	static uint16_t shift = 0; //help to switch button display and whether we compute inverse trig functions or not (extensions to fit more functionality in a small screen)
	static uint16_t deg = 1; //flag to help user to use trig functions for degrees or radian answer conversion before computing. (does not apply to hyperbolic functions)
	static uint16_t eq = 0; //equal symbol flag to help clear screen when taking user input after computing an answer

	if (resetFlag == 0) {
		tokens = (uint8_t**) malloc(sizeof(uint8_t*) * 159); //allocate memory on first pass
		memset(token, '\0', sizeof(token));
		resetFlag = 1;
	}

	//
	//	BSP_LCD_DrawVLine (52, 60, 89);
	//	    BSP_LCD_DrawVLine (102, 60, 89);
	//	    BSP_LCD_DrawHLine (2, 105, 150);
	if (BSP_TP_GetDisplayPoint(&display) == 0) { // if touched
		if ((display.x >= 2) && (display.x <= 51) && (display.y >= 60)
				&& (display.y <= 105)) {
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(23, 77, "10cm", 0);
			BSP_LCD_DisplayStringAt(73, 77, "20cm", 0);
			BSP_LCD_DisplayStringAt(123, 77, "30cm", 0);
			BSP_LCD_DisplayStringAt(23, 122, "40cm", 0);
			BSP_LCD_DisplayStringAt(73, 122, "50cm", 0);
			BSP_LCD_DisplayStringAt(123, 122, "60cm", 0);
			HAL_Delay(800);
			while (distSelect() == 1) {
			}
			//			while(BSP_TP_GetDisplayPoint(&display) != 0){}
			//			if ((display.x >= 2) && (display.x <= 51) && (display.y >= 60)&& (display.y <= 105)) stopDist=10;
			//			else if ((display.x >= 52) && (display.x <= 101) && (display.y >= 60)&& (display.y <= 105)) stopDist=20;
			//			else if ((display.x >= 102) && (display.x <= 150) && (display.y >= 60)&& (display.y <= 105)) stopDist=30;
			//			else if ((display.x >= 2) && (display.x <= 51) && (display.y >= 106)&& (display.y <= 149)) stopDist=40;
			//			else if ((display.x >= 52) && (display.x <= 101) && (display.y >= 106)&& (display.y <= 149)) stopDist=50;
			//			else if ((display.x >= 102) && (display.x <= 150)&& (display.y >= 106) && (display.y <= 149)) stopDist=60;
			state = state2;

			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(23, 77, "1   ", 0);
			BSP_LCD_DisplayStringAt(73, 77, "2   ", 0);
			BSP_LCD_DisplayStringAt(23, 122, "4   ", 0);
			BSP_LCD_DisplayStringAt(73, 122, "5   ", 0);
			BSP_LCD_DisplayStringAt(123, 77, "3   ", 0);
			BSP_LCD_DisplayStringAt(123, 122, "6   ", 0);
			return 0;
		} else if ((display.x >= 52) && (display.x <= 101) && (display.y >= 60)
				&& (display.y <= 105)) {
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			BSP_LCD_DisplayStringAt(73, 77, "2", 0);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(23, 77, "1", 0);
			BSP_LCD_DisplayStringAt(23, 122, "4", 0);

			BSP_LCD_DisplayStringAt(73, 122, "5", 0);
			BSP_LCD_DisplayStringAt(123, 77, "3", 0);
			BSP_LCD_DisplayStringAt(123, 122, "6", 0);
			state = state4;
			return 0;
		} else if ((display.x >= 102) && (display.x <= 150) && (display.y >= 60)
				&& (display.y <= 105)) {
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			BSP_LCD_DisplayStringAt(123, 77, "3", 0);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(23, 77, "1", 0);
			BSP_LCD_DisplayStringAt(23, 122, "4", 0);
			BSP_LCD_DisplayStringAt(73, 77, "2", 0);
			BSP_LCD_DisplayStringAt(73, 122, "5", 0);

			BSP_LCD_DisplayStringAt(123, 122, "6", 0);
			state = state5;
			return 0;
		} else if ((display.x >= 2) && (display.x <= 51) && (display.y >= 106)
				&& (display.y <= 149)) {
//			BSP_LCD_SetTextColor(LCD_COLOR_RED);
//			BSP_LCD_DisplayStringAt(23, 122, "4", 0);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
						BSP_LCD_DisplayStringAt(23, 77, "long", 0);
						BSP_LCD_DisplayStringAt(73, 77, "short", 0);
						BSP_LCD_DisplayStringAt(123, 77, "diag", 0);
						BSP_LCD_DisplayStringAt(23, 122, "T4", 0);
						BSP_LCD_DisplayStringAt(73, 122, "T5", 0);
						BSP_LCD_DisplayStringAt(123, 122, "T6", 0);
						HAL_Delay(800);
						while (trackSelect() == 1) {
						}
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(23, 77, "1   ", 0);

			BSP_LCD_DisplayStringAt(73, 77, "2    ", 0);
			BSP_LCD_DisplayStringAt(73, 122, "5 ", 0);
			BSP_LCD_DisplayStringAt(123, 77, "3   ", 0);
			BSP_LCD_DisplayStringAt(23, 122, "4 ", 0);
			BSP_LCD_DisplayStringAt(123, 122, "6 ", 0);
			state = state6;
			return 0;
		} else if ((display.x >= 52) && (display.x <= 101) && (display.y >= 106)
				&& (display.y <= 149)) {
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			BSP_LCD_DisplayStringAt(73, 122, "5", 0);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(23, 77, "1", 0);
			BSP_LCD_DisplayStringAt(23, 122, "4", 0);
			BSP_LCD_DisplayStringAt(73, 77, "2", 0);

			BSP_LCD_DisplayStringAt(123, 77, "3", 0);
			BSP_LCD_DisplayStringAt(123, 122, "6", 0);
		} else if ((display.x >= 102) && (display.x <= 150)
				&& (display.y >= 106) && (display.y <= 149)) {
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			BSP_LCD_DisplayStringAt(123, 122, "6", 0);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(23, 77, "1", 0);
			BSP_LCD_DisplayStringAt(23, 122, "4", 0);
			BSP_LCD_DisplayStringAt(73, 77, "2", 0);
			BSP_LCD_DisplayStringAt(73, 122, "5", 0);
			BSP_LCD_DisplayStringAt(123, 77, "3", 0);

		}
	}
	HAL_Delay(100);
}

int trackSelect() {
	int a = 0;
	if (BSP_TP_GetDisplayPoint(&display) == 0) {
		if ((display.x >= 2) && (display.x <= 51) && (display.y >= 60)
				&& (display.y <= 105)){
			track = TRACKONE;
		trackLight = track*0.45;}
		else if ((display.x >= 52) && (display.x <= 101) && (display.y >= 60)
				&& (display.y <= 105)){
			track = TRACKTWO;
		trackLight = 0.4*track;}
		else if ((display.x >= 102) && (display.x <= 150) && (display.y >= 60)
				&& (display.y <= 105)){
			track = TRACKTHREE;
		trackLight = track*0.5;}
		else if ((display.x >= 2) && (display.x <= 51) && (display.y >= 106)
				&& (display.y <= 149)){
			track = TRACKFOUR;}
		else if ((display.x >= 52) && (display.x <= 101) && (display.y >= 106)
				&& (display.y <= 149)){
			track = TRACKFIVE;}
		else if ((display.x >= 102) && (display.x <= 150) && (display.y >= 106)
				&& (display.y <= 149)){
			track = TRACKSIX;}
	} else {
		a = 1;
	}

	return a;

}

int distSelect(){
	int a = 0;
		if (BSP_TP_GetDisplayPoint(&display) == 0) {
			if ((display.x >= 2) && (display.x <= 51) && (display.y >= 60)
					&& (display.y <= 105))
				stopDist = 20;
			else if ((display.x >= 52) && (display.x <= 101) && (display.y >= 60)
					&& (display.y <= 105))
				stopDist = 25;
			else if ((display.x >= 102) && (display.x <= 150) && (display.y >= 60)
					&& (display.y <= 105))
				stopDist = 30;
			else if ((display.x >= 2) && (display.x <= 51) && (display.y >= 106)
					&& (display.y <= 149))
				stopDist = 40;
			else if ((display.x >= 52) && (display.x <= 101) && (display.y >= 106)
					&& (display.y <= 149))
				stopDist = 50;
			else if ((display.x >= 102) && (display.x <= 150) && (display.y >= 106)
					&& (display.y <= 149))
				stopDist = 60;
		} else {
			a = 1;
		}

		return a;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	char *input=(char*)rx_data;
	light= rx_data[0];
	one= rx_data[1];
	two= rx_data[2];
	//HAL_UART_Transmit(&huart3,(uint8_t) Light, 1, HAL_MAX_DELAY);

	if((one == '0') && (two == '3')){
		//just a test to see how to get the traffic time
		//works
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_SET);
	}else{
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_RESET);
	}

	if( light == 'R' ){
		//printf("%s\n",input);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
		trafficStop=1;

	}
	if(light == 'G' ){
		//printf("%s\n",input);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
		trafficStop=0;
	}
	if(light == 'A' ){
		//printf("%s\n",input);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
		trafficStop=1;
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if(ADC_VALUE>7500){
		batCap= (int)(ADC_VALUE*100/8200);
	}
	else if(ADC_VALUE>6500){
		batCap=(int)((70/1000)*ADC_VALUE-435);
	}
	else{
		batCap=20;
	}
distTravelled= quarterRevs*4.6;
	printf("<Bat. Capacity: %4d %% Speed: %5.3f m/s  US Dist: %4d cm Dist. Travelled: %4.1f cm>\n",
			(int) batCap, speed, distance, distTravelled);
LCD_Print(stateString);
//	printf("<Hello>\n");

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
