/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "cmsis_os.h"
#include "font.h"
#include "lcd_st7565.h"
#include "lcd_st7565_pinconf.h"
#include "buzzer.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* Definitions for displayUpdate */
osThreadId_t displayUpdateHandle;
const osThreadAttr_t displayUpdate_attributes =
{ .name = "displayUpdate", .stack_size = 128 * 4, .priority =
		(osPriority_t) osPriorityNormal, };
/* Definitions for readDistance */
osThreadId_t readDistanceHandle;
const osThreadAttr_t readDistance_attributes =
{ .name = "readDistance", .stack_size = 128 * 4, .priority =
		(osPriority_t) osPriorityHigh, };
/* Definitions for rotateRadarTask */
osThreadId_t rotateRadarTaskHandle;
const osThreadAttr_t rotateRadarTask_attributes =
{ .name = "rotateRadarTask", .stack_size = 128 * 4, .priority =
		(osPriority_t) osPriorityNormal, };
/* Definitions for setRadarPos */
osThreadId_t setRadarPosHandle;
const osThreadAttr_t setRadarPos_attributes =
{ .name = "setRadarPos", .stack_size = 128 * 4, .priority =
		(osPriority_t) osPriorityNormal, };
/* Definitions for isManualRadarRotation */
osEventFlagsId_t isManualRadarRotationHandle;
const osEventFlagsAttr_t isManualRadarRotation_attributes =
{ .name = "isManualRadarRotation" };
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC_Init(void);
void displayUpdate_function(void *argument);
void readDistance_function(void *argument);
void rotateRadar_function(void *argument);
void setRadarPosition_function(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int adc_local = 0;

void delay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < time)
		;
}

static uint32_t IC_Val1 = 0;
static uint32_t IC_Val2 = 0;
static uint32_t Difference = 0;
static uint8_t Is_First_Captured = 0;
static uint8_t Distance = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{ // if the first value is not captured
		if (Is_First_Captured == 0)
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
			Is_First_Captured = 1; // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (Is_First_Captured == 1) // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read second value
			__HAL_TIM_SET_COUNTER(htim, 0); // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2 - IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034 / 2;
			Is_First_Captured = 0; // set it back to false

			// set the polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC4);
		}
	}
}

void HCSR04_Read(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	delay(10); // wait for 10us
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC4);
}

#define PI 3.14159265358979323846

// Zone 1 is between 0 - 60 degrees
static uint8_t isZone1 = 0;
// Zone 2 is between 60 - 120 degrees
static uint8_t isZone2 = 0;
// Zone 3 is between 120 - 180 degrees
static uint8_t isZone3 = 0;

void drawRadar()
{
	// Draw the linear line at the bottom
	st7565_drawline(buffer, 0, 50, 128, 50, 1);

	uint8_t radarDegrees = isZone1 ? 60 : isZone2 ? 120 : 180;
	uint8_t radarStartingDegrees = isZone1 ? 0 : isZone2 ? 60 :
									isZone3 ? 120 : 0;

	// Draw the radar arc lines for distances in cm
	float distances[] =
	{ 15, 30, 45 }; // Distances for the arcs (in cm)
	for (int d = 0; d < sizeof(distances) / sizeof(distances[0]); d++)
	{
		float radius = distances[d]; // Get radius for current distance
		for (int i = 0; i < 180; i++)
		{
			// Calculate the x and y coordinates with the specified radius
			float x = 64 - (radius * cos(i * PI / 180));
			float y = 50 - (radius * sin(i * PI / 180));

			// Draw the points for the arc only for the degrees between radarStartingDegrees and radarDegrees, both included.
			if (i >= radarStartingDegrees && i <= radarDegrees)
			{
				st7565_setpixel(buffer, (uint8_t) x, (uint8_t) y, 1);
			}
			else
			{
				st7565_setpixel(buffer, (uint8_t) x, (uint8_t) y, 0);
			}
		}
	}

	// Draw the radar degrees lines.
	for (int i = 30; i <= 150; i += 30)
	{
		// Calculate the x and y coordinates with a larger radius
		float radius = 55;
		float x = 64 - (radius * cos(i * PI / 180));
		float y = 60 - (radius * sin(i * PI / 180));

		// Draw the line for the degrees between radarStartingDegrees and radarDegrees, both included.
		if (i <= radarDegrees && i >= radarStartingDegrees)
		{
			st7565_drawline(buffer, 64, 50, (uint8_t) x, (uint8_t) y, 1);
		}
		else
		{
			st7565_drawline(buffer, 64, 50, (uint8_t) x, (uint8_t) y, 0);
		}
	}

	// TODO Extract the below code into a separate method, something like drawArcDistancesAndLineDegrees()
	// Draw distances in cm for each arc line.
	st7565_drawstring(buffer, 70, 7, "10");
	st7565_drawstring(buffer, 90, 7, "20");
	st7565_drawstring(buffer, 105, 7, "30");

	// Draw angles for each line based on the selected zone.
	if (isZone1)
	{
		st7565_drawstring(buffer, 6, 4, "30");
		st7565_drawstring(buffer, 20, 1, "60");
	}
	else if (isZone2)
	{
		st7565_drawstring(buffer, 20, 1, "60");
		st7565_drawstring(buffer, 58, 0, "90");
		st7565_drawstring(buffer, 95, 1, "120");
	}
	else if (isZone3)
	{
		st7565_drawstring(buffer, 95, 1, "120");
		st7565_drawstring(buffer, 107, 4, "150");
	}
	// All zones
	else
	{
		st7565_drawstring(buffer, 6, 4, "30");
		st7565_drawstring(buffer, 20, 1, "60");
		st7565_drawstring(buffer, 58, 0, "90");
		st7565_drawstring(buffer, 95, 1, "120");
		st7565_drawstring(buffer, 107, 4, "150");
	}

	// Write buffer to display
	st7565_write_buffer(buffer);
}

// Function to draw just the radar line with proper clearing of the previous line
// This function clears the previous radar line before drawing the new line to avoid
// leaving artifacts on the display. It addresses a bug where the radar line wasn't
// properly cleared before redrawing, causing remnants of the previous line to remain
// visible on the display and potentially causing the display to go blank after a few
// seconds due to accumulation of artifacts.
void drawRadarLine(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t x1,
		uint8_t y1, uint8_t color)
{
	// Clear the previous radar line
	st7565_drawline(buff, x0, y0, x1, y1, 0);

	// Draw the new radar line
	st7565_drawline(buff, x0, y0, x1, y1, color);
}

// TODO Review implementation, is working very slow right now and the point is not properly placed.
void drawDetectionPoint(uint16_t servoAngle, uint8_t distance)
{
	// Check if the detected distance is below 30 cm
	if (distance < 30)
	{
		// Calculate the position of the detection point on the radar
		// Adjust the calculation based on the changes made to the radar drawing logic
		float radius = 50; // Assuming radius of 50 pixels
		float radarX = 64 - (radius * cos(servoAngle * PI / 180));
		float radarY = 64 - (radius * sin(servoAngle * PI / 180)) - distance; // Adjust for distance from radar line

		// Draw the detection point on the radar
		st7565_setpixel(buffer, (uint8_t) radarX, (uint8_t) radarY, 1);
	}
}

void updateRadarDisplay(uint16_t servoAngle)
{
	// Static variables to store previous radar line position
	static uint8_t previousRadarX = 0;
	static uint8_t previousRadarY = 0;

	// Calculate radar line position based on servo angle
	// You need to adjust these calculations based on your radar display setup
	float radarX = 64 - (50 * cos(servoAngle * PI / 180));
	float radarY = 60 - (50 * sin(servoAngle * PI / 180));

	// Clear the old radar line and draw the new one
	drawRadarLine(buffer, 64, 50, previousRadarX, previousRadarY, 0);
	drawRadarLine(buffer, 64, 50, radarX, radarY, 1);

	// Store current radar line position for clearing in the next iteration
	previousRadarX = (uint8_t) radarX;
	previousRadarY = (uint8_t) radarY;

	// Draw detection point if distance is below 30 cm
//	drawDetectionPoint(servoAngle, Distance);
}

// Distance is represented in cm. If there is no object detected, distance is going to be 0.
void drawDistance()
{
	char distanceStr[20]; // Assuming a maximum length of 20 characters for the string

	// Convert Distance to a string
	sprintf(distanceStr, "Dist:%dcm", Distance);

	// Draw the distance on the screen
	st7565_drawstring(buffer, 1, 7, (uint8_t*) distanceStr);

	// Update the LCD display
	st7565_write_buffer(buffer);
}

// Calculate the angle from servo position in order to set the radar line position properly.
uint16_t calculateAngle(uint16_t servoPosition, uint16_t minPWM,
		uint16_t maxPWM)
{
	// Map the servo position to the corresponding angle using linear interpolation
	uint16_t angle = (servoPosition - minPWM) * 180 / (maxPWM - minPWM);

	return angle;
}

// When the radar is stationary in one position, we must alert using the buzzer is there are any objects detected under 30 cm.
void distanceBuzzAlert()
{
	// Stop any previous buzzer sound
	BUZZER_Off();

	// Start again the buzzer
	BUZZER_Init();

	// Control the buzzer based on distance
	if (Distance < 10)
	{
		// Close distance, long beeps
		BUZZER_Go(TBUZ_200, TICK_8);
	}
	else if (Distance < 20)
	{
		// Medium distance, short beeps
		BUZZER_Go(TBUZ_50, TICK_2);
		osDelay(500); // Delay between beeps
		BUZZER_Go(TBUZ_50, TICK_2);
	}
	else if (Distance < 30)
	{
		// Far distance, short beeps
		BUZZER_Go(TBUZ_50, TICK_2);
		osDelay(1000); // Delay between beeps
		BUZZER_Go(TBUZ_50, TICK_2);
	}
}

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
	MX_TIM2_Init();
	MX_SPI1_Init();
	MX_TIM14_Init();
	MX_TIM3_Init();
	MX_ADC_Init();
	st7565_init();
	st7565_backlight_enable();
	/* USER CODE BEGIN 2 */

	// == ULTRASONIC ===
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	drawRadar();

	// LCD Shield have digital pins that are connected to the board pins, I added a wire between those 2 pins on the shield, that represents PB3 and PA10 on the board.
	// We set one pin high and because the wire connects them, the other is set to high, so we can check if it was set to high. If it doesn't, then it means that either the wire is not
	// connected or the LCD Shield is not connected to the board.
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	uint8_t isLCDConnected = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);

	if (isLCDConnected)
	{
		BUZZER_Init();
		BUZZER_Go(TBUZ_100, TICK_4);
	}

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of displayUpdate */
	displayUpdateHandle = osThreadNew(displayUpdate_function, NULL,
			&displayUpdate_attributes);

	/* creation of readDistance */
	readDistanceHandle = osThreadNew(readDistance_function, NULL,
			&readDistance_attributes);

	/* creation of rotateRadarTask */
	rotateRadarTaskHandle = osThreadNew(rotateRadar_function, NULL,
			&rotateRadarTask_attributes);

	/* creation of setRadarPos */
	setRadarPosHandle = osThreadNew(setRadarPosition_function, NULL,
			&setRadarPos_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the event(s) */
	/* creation of isManualRadarRotation */
	isManualRadarRotationHandle = osEventFlagsNew(
			&isManualRadarRotation_attributes);

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void)
{

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_10B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_IC_InitTypeDef sConfigIC =
	{ 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 48 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xffff - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 48 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 10000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void)
{

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 24 - 1;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 1000 - 1;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */
	HAL_TIM_MspPostInit(&htim14);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, BUZZER_Pin | SPICD_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | LCDOUT_Pin | SPICS_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, BL_Pin | SPIRST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BUZZER_Pin */
	GPIO_InitStruct.Pin = BUZZER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
	GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 LCDOUT_Pin SPICS_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | LCDOUT_Pin | SPICS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : SPICD_Pin */
	GPIO_InitStruct.Pin = SPICD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPICD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BL_Pin SPIRST_Pin */
	GPIO_InitStruct.Pin = BL_Pin | SPIRST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LCDINP_Pin */
	GPIO_InitStruct.Pin = LCDINP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LCDINP_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*** SYSTICK (T = 1ms) ***/
/**
 * @brief  SYSTICK callback.
 * @retval None
 */
void HAL_SYSTICK_Callback(void)
{
	BUZZER_Handler();
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_displayUpdate_function */
/**
 * @brief  Function implementing the displayUpdate thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_displayUpdate_function */
void displayUpdate_function(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;)
	{
		// === LCD SCREEN ===

		drawDistance();

		// TODO This should be removed from here, it afects the performance of the servomotor (it moves more slowly because of this) and also it's not needed, because we need to draw the radar only once on the screen.
		// It's keept here for the moment because the draw radar line is removing the radar arcs needed for splitting the radar into the 6 zones.
		// An implementation must be done where we check if draw line overlaps with a radar arc and we will not draw that line at that point.
		// Or maybe there are better solutions. This is what came in my mind without thinking too much at the problem at this moment of writting this. Further research is required.
		drawRadar();

		osDelay(250);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_readDistance_function */
/**
 * @brief Function implementing the readDistance thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_readDistance_function */
void readDistance_function(void *argument)
{
	/* USER CODE BEGIN readDistance_function */
	/* Infinite loop */
	for (;;)
	{
		// === HCSR04 SENSOR ===
		HCSR04_Read();

		osDelay(20);
	}
	/* USER CODE END readDistance_function */
}

/* USER CODE BEGIN Header_rotateRadar_function */
/**
 * @brief Function implementing the rotateRadarTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_rotateRadar_function */
void rotateRadar_function(void *argument)
{
	/* USER CODE BEGIN rotateRadar_function */
	/* Infinite loop */
	for (;;)
	{
		// Rotate radar auto only if the user haven't toggled the manual rotation by using the joystick
		if (osEventFlagsGet(isManualRadarRotationHandle) == 1)
		{
		}
		else
		{
			// === SERVOMOTOR ===

			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

			// Set the target position (180 degrees)
			uint16_t targetPosition = 2500;

			// Set the initial position (0 degrees)
			uint16_t initialPosition = 625;

			// Define the number of steps for the transition
			uint16_t steps = 200;		// Increase the number of steps further

			// Calculate the step size
			uint16_t stepSize = (targetPosition - initialPosition) / steps;

			// Gradually change the PWM signal to move the servo
			for (int i = 0;
					i < steps
							&& osEventFlagsGet(isManualRadarRotationHandle) != 1;
					i++)
			{
				// Incrementally increase the PWM signal
				uint16_t newPosition = initialPosition + i * stepSize;
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, newPosition);
				osDelay(100);// Adjust the delay time between steps as necessary

				// Calculate the angle from the servo position
				uint16_t servoAngle = calculateAngle(newPosition,
						initialPosition, targetPosition);

				// Update radar display based on servo angle
				updateRadarDisplay(servoAngle);
			}

			// Wait for the servo to reach the target position
			osDelay(2000);		// Increased wait time for slower movement

			// Define the number of steps for the backward transition
			uint16_t steps_back = 200;// Increase the number of steps for backward transition

			// Calculate the step size for the backward transition
			uint16_t stepSize_back = (initialPosition - targetPosition)
					/ steps_back;

			// Gradually change the PWM signal to move the servo back to the initial position
			for (int i = 0;
					i < steps_back
							&& osEventFlagsGet(isManualRadarRotationHandle) != 1;
					i++)
			{
				// Decrementally decrease the PWM signal
				uint16_t newPosition_back = targetPosition + i * stepSize_back;
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, newPosition_back);
				osDelay(100);// Adjust the delay time between steps as necessary

				// Calculate the angle from the servo position
				uint16_t servoAngle = calculateAngle(newPosition_back,
						initialPosition, targetPosition);

				// Update radar display based on servo angle
				updateRadarDisplay(servoAngle);
			}

			// Wait for the servo to reach the initial position
			osDelay(2000);		// Increased wait time for slower movement
		}

	}
	/* USER CODE END rotateRadar_function */
}

/* USER CODE BEGIN Header_setRadarPosition_function */
/**
 * @brief Function implementing the setRadarPos thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_setRadarPosition_function */
void setRadarPosition_function(void *argument)
{
	/* USER CODE BEGIN setRadarPosition_function */
	/* Infinite loop */
	for (;;)
	{

		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, 100);
		if (osEventFlagsGet(isManualRadarRotationHandle) == 1)
		{
			distanceBuzzAlert();

			adc_local = HAL_ADC_GetValue(&hadc);

			if (adc_local < 500 && adc_local > 200)
			{
				osEventFlagsClear(isManualRadarRotationHandle, 1);

				// Reset zones
				isZone1 = 0;
				isZone2 = 0;
				isZone3 = 0;
			}
		}
		// TODO Add some proper values based on the right angles to place the position (radar line) in all 3 possible zones.
		// There are 3 possible joystick combination that's why, so each manual area must cover 2 radar zones (there are 6 in total)
		else
		{
			HAL_ADC_Start(&hadc);
			HAL_ADC_PollForConversion(&hadc, 100);
			adc_local = HAL_ADC_GetValue(&hadc);

			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

			// JOYSTICK UP - This doesn't work because it have the same adc_valuevalue as not pressing any button.
			if (adc_local > 1000 && adc_local < 1200)
			{

			}
			// JOYSTICK RIGHT
			else if (adc_local > 900 && adc_local < 999)
			{
				osEventFlagsSet(isManualRadarRotationHandle, 1);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 650);
				isZone3 = 1;
			}
			// JOYSTICK BOTTOM
			else if (adc_local > 550 && adc_local < 700)
			{
				osEventFlagsSet(isManualRadarRotationHandle, 1);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1250);
				isZone2 = 1;
			}
			// JOYSTICK PRESSED -> Used for toggling the auto mode
			else if (adc_local < 500 && adc_local > 200)
			{
			}
			// JOYSTICK LEFT
			else if (adc_local < 200)
			{
				osEventFlagsSet(isManualRadarRotationHandle, 1);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1850);
				isZone1 = 1;
			}
			else
			{

			}
		}
		osDelay(1);
	}
	/* USER CODE END setRadarPosition_function */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
