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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AuvControl.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDRESS (0x68<<1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

int __io_putchar(int ch){
	ITM_SendChar(ch);
	return ch;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;
osThreadId ReadInputHandle;
osThreadId ProcInputDataHandle;
osThreadId MapOutputHandle;
osThreadId ControllerHandle;
osThreadId StateControllerHandle;
osThreadId ExternalCommuniHandle;
osMutexId uartMutexHandle;
osSemaphoreId inputsReaded_S_Handle;
osSemaphoreId inputsCalculated_S_Handle;
osSemaphoreId controlDone_S_Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void readInputFcn(void const * argument);
void processInputDataFcn(void const * argument);
void MapOutputFunction(void const * argument);
void ControllerFcn(void const * argument);
void StateControlFcn(void const * argument);
void ExternalCommunicationFcn(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Global Variables */
// Controling constants
#define Ts 0.05 // sampling time [s]
#define TaskSamplingTime_ms 50
#define StateControllerSamplingTime_ms 200
// Physical AUV constants
#define PI 3.141592f
#define g 9.81f
#define waterDensity  998.2f
#define hydrofoilFrontArea 1 // [m^2] forntal area of each hydro foil
#define d1_stere 0.19 // [m] distance between center of hydro foil
#define d2_stere 0.4 // [m] distance between center of hydro foil


// Raw data from IMU
int16_t accellRaw[3];
int16_t gyroRaw[3];

// Local Body Frame
float accellLocal[3]; // Accelerations in Local Frame
float gyroLocal[3]; // Angular Velocity in Local Frame
float gyroBias[3]; // Measured Bias for Gyroscope
float orientationAccelLocal[3]; // Global orientation angle based on accelerations
float velocityXLocal=0.0f; //0.6f; // Linear velocity in local frame, axis X // Fixed value

// Global Body Frame
float accellGlobal[3];
float gyroGlobal[3]; // Angular Velocity in Global Frame
float orientationGlobal[3]={0.0f}; // Determined Orientation
float angularVelocityGlobal[3]; // Rate of changes of determined orientation
float positionGlobal[3]={0.0f};

// Control variables
float orientationSetpoint[3]; // Setpoints of orientation angles
float servoAngleRequest[2]; // Requested angle of servos
float motorPower_percent;
bool initRequest,NavigInitDone=false; // Initialization

regulator_PID rollController; // regulator instance
regulator_PID directionController; // regulator instance

bool RollControllerActive;
bool PitchControllerActive;
bool YawControllerActive;
// Recived from ESP8266:

bool NewTask;
Task TaskTarget; // What should be regulated

float TaskTargetValue=0.0f; // Regulated Value
float TaskTargetOffset=0.1; // accetable offset
float DerivativeTargetOffset=0.1;
// diagnostics variables
HAL_StatusTypeDef mpuInitStatus,mpuCommStatus,uartStatus; // communication diagnostic
TickType_t cycleStart,cycleDuration; // cycle duration diagnostic

uint8_t buffESP[]="6969";
// debugging variables:
bool taskExec;
float test[3];

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  mpuInitStatus = mpu6050_init(&hi2c3,MPU6050_ADDRESS);
  initRequest = true;
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of uartMutex */
  osMutexDef(uartMutex);
  uartMutexHandle = osMutexCreate(osMutex(uartMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of inputsReaded_S_ */
  osSemaphoreDef(inputsReaded_S_);
  inputsReaded_S_Handle = osSemaphoreCreate(osSemaphore(inputsReaded_S_), 1);

  /* definition and creation of inputsCalculated_S_ */
  osSemaphoreDef(inputsCalculated_S_);
  inputsCalculated_S_Handle = osSemaphoreCreate(osSemaphore(inputsCalculated_S_), 1);

  /* definition and creation of controlDone_S_ */
  osSemaphoreDef(controlDone_S_);
  controlDone_S_Handle = osSemaphoreCreate(osSemaphore(controlDone_S_), 1);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ReadInput */
  osThreadDef(ReadInput, readInputFcn, osPriorityRealtime, 0, 128);
  ReadInputHandle = osThreadCreate(osThread(ReadInput), NULL);

  /* definition and creation of ProcInputData */
  osThreadDef(ProcInputData, processInputDataFcn, osPriorityHigh, 0, 512);
  ProcInputDataHandle = osThreadCreate(osThread(ProcInputData), NULL);

  /* definition and creation of MapOutput */
  osThreadDef(MapOutput, MapOutputFunction, osPriorityNormal, 0, 256);
  MapOutputHandle = osThreadCreate(osThread(MapOutput), NULL);

  /* definition and creation of Controller */
  osThreadDef(Controller, ControllerFcn, osPriorityAboveNormal, 0, 256);
  ControllerHandle = osThreadCreate(osThread(Controller), NULL);

  /* definition and creation of StateController */
  osThreadDef(StateController, StateControlFcn, osPriorityHigh, 0, 128);
  StateControllerHandle = osThreadCreate(osThread(StateController), NULL);

  /* definition and creation of ExternalCommuni */
  osThreadDef(ExternalCommuni, ExternalCommunicationFcn, osPriorityAboveNormal, 0, 256);
  ExternalCommuniHandle = osThreadCreate(osThread(ExternalCommuni), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 19;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 3749;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_BOARD_Pin|SPI_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_BOARD_Pin */
  GPIO_InitStruct.Pin = BUTTON_BOARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_BOARD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_BOARD_Pin SPI_SS_Pin */
  GPIO_InitStruct.Pin = LED_BOARD_Pin|SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	float gyroBiasSum[3]={0.0f};
	const TickType_t initTime = pdMS_TO_TICKS(3000); // 3 seconds
	TickType_t pressStartTime = 0;
	int n=0;

  /* Infinite loop */
  for(;;)
  {

	  if (HAL_GPIO_ReadPin(GPIOC, BUTTON_BOARD_Pin)== GPIO_PIN_RESET){
		  initRequest = true;
	  }


	  if (initRequest == true){
	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  	if (pressStartTime == 0) {
	  		// Start the timer
	  		pressStartTime = xTaskGetTickCount();
	  		gyroBiasSum[0]=0.0f;
	  		gyroBiasSum[1]=0.0f;
	  		gyroBiasSum[2]=0.0f;
	  		n=0;
	  		}
	  	else if((xTaskGetTickCount() - pressStartTime) < initTime){

	  		for (int i=0;i<=2;i++){
	  			gyroBiasSum[i]+=gyroLocal[i];
	  			}
	  		n++;
	  		vTaskDelay(50);
	  		}
	  	else{

	  		for (int i=0;i<=2;i++){
	  			gyroBias[i]= (gyroBiasSum[i]/((float)n));
	  			 }
	  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	  		pressStartTime = 0;
	  		NavigInitDone = true;
	  		initRequest = false;
	  		}
	  	}
  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_readInputFcn */
/**
* @brief Function implementing the ReadInput thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readInputFcn */
void readInputFcn(void const * argument)
{
  /* USER CODE BEGIN readInputFcn */
  /* Infinite loop */
  for(;;)
  {
	cycleStart = xTaskGetTickCount(); // start counting time of 1 cycle duration
	mpuCommStatus = mpu6050_update(&hi2c3,MPU6050_ADDRESS,accellRaw,gyroRaw); // read data from MPU

	xSemaphoreGive(inputsReaded_S_Handle);
	vTaskDelay(pdMS_TO_TICKS(TaskSamplingTime_ms));
  }
  /* USER CODE END readInputFcn */
}

/* USER CODE BEGIN Header_processInputDataFcn */
/**
* @brief Function implementing the ProcInputData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_processInputDataFcn */
void processInputDataFcn(void const * argument)
{
  /* USER CODE BEGIN processInputDataFcn */
  /* Infinite loop */
  for(;;)
  {
	  if(xSemaphoreTake(inputsReaded_S_Handle,portMAX_DELAY)==pdTRUE){

	  // Scale data and calculate angle based on acell
	  accelCalc(accellRaw, accellLocal , orientationAccelLocal);
	  gyroCalc(gyroRaw,gyroLocal);
	  // transform acceleration an angular velocity to global frame
	  vectTransform(orientationGlobal,gyroLocal,gyroBias,gyroGlobal,accellLocal,accellGlobal);
	  // filtering
	  yawEvaluate(orientationGlobal, gyroGlobal[2] ,NavigInitDone);
	  kalmanFilter(orientationAccelLocal, gyroGlobal, orientationGlobal, accellLocal, gyroBias, NavigInitDone);

	 // if initialization procces active -> finish
	  if (NavigInitDone == true)NavigInitDone=false;
	  xSemaphoreGive(inputsCalculated_S_Handle);
	  }
  }
  /* USER CODE END processInputDataFcn */
}

/* USER CODE BEGIN Header_MapOutputFunction */
/**
* @brief Function implementing the MapOutput thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MapOutputFunction */
void MapOutputFunction(void const * argument)
{
  /* USER CODE BEGIN MapOutputFunction */
	servoInit(&htim3);
	motorInit(&htim1);
  /* Infinite loop */
  for(;;)
  {
	 if(xSemaphoreTake(controlDone_S_Handle,portMAX_DELAY)==pdTRUE){

		servoSet(servoAngleRequest,&htim3);
		motorSet(motorPower_percent,&htim1);
		cycleDuration = (xTaskGetTickCount()- cycleStart); // calculate duration time [ms]

	 }
  }
  /* USER CODE END MapOutputFunction */
}

/* USER CODE BEGIN Header_ControllerFcn */
/**
* @brief Function implementing the Controller thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControllerFcn */
void ControllerFcn(void const * argument)
{
  /* USER CODE BEGIN ControllerFcn */
	float orientationGlobalPrev[3]={0.0f};

	// Set initial parameters for regulator
		rollController.Kp=1.368792;
		rollController.Ki=0.802290;
		rollController.Kd=0.448888;
		rollController.Nd=13.235897;
		rollController.outMax = 30;
		rollController.outMin = 30;
		directionController.Kp=1.368792;
		directionController.Ki=0.802290;
		directionController.Kd=0.448888;
		directionController.Nd=13.235897;
		directionController.outMax = 30;
		directionController.outMin = 30;

  /* Infinite loop */
  for(;;)
  {
	  if(xSemaphoreTake(inputsCalculated_S_Handle,portMAX_DELAY)==pdTRUE){

		  if ( RollControllerActive == true ){
			  servoAngleRequest[0]=PIDcontroller(&rollController,orientationGlobal[0],orientationSetpoint[0],NavigInitDone);
			  servoAngleRequest[1]=-servoAngleRequest[0];
		  }
		  else if ( PitchControllerActive == true){

			  servoAngleRequest[0]=PIDcontroller(&directionController,orientationGlobal[1],orientationSetpoint[1],NavigInitDone);
			  servoAngleRequest[1]=servoAngleRequest[0];
		  }
		  else if ( YawControllerActive == true){
			  servoAngleRequest[0]=PIDcontroller(&directionController,orientationGlobal[2],orientationSetpoint[2],NavigInitDone);
			  servoAngleRequest[1]=servoAngleRequest[0];
		  }

		  // Calculate Rate of changes of orientation
		  for (int i=0; i< 3 ; i++){
			  angularVelocityGlobal[i]= (orientationGlobal[i]-orientationGlobalPrev[i])/Ts;
			  orientationGlobalPrev[i]= orientationGlobal[i];
		  }
		 xSemaphoreGive(controlDone_S_Handle);
	  }
  }
  /* USER CODE END ControllerFcn */
}

/* USER CODE BEGIN Header_StateControlFcn */
/**
* @brief Function implementing the StateController thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StateControlFcn */
void StateControlFcn(void const * argument)
{
  /* USER CODE BEGIN StateControlFcn */
	typedef enum { // What should be regulated
	    seting_propper_roll = 0,
	    changing_regulated_angle = 1,
	    returning_to_default_pos = 2,
		regulatation_done = 3,
	}State;

	State stateCounter=seting_propper_roll;
  /* Infinite loop */
  for(;;)
  {
// isSet(float input, float derivative , float setpoint , float input_offset , float derivative_offset )
	  if (NewTask){
		  stateCounter = 0;
	  }

	  switch (TaskTarget){
	  case Roll:
		  RollControllerActive=true;
		  PitchControllerActive=false;
		  YawControllerActive=false;
		  if (isSet(orientationGlobal[0],angularVelocityGlobal[0],orientationSetpoint[0],TaskTargetOffset,DerivativeTargetOffset)){
			  stateCounter = regulatation_done;
		  }
		  break;

	  case Yaw:
		  switch (stateCounter){
		  	  case seting_propper_roll:
		  		  RollControllerActive=true;
		  		  PitchControllerActive=false;
		  		  YawControllerActive=false;
		  		  orientationSetpoint[0]=90.0;
		  		  if (isSet(orientationGlobal[0],angularVelocityGlobal[0],orientationSetpoint[0],TaskTargetOffset,DerivativeTargetOffset)){
			  				  stateCounter = changing_regulated_angle;
		  		  }
			  	  break;
		  	  case changing_regulated_angle:
		  		  RollControllerActive=false;
		  		  YawControllerActive=true;
		  		  orientationSetpoint[2]=TaskTargetValue;
		  		  if (isSet(orientationGlobal[2],angularVelocityGlobal[2],orientationSetpoint[2],TaskTargetOffset,DerivativeTargetOffset)){
		  			  stateCounter = returning_to_default_pos;
		  		  }
		  		  break;
		  	  case returning_to_default_pos:
		  		  RollControllerActive=true;
		  		  YawControllerActive=false;
		  		  orientationSetpoint[0]=0.0f;
		  		  if (isSet(orientationGlobal[0],angularVelocityGlobal[0],orientationSetpoint[0],TaskTargetOffset,DerivativeTargetOffset)){
		  			  stateCounter = regulatation_done;
		  		  }
		  		  break;
		  	case regulatation_done:

		    break;
		  }

		  break;
	  case Pitch:
		  switch (stateCounter){
		  		  case seting_propper_roll:
		  			  RollControllerActive=true;
		  			  PitchControllerActive=false;
		  			  YawControllerActive=false;
		  			  orientationSetpoint[0]=0.0;
		  			  if (isSet(orientationGlobal[0],angularVelocityGlobal[0],orientationSetpoint[0],TaskTargetOffset,DerivativeTargetOffset)){
		  			  	  stateCounter = changing_regulated_angle;
		  			  }
		  			  break;
		  		  case changing_regulated_angle:
		  			  RollControllerActive=false;
		  			  PitchControllerActive=true;
		  			  orientationSetpoint[2]=TaskTargetValue;
		  			  if (isSet(orientationGlobal[2],angularVelocityGlobal[2],orientationSetpoint[2],TaskTargetOffset,DerivativeTargetOffset)){
		  				  stateCounter = returning_to_default_pos;
		  			  }
		  			  break;
		  		  case returning_to_default_pos:
		  		  	 RollControllerActive=true;
		  		  	 PitchControllerActive=false;
		  		  	 orientationSetpoint[0]=0.0f;
		  		  	 if (isSet(orientationGlobal[0],angularVelocityGlobal[0],orientationSetpoint[0],TaskTargetOffset,DerivativeTargetOffset)){
		  		  		stateCounter = regulatation_done;
		  		  	}
		  		  	break;
		  		  case regulatation_done:

		  		  break;
		  		  }
		  break;

	  }




	  vTaskDelay(StateControllerSamplingTime_ms);
  }
  /* USER CODE END StateControlFcn */
}

/* USER CODE BEGIN Header_ExternalCommunicationFcn */
/**
* @brief Function implementing the ExternalCommuni thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ExternalCommunicationFcn */
void ExternalCommunicationFcn(void const * argument)
{
  /* USER CODE BEGIN ExternalCommunicationFcn */
  /* Infinite loop */
	/*
	// TO DO:
	1) communication Init, control
	2) To get:
	TaskTarget,TaskTargetValue, TaskTargetOffset
	Regulator Parameters
	Init
	Power Motor

	3) to calculate and set
	NewTask
	Regulator Parameters

	4) to send
	Orientation
	Status
	Setpoints

	2) Get command: orientation setpoint, motor power,
	3) Get data: Max, min angle; regulator parameters
	4) Variable: New Task
	5) send: orientation, velocity no!
	*/
  for(;;)
  {



	uartStatus=HAL_UART_Transmit(&huart1, buffESP, strlen((char*)buffESP), 500);
    vTaskDelay(250);
  }
  /* USER CODE END ExternalCommunicationFcn */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
