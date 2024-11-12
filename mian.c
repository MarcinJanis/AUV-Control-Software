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

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;
osThreadId ReadInputHandle;
osThreadId ProcInputDataHandle;
osThreadId MapOutputHandle;
osThreadId ControllerHandle;
osThreadId StateControllerHandle;
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
void StartDefaultTask(void const * argument);
void readInputFcn(void const * argument);
void processInputDataFcn(void const * argument);
void MapOutputFunction(void const * argument);
void ControllerFcn(void const * argument);
void StartTask06(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Global Variables */
// Controling constants
#define Ts 0.05 // sampling time [s]
#define TaskSamplingTime_ms 50

// Physical AUV constants
#define PI 3.141592f
#define g 9.81f
#define waterDensity  998.2f
#define hydrofoilFrontArea 1 // [m^2] forntal area of each hydro foil
#define d1_stere 0.19 // [m] distance between center of hydro foil


// Raw data from IMU
int16_t accellRaw[3];
int16_t gyroRaw[3];

// Local Body Frame
float accellLocal[3]; // Accelerations in Local Frame
float gyroLocal[3]; // Angular Velocity in Local Frame
float gyroBias[3]; // Measured Bias for Gyroscope
float orientationAccelLocal[3]; // Global orientation angle based on accelerations
float velocityXLocal=0.6f; // Linear velocity in local frame, axis X // Fixed value

// Global Body Frame
float accellGlobal[3];
float gyroGlobal[3]; // Angular Velocity in Global Frame
float orientationGlobal[3]={0.0f}; // Determined Orientation
float angularVelocityGlobal[3]; // Rate of changes of determined orientation
float positionGlobal[3]={0.0f};

// Control variables
float orientationSetpoint[3]; //
float stereRequest[2];
bool initRequest,NavigInitDone=false; // Initialization

bool RollControlerActive;
bool PitchControlerActive;
bool YawControlerActive;


// diagnostics variables
HAL_StatusTypeDef mpuInitStatus,mpuCommStatus,uartStatus; // communication diagnostic
TickType_t cycleStart,cycleDuration; // cycle duration diagnostic

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
  osThreadDef(StateController, StartTask06, osPriorityNormal, 0, 128);
  StateControllerHandle = osThreadCreate(osThread(StateController), NULL);

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
  RCC_OscInitStruct.PLL.PLLN = 70;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim3.Init.Period = 65535;
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
	  //osSemaphoreId inputsReaded_S_Handle;
	  //osSemaphoreId inputsCaluclated_S_Handle;
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
	  //outputMapFcn = true;
	  // Scale data and calculate angle based on acell
	  accelCalc(accellRaw, accellLocal , orientationAccelLocal);
	  gyroCalc(gyroRaw,gyroLocal);
	  // transform acceleration an angular velocity to global frame
	  vectTransform(orientationGlobal,gyroLocal,gyroBias,gyroGlobal,accellLocal,accellGlobal);
	  // filtering
	  yawEvaluate(orientationGlobal, gyroGlobal[2] ,NavigInitDone);
	  kalmanFilter(orientationAccelLocal, gyroGlobal, orientationGlobal, accellLocal, gyroBias, NavigInitDone);

	 //positionEvaluate(positionGlobal,accellGlobal,NavigInitDone);
	 velocityLocal(&velocityXLocal,accellLocal,orientationGlobal,NavigInitDone);

	 // if initialization procces active -> finish
	  if (NavigInitDone == true)NavigInitDone=false;
	  //outputMapFcn = false;
	  xSemaphoreGive(inputsCalculated_S_Handle);
	  //vTaskDelay(pdMS_TO_TICKS(TaskSamplingTime_ms));
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

	// Start Servo PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* Infinite loop */
  for(;;)
  {
	 if(xSemaphoreTake(controlDone_S_Handle,portMAX_DELAY)==pdTRUE){
		 char buff[50];
		 printf("RawData, %d, %d, %d, %d, %d, %d\n" ,accellRaw[0],accellRaw[1],accellRaw[2],gyroRaw[0],gyroRaw[1],gyroRaw[2]);
		 sprintf(buff,"Orientation, %.2f, %.2f, %.2f\n",orientationGlobal[0],orientationGlobal[1],orientationGlobal[2]);
		 printf(buff);


		 cycleDuration = (xTaskGetTickCount()- cycleStart); // calculate in ms
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

	float torqueDemand;
	float orientationGlobalPrev[3]={0.0f};

	// Set parameters for regulator
	torque2angleStruct torque2angleV0_6;
		torque2angleV0_6.coe[0]=-45.488561;
		torque2angleV0_6.coe[1]=148.794907;
		torque2angleV0_6.coe[2]=75.992147;
		torque2angleV0_6.coe[3]=-196.131888;
		torque2angleV0_6.coe[4]=-45.962337;
		torque2angleV0_6.coe[5]=100.708216;
		torque2angleV0_6.coe[6]=12.085230;
		torque2angleV0_6.coe[7]=-23.741009;
		torque2angleV0_6.coe[8]=-1.283942;
		torque2angleV0_6.coe[9]=14.986159;
		torque2angleV0_6.coe[10]=0;
		torque2angleV0_6.velocity=0.6;
		torque2angleV0_6.maxLiftCoe=0.7816;
		torque2angleV0_6.maxStableAngle=11;

	regulator_PID rollController;
		rollController.Kp=1.368792;
		rollController.Ki=0.802290;
		rollController.Kd=0.448888;
		rollController.Nd=13.235897;





  /* Infinite loop */
  for(;;)
  {
	  if(xSemaphoreTake(inputsCalculated_S_Handle,portMAX_DELAY)==pdTRUE){

		  if ( RollControlerActive == true ){
			  rollController.outMax=maxTorqueAvailable(&torque2angleV0_6, d1_stere ,velocityXLocal);
			  rollController.outMin=-rollController.outMax;
			  torqueDemand=PIDcontroller(&rollController,orientationGlobal[0],orientationSetpoint[0],NavigInitDone);
			  stereRequest[0]=torque2angle(torqueDemand,&torque2angleV0_6,d1_stere,velocityXLocal); // right stere angle
			  stereRequest[1]=-stereRequest[0]; // left stere angle
		  }
		  else if (YawControlerActive == true ){
			  rollController.outMax=maxTorqueAvailable(&torque2angleV0_6, d1_stere ,velocityXLocal);
			  rollController.outMin=-rollController.outMax;
			  torqueDemand=PIDcontroller(&rollController,orientationGlobal[0],orientationSetpoint[0],NavigInitDone);
			  stereRequest[0]=torque2angle(torqueDemand,&torque2angleV0_6,d1_stere,velocityXLocal); // right stere angle
			  stereRequest[1]= stereRequest[0]; // left stere angle
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

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the StateController thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
	//int state=0;
	orientationSetpoint[0]=90;

  /* Infinite loop */
  for(;;)
  {


    vTaskDelay(200);
  }
  /* USER CODE END StartTask06 */
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
