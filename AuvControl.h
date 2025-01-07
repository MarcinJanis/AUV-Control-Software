#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef AuvControl_h
#define AuvControl_h

// Math Support Functions:
float deg2rad(float input);	// covert degrees to radians
float rad2deg(float input);	// covert radians to degrees
void R_roll(float angle, float R[3][3]);	// create Roll rotation Matrix
void R_pitch(float angle, float R[3][3]);	// create Pitch rotation Matrix
void R_yaw(float angle, float R[3][3]);	// create Yaw rotation Matrix
void matrixMultiply(int rowsA, int colA, float A[rowsA][colA],
                    int rowsB, int colB, float B[rowsB][colB],
                    float C[rowsA][colB]);	// Multiplication of two matrix

bool isSet(float input, float derivative , float setpoint , float input_offset , float derivative_offset );	// Check if set point is reached
// Set point is consider as reached, when input is equal to setpoint (with offset) and rate of input changes is less than it's offset


// Communication:
HAL_StatusTypeDef mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t ADDRESS);	// MPU-6050 configuration
HAL_StatusTypeDef mpu6050_update(I2C_HandleTypeDef *hi2c, uint8_t ADDRESS,int16_t AccellDataRaw[3], int16_t GyroDataRaw[3]);	// Get data from MPU-6050

// Orientation caluclation:
void accelCalc(int16_t accellDataRaw[3],float accellLocal[3],float orientationAccelGlobal[3]);	// Calculation of acceleration and orientation based on accelerometr measurement
void gyroCalc(int16_t gyroDataRaw[3], float gyroDataScaled[3]); // Calculation of angle velocity based on gyroscope measurement
void kalmanFilter(float orientationAccellGlobal[3], float gyroGlobal[3], float orientationGlobal[3], float accellLocal[3],float gyroBias[3], bool NavigInitDone);	// Kalman filtering
void yawEvaluate(float orientationGlobal[3], float angVeloYaw, bool NavigInitDone);	// Caluclate Yaw angle
void vectTransform(float orientationGlobal[3], float gyroLocal[3],float gyroBias[3],float gyroGlobal[3],float accellLocal[3], float accellGlobal[3]);	// Transform from local to global body frame


// Control:
typedef struct{ // Structure contains regulator parameters
	float T; // sampling time
	float Kp,Ki,Kd,Nd; // Gain, and filter coefficient
	float integrator,error_prev,deriv_prev; // sum, previous error, previous derivative factor
	float outMin,outMax; // output limitation
	bool isSaturation; // output Saturation flag
}regulator_PID;

typedef enum { // What should be regulated
    Roll = 0,
    Pitch = 1,
    Yaw = 2,
}Task;

float PIDcontroller(regulator_PID *ctrlParam, float input,float setpoint, bool reset);	// Returns output of PID controller
void PIDcontroller_Reset(regulator_PID *ctrlParam);	// Reset regulation

void servoInit(TIM_HandleTypeDef *htim);	// Init Timers for Servo
void servoSet(float servoAngle[2],TIM_HandleTypeDef *htim);	//Set Servo for requested angle

void motorInit(TIM_HandleTypeDef *htim);	// Init timer for motor
void motorSet(float power_percent,TIM_HandleTypeDef *htim);	// Set motor for requested power
#endif



/*
Unused funtions
void positionEvaluate(float positionGlobal[3],float accellGlobal[3] ,bool NavigInitDone);
void velocityLocal(float *velocity, float accelerationLocal[3], float orientationGlobal[3], bool NavigInitDone);
float filter(filterStruct *filterData ,float input, bool NavigInitDone)
typedef struct{
	// Max size of tab will be 10
	// to define before use
	 bool IIR; // true if its IIR type filter
	 int order; // max is 10
	 float numMatrix[11];
	 float denumMatrix[11];
	// memory
	 float inRegister[11]; // need to be init
	 float outRegister[11]; // need to be init
}filterStruct;
 */

