#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef AuvControl_h
#define AuvControl_h

// Math Support:
float deg2rad(float input);
float rad2deg(float input);
void R_roll(float angle, float R[3][3]);
void R_pitch(float angle, float R[3][3]);
void R_yaw(float angle, float R[3][3]);
void matrixMultiply(int rowsA, int colA, float A[rowsA][colA],
                    int rowsB, int colB, float B[rowsB][colB],
                    float C[rowsA][colB]);
bool isSet(float input, float derivative , float setpoint , float input_offset , float derivative_offset );
// Communication:
HAL_StatusTypeDef mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t ADDRESS);
HAL_StatusTypeDef mpu6050_update(I2C_HandleTypeDef *hi2c, uint8_t ADDRESS,int16_t AccellDataRaw[3], int16_t GyroDataRaw[3]);

// Orientation evaluate:
void accelCalc(int16_t accellDataRaw[3],float accellLocal[3],float orientationAnccelGlobal[3]);
void gyroCalc(int16_t gyroDataRaw[3], float gyroDataScaled[3]);
void kalmanFilter(float orientationAccellGlobal[3], float gyroGlobal[3], float orientationGlobal[3], float accellLocal[3],float gyroBias[3], bool NavigInitDone);
void yawEvaluate(float orientationGlobal[3], float angVeloYaw, bool NavigInitDone);
void vectTransform(float orientationGlobal[3], float gyroLocal[3],float gyroBias[3],float gyroGlobal[3],float accellLocal[3], float accellGlobal[3]);
void positionEvaluate(float positionGlobal[3],float accellGlobal[3] ,bool NavigInitDone);
void velocityLocal(float *velocity, float accelerationLocal[3], float orientationGlobal[3], bool NavigInitDone);

// Control:
typedef struct{ // Struct with regulator parametr
	float Kp,Ki,Kd,Nd;
	float integrator,error_prev; // need to be reset
	float outMin,outMax;
}regulator_PID;

typedef enum { // What should be regulated
    Roll = 0,
    Pitch = 1,
    Yaw = 2,
}Task;

float PIDcontroller(regulator_PID *ctrlParam, float input,float setpoint, bool reset);
void PIDcontroller_Reset(regulator_PID *ctrlParam);

void servoInit(TIM_HandleTypeDef *htim);
void servoSet(float servoAngle[2],TIM_HandleTypeDef *htim);

void motorInit(TIM_HandleTypeDef *htim);
void motorSet(float power_percent,TIM_HandleTypeDef *htim);
#endif



/*
 *
 *
 *
float maxTorqueAvailable(torque2angleStruct *scalingParam, float forceArm ,float velocity);
float torque2angle(float torqueDemand, torque2angleStruct *scalingParam, float forceArm , float velocity );
 * typedef struct{
	// Struct with hydrofoil characteristic for specific velocity
	float velocity;
	float coe[11];
	float maxLiftCoe;
	float maxStableAngle;
}torque2angleStruct;
 *
 *
 */

