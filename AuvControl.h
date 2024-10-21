#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef AuvControl_h
#define AuvControl_h

HAL_StatusTypeDef mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t ADDRESS);
HAL_StatusTypeDef mpu6050_update(I2C_HandleTypeDef *hi2c, uint8_t ADDRESS,int16_t AccelDataRaw[3], int16_t GyroDataRaw[3]);
void accelCalc(int16_t AccelDataRaw[3],float Acceleration[3],float AccelAngle[3]);
void gyroCalc(int16_t GyroDataRaw[3], float GyroScaledData[3]);
void kalmanFilter(float AccellAngle[3], float GyroScaledData[3], float OrientationAngles[3], bool NavigInitDone);
float ellipFilter(float GyroScaled[3], float controlOutput[1], bool NavigInitDone);
void YawEvaluate(float orientationAngles[3], float GyroScaled[3],float gyroBias,bool NavigInitDone);
//void navigInit(float gyroScaled[3],float gyroBias[3]);
void navigInitCheck();
#endif
