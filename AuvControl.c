#include "AuvControl.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>


#define I2C_TIMEOUT 200



HAL_StatusTypeDef mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t ADDRESS){
// initialization of MPU6050
// input data:
// I2C_HandleTypeDef *hi2c - I2C handler

// default timeout - HAL_MAX_DELAY
	HAL_StatusTypeDef status;
	uint8_t data[2]; // first byte - address, second byte - data



	if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY) {
	// Power Management
	data[0]=0x6B;
	data[1]=0b00001000;
	status=HAL_I2C_Master_Transmit(hi2c, ADDRESS, data, 2, I2C_TIMEOUT);

	// Low Pass Filter
	data[0]=0x1A;
	data[1]=0b00000000;
	status=HAL_I2C_Master_Transmit(hi2c, ADDRESS, data, 2, I2C_TIMEOUT);

	// Gyro Config
	data[0]= 0x1B;
	data[1]= 0; // Set full range to +/- 250 deg/s
	status=HAL_I2C_Master_Transmit(hi2c, ADDRESS, data, 2, I2C_TIMEOUT);

	// Accel Config
	data[0]= 0x1C;
	data[1]= 0; // Set full range to +/- 2g deg/s
	status=HAL_I2C_Master_Transmit(hi2c, ADDRESS, data, 2, I2C_TIMEOUT);
	}

	return status;
}



HAL_StatusTypeDef mpu6050_update(I2C_HandleTypeDef *hi2c, uint8_t ADDRESS,int16_t AccelDataRaw[3], int16_t GyroDataRaw[3]){

	HAL_StatusTypeDef status;
	uint8_t data[14];
	uint8_t addr = 0x3B;
	if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY) {
	status=HAL_I2C_Master_Transmit(hi2c, ADDRESS, &addr, 1 , I2C_TIMEOUT); // send request for first register
	status=HAL_I2C_Master_Receive(hi2c, ADDRESS, data, 14 , I2C_TIMEOUT); // read 14 bytes
	}
	AccelDataRaw[0] = (int16_t)(data[0]<<8 | data[1]); // Gyro -> X Axis
	AccelDataRaw[1] = (int16_t)(data[2]<<8 | data[3]); // Gyro -> Y Axis
	AccelDataRaw[2] = (int16_t)(data[4]<<8 | data[5]); // Gyro -> Z Axis

	GyroDataRaw[0] = (int16_t)(data[8]<<8 | data[9]);   // Accell -> X Axis
	GyroDataRaw[1] = (int16_t)(data[10]<<8 | data[11]); // Accell -> Y Axis
	GyroDataRaw[2] = (int16_t)(data[12]<<8 | data[13]); // Accell -> Z Axis

	return status;
}


void accelCalc(int16_t AccelDataRaw[3],float Acceleration[3],float AccelAngle[3]){

	// AccelDataRaw is input tab, data from accelerometr, size of AccelDataRaw shall be 3;
	// Acceleration is output tab, Acceleration in local frame, size of Acceleration shall be 3;
	// AccelAngle is output tab, Orientation calculated based on accelerometr, size of AccelAngle shall be 3;

	for (uint8_t i=0; i<=2 ; i++){
		Acceleration[i]=AccelDataRaw[i]/16384.0f;
	}

	AccelAngle[0]=atan2(Acceleration[1],sqrt(pow(Acceleration[0],2)+pow(Acceleration[2],2)));
	AccelAngle[1]=atan2(Acceleration[0],sqrt(pow(Acceleration[1],2)+pow(Acceleration[2],2)));
}

/*
float gyroCalc(){

}

float kalmanFilter(){

}

float YawCalc(){

}
*/
