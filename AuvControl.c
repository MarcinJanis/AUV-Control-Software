#include "AuvControl.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>



#define I2C_TIMEOUT 200
#define Ts 0.05
#define PI 3.141592

float deg2rad(float input){
	return input*PI/180.0;
}
float rad2deg(float input){
	return input*180.0/PI;
}

void R_roll(float angle, float R[3][3]) {
    float rad = deg2rad(angle);
    R[0][0] = 1;         R[0][1] = 0;            R[0][2] = 0;
    R[1][0] = 0;         R[1][1] = cos(rad);     R[1][2] = -sin(rad);
    R[2][0] = 0;         R[2][1] = sin(rad);     R[2][2] = cos(rad);
}

void R_pitch(float angle, float R[3][3]) {
    float rad = deg2rad(angle); // Użyj konwersji do radianów
    R[0][0] = cos(rad);        R[0][1] = 0;   R[0][2] = sin(rad);
    R[1][0] = 0;               R[1][1] = 1;   R[1][2] = 0;
    R[2][0] = -sin(rad);       R[2][1] = 0;   R[2][2] = cos(rad);
}

void R_yaw(float angle, float R[3][3]) {
    float rad = deg2rad(angle); // Użyj konwersji do radianów
    R[0][0] = cos(rad);        R[0][1] = -sin(rad);   R[0][2] = 0;
    R[1][0] = sin(rad);        R[1][1] = cos(rad);    R[1][2] = 0;
    R[2][0] = 0;               R[2][1] = 0;           R[2][2] = 1;
}

void matrixMultiply(int rowsA, int colA, float A[rowsA][colA],
                    int rowsB, int colB, float B[rowsB][colB],
                    float C[rowsA][colB]) {

    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colB; j++) {
            C[i][j] = 0;
        }
    }


    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colB; j++) {
            for (int k = 0; k < colA; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


HAL_StatusTypeDef mpu6050_init(I2C_HandleTypeDef *hi2c, uint8_t ADDRESS){
// initialization of MPU6050
// input data:
// I2C_HandleTypeDef *hi2c - I2C handler
// uint8_t ADDRESS - Address of device
// timeout - global var I2C_TIMEOUT

	HAL_StatusTypeDef status; // status of communication
	uint8_t data[2]; // sending data format: first byte - address, second byte - data

	if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY) {

		// Power Management
		data[0]=0x6B;
		data[1]=0b00001000; // turn on, temp. measurement off
		status=HAL_I2C_Master_Transmit(hi2c, ADDRESS, data, 2, I2C_TIMEOUT);

		// Low Pass Filter
		data[0]=0x1A;
		data[1]=0b00000010; // flow pass filter for: Accel 94 Hz, 3 ms delay, Gyro 98, 2.8 ms delay
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
	//NavigationSysInitDone = true;
	return status;
}



HAL_StatusTypeDef mpu6050_update(I2C_HandleTypeDef *hi2c, uint8_t ADDRESS,int16_t AccelDataRaw[3], int16_t GyroDataRaw[3]){
// Get measurement from mpu 6050 measurement unit
// Inputs:
// I2C_HandleTypeDef *hi2c - I2c Handler
// uint8_t ADDRESS - device address
// int16_t AccelDataRaw[3] - Matrix with Raw Accell Measurement to save in
// int16_t GyroDataRaw[3] - Matrix with Raw Gyro Measurement to save in

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

	GyroDataRaw[0] =  (int16_t)(data[8]<<8 | data[9]);   // Accell -> X Axis
	GyroDataRaw[1] =  (int16_t)(data[10]<<8 | data[11]); // Accell -> Y Axis
	GyroDataRaw[2] =  (int16_t)(data[12]<<8 | data[13]); // Accell -> Z Axis
	return status;
}


void accelCalc(int16_t AccelDataRaw[3],float Acceleration[3],float AccelAngle[3]){

	// AccelDataRaw is input tab, data from accelerometr, size of AccelDataRaw shall be 3;
	// Acceleration is output tab, Acceleration in local frame, size of Acceleration shall be 3;
	// AccelAngle is output tab, Orientation calculated based on accelerometr, size of AccelAngle shall be 3;

	for (uint8_t i=0; i<=2 ; i++){
		Acceleration[i]=AccelDataRaw[i]/16384.0f;
	}

	AccelAngle[0]=atan2(Acceleration[1],sqrt(pow(Acceleration[0],2)+pow(Acceleration[2],2)))*180/PI;
	AccelAngle[1]=atan2(-Acceleration[0],sqrt(pow(Acceleration[1],2)+pow(Acceleration[2],2)))*180/PI;
}


void gyroCalc(int16_t GyroDataRaw[3], float GyroScaledData[3]){

	for (int i=0; i<=2; i++){
		GyroScaledData[i]=GyroDataRaw[i]/131.0f; // ustalic czy ma tu być minus czy nie
	}
}




void kalmanFilter(float AccellAngle[3], float GyroScaledData[3], float OrientationAngles[3], float AccellScaled[3],float gyroBias[3], bool NavigInitDone){
	static const float PredictErr=0.1; //Q Covariance ...
	static const float CorrectErr = 1.2; //R Covariance ...
	static float K[2] = {0,0}; // Kalman Gain
	static float xCorrect[2] = {0,0};
	static float pPredict[2] = {1,1};
	static float xPredict[2],pCorrect[2];
	//static float result[2];
	//static int quart[2]={1,1};



		for (int i=0;i<=1;i++){
		// 1. Prediction Phase
		xPredict[i] = xCorrect[i] + (Ts * (GyroScaledData[i]-gyroBias[i])); //State prediction
		pPredict[i] = pCorrect[i] + PredictErr; //Covariance prediction
		K[i] = pPredict[i]/(CorrectErr + pPredict[i]); //Kalman's Gain calculation

		// 2. Correction phase
		xCorrect[i] = xPredict[i] + K[i]*(AccellAngle[i] - xPredict[i]); //State correction
		pCorrect[i] = (1 - K[i])*pPredict[i]; //Covariance correction
		//result[i]= xCorrect[i]; //output mapping

		//if (quart == 1 && result[i]>90) quart = 1;
		//if (quart == 1 && result[i]>90) quart = 1;

		if (i==0){
			if (AccellScaled[1]>0 && AccellScaled[2]>0) OrientationAngles[i]=xCorrect[i];				// 1st quarter
			else if (AccellScaled[1]>0 && AccellScaled[2]<0) OrientationAngles[i]= 180 - xCorrect[i];	// 2nd quarter
			else if (AccellScaled[1]<0 && AccellScaled[2]<0) OrientationAngles[i]= -180 - xCorrect[i];			// 3rd quarter
			else if (AccellScaled[1]<0 && AccellScaled[2]>0) OrientationAngles[i]= xCorrect[i];	// 4rd quarter
		}
		else {
			if (AccellScaled[0]<0 && AccellScaled[2]>0) OrientationAngles[i]=xCorrect[i];				// 1st quarter
			else if (AccellScaled[0]<0 && AccellScaled[2]<0) OrientationAngles[i]= 180 - xCorrect[i];	// 2nd quarter
			else if (AccellScaled[0]>0 && AccellScaled[2]<0) OrientationAngles[i]= - 180 - xCorrect[i];	// 3rd quarter
			else if (AccellScaled[0]>0 && AccellScaled[2]>0) OrientationAngles[i]= xCorrect[i];			// 4th quarter
		}
		}


		if (NavigInitDone == true){
				xCorrect[0]=AccellAngle[0];
				xCorrect[1]=AccellAngle[1];
				pPredict[0]=1;
				pPredict[1]=1;
			}

}



//float R_yaw(float yaw, float R[3][3]){
//matrixMultiply


void YawEvaluate(float orientationAngles[3], float GyroScaled[3],float gyroBias[3],bool NavigInitDone, float debugMatrix[2]){

	float R[3][3],Rtemp[3][3],Rroll[3][3],Rpitch[3][3],Ryaw[3][3],wGlobal[3][1],wLocal[3][1];
	R_roll(orientationAngles[0],Rroll);
	R_pitch(orientationAngles[1],Rpitch);
	R_yaw(orientationAngles[2],Ryaw);

	matrixMultiply(3,3,Ryaw,3,3,Rpitch,Rtemp);// R = Ryaw * Rpitch
	debugMatrix[0]=Rtemp[0][0];
	matrixMultiply(3,3,Rtemp,3,3,Rroll,R); // R = R * Rroll
	debugMatrix[0]=R[0][0];
	wGlobal[0][0] = 0;
	wGlobal[1][0] = 0;
	wGlobal[2][0] = 0;
	wLocal[0][0] = (GyroScaled[0]-gyroBias[0]);
	wLocal[1][0] = (GyroScaled[1]-gyroBias[1]);
	wLocal[2][0] = (GyroScaled[2]-gyroBias[2]);

	matrixMultiply(3,3,R,3,1,wLocal,wGlobal); // transform angular velocity from local to global frame

	orientationAngles[2]+= wGlobal[2][0]*Ts;
	if (NavigInitDone == true)orientationAngles[2] = 0;
}



