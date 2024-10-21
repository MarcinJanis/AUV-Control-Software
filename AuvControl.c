#include "AuvControl.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>


#define I2C_TIMEOUT 200
#define Ts 0.1



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

	AccelAngle[0]=atan2(Acceleration[1],sqrt(pow(Acceleration[0],2)+pow(Acceleration[2],2)))*180/3.14;
	AccelAngle[1]=atan2(-Acceleration[0],sqrt(pow(Acceleration[1],2)+pow(Acceleration[2],2)))*180/3.14;
}


void gyroCalc(int16_t GyroDataRaw[3], float GyroScaledData[3]){

	for (int i=0; i<=2; i++){
		GyroScaledData[i]=GyroDataRaw[i]/131.0f; // ustalic czy ma tu być minus czy nie
	}
}




void kalmanFilter(float AccellAngle[3], float GyroScaledData[3], float OrientationAngles[3], bool NavigInitDone){
	static const float PredictErr=0.1; //Q Covariance ...
	static const float CorrectErr=1; //R Covariance ...
	static float K[2] = {0,0}; // Kalman Gain
	static float xCorrect[2] = {0,0};
	static float pPredict[2] = {1,1};
	static float xPredict[2],pCorrect[2];


		for (int i=0;i<=1;i++){
		// 1. Prediction Phase
		xPredict[i] = xCorrect[i] + (Ts * GyroScaledData[i]); //State prediction
		pPredict[i] = pCorrect[i] + PredictErr; //Covariance prediction
		K[i] = pPredict[i]/(CorrectErr + pPredict[i]); //Kalman's Gain calculation

		// 2. Correction phase
		xCorrect[i] = xPredict[i] + K[i]*(AccellAngle[i] - xPredict[i]); //State correction
		pCorrect[i] = (1 - K[i])*pPredict[i]; //Covariance correction
		OrientationAngles[i] = xCorrect[i]; //output mapping
			}

		if (NavigInitDone == true){
				xCorrect[0]=AccellAngle[0];
				xCorrect[1]=AccellAngle[1];
				pPredict[0]=1;
				pPredict[1]=1;
				NavigInitDone = false;
			}

}

void YawEvaluate(float orientationAngles[3], float GyroScaled[3],float gyroBias,bool NavigInitDone){

	orientationAngles[2]+= (GyroScaled[2]-gyroBias)*Ts;
	if (NavigInitDone == true)orientationAngles[2] = 0;
}

/*

float ellipFilter(float GyroScaled[3], float controlOutput[1],bool NavigInitDone) {
    static const int FilterOrder = 20;
    static float X[21] = {0}; // 4 elementy
    //static float Y[3] = {0, 0, 0}; // 3 elementy
    static const float bias = -2.6537;
    static const float filterCooNum[21] ={
    	    -0.029225, -0.034278, -0.039203, -0.043886, -0.048214,
    	    -0.052079, -0.055387, -0.058052, -0.060006, -0.061200,
    	     0.965081, -0.061200, -0.060006, -0.058052, -0.055387,
    	    -0.052079, -0.048214, -0.043886, -0.039203, -0.034278,
    	    -0.029225
    	};

   // static const float filterCooDen[4] = {1, -2.885227, 2.776314, -0.890847};

    float output = 0.0f;

    // Shift input samples
    for (int i = FilterOrder; i > 0; i--) {
        X[i] = X[i - 1];
    }

    // Add current input to register
    X[0] = GyroScaled[2]-bias;

    // Calculate output sample
    for (int i = 0; i <= FilterOrder; i++) {
        output += filterCooNum[i] * X[i];
    }
   // for (int i = 1; i < FilterOrder; i++) {
   //     output -= filterCooDen[i] * Y[i - 1];
   // }

    // Shift output samples
    //for (int i = FilterOrder - 1; i > 0; i--) {
    //    Y[i] = Y[i - 1];
    // }
    //Y[0] = output; // Map output
    controlOutput[0] = output;
    return output;
}

*/

/*

	static const int FilterOrder=3;
	static float X[4]={0,0,0,0};
	static float Y[4]={0,0,0,0};

	static const float filterCooNum[4]={0.944130,-2.832064,2.832064,-0.944130};
	static const float filterCooDen[4]={1,-2.885227,2.776314,-0.890847};
	//static const float filterGain[1]={0.808602};
	static float output;
	// shift inputs sample register


		output = 0.0f;

		// shift inputs register
		for (int i=FilterOrder;i>0;i--){
			X[i]=X[i-1];
		}

		// add currect input to register

			 X[0] = GyroScaled[2];

		// Calculate actual output sample
		for (int i=0;i<=FilterOrder;i++){
			output +=  filterCooNum[i]*X[i];
		}
		for (int i=0;i<=FilterOrder;i++){
			output -=  filterCooDen[i]*Y[i];
		}
		// shift outputs samples register
		for (int i=FilterOrder;i>0;i--){
			Y[i]=Y[i-1];
		}
		Y[0]=output; // map output
		controlOutput[0]=Y[0];

		return Y[0];// map output
	*/

