#include "AuvControl.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>



#define I2C_TIMEOUT 60
#define Ts 0.06
#define CLK 100000000


#define PI 3.141592f
#define g 9.81f

float debugMatrix[3];
float Q=0.2; //0.1;
float V=1.7; //1.2;

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

	// setting C matrix to 0
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colB; j++) {
            C[i][j] = 0;
        }
    }

    // multiplying
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colB; j++) {
            for (int k = 0; k < colA; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

bool isSet(float input, float derivative , float setpoint , float input_offset , float derivative_offset ){

	bool output;

	if (fabs(input) <= (setpoint + input_offset) ){
		if (fabs(derivative) <= (derivative_offset)){
			output = true;
		}
		else output = false;
	}
	else output = false;

	return output;
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

HAL_StatusTypeDef mpu6050_update(I2C_HandleTypeDef *hi2c, uint8_t ADDRESS,int16_t AccellDataRaw[3], int16_t GyroDataRaw[3]){
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

	AccellDataRaw[0] = (int16_t)(data[0]<<8 | data[1]); // Gyro -> X Axis
	AccellDataRaw[1] = (int16_t)(data[2]<<8 | data[3]); // Gyro -> Y Axis
	AccellDataRaw[2] = (int16_t)(data[4]<<8 | data[5]); // Gyro -> Z Axis

	GyroDataRaw[0] =  (int16_t)(data[8]<<8 | data[9]);   // Accell -> X Axis
	GyroDataRaw[1] =  (int16_t)(data[10]<<8 | data[11]); // Accell -> Y Axis
	GyroDataRaw[2] =  (int16_t)(data[12]<<8 | data[13]); // Accell -> Z Axis
	return status;
}


void accelCalc(int16_t accellDataRaw[3],float accellLocal[3],float orientationAnccelGlobal[3]){

	// AccelDataRaw is input tab, data from accelerometr, size of AccelDataRaw shall be 3;
	// Acceleration is output tab, Acceleration in local frame, size of Acceleration shall be 3;
	// AccelAngle is output tab, Orientation calculated based on accelerometr, size of AccelAngle shall be 3;

	for (uint8_t i=0; i<=2 ; i++){
		accellLocal[i]=accellDataRaw[i]/16384.0f;
	}

	//orientationAnccelGlobal[0]=atan2(accellLocal[1],sqrt(pow(accellLocal[0],2)+pow(accellLocal[2],2)))*180/PI;
	orientationAnccelGlobal[0]=atan2(accellLocal[1],accellLocal[2])*180/PI;
	orientationAnccelGlobal[1]=atan2(-accellLocal[0],sqrt(pow(accellLocal[1],2)+pow(accellLocal[2],2)))*180/PI;
}

void gyroCalc(int16_t gyroDataRaw[3], float gyroDataScaled[3]){

	for (int i=0; i<=2; i++){
		gyroDataScaled[i]=gyroDataRaw[i]/131.0f; // ustalic czy ma tu być minus czy nie
	}
}

void kalmanFilter(float orientationAccellGlobal[3], float gyroGlobal[3], float orientationGlobal[3], float accellLocal[3],float gyroBias[3], bool NavigInitDone){
	float PredictErr; //Q Covariance ...
	float CorrectErr; //R Covariance ...
	static float K[2] = {0,0}; // Kalman Gain
	static float xCorrect[2] = {0,0};
	static float pPredict[2] = {1,1};
	static float xPredict[2],pCorrect[2];
	//static float result[2];
	//static int quart[2]={1,1};

	PredictErr=Q;
	CorrectErr=V;
		for (int i=0;i<=1;i++){
			// 1. Prediction Phase
			xPredict[i] = xCorrect[i] + (Ts * (gyroGlobal[i])); //State prediction
			pPredict[i] = pCorrect[i] + PredictErr; //Covariance prediction
			K[i] = pPredict[i]/(CorrectErr + pPredict[i]); //Kalman's Gain calculation

			// 2. Correction phase
			xCorrect[i] = xPredict[i] + K[i]*(orientationAccellGlobal[i] - xPredict[i]); //State correction
			pCorrect[i] = (1 - K[i])*pPredict[i]; //Covariance correction
			orientationGlobal[i]=xCorrect[i];

		} // end of for()


		if (NavigInitDone == true){
				xCorrect[0]=orientationAccellGlobal[0];
				xCorrect[1]=orientationAccellGlobal[1];
				pPredict[0]=1.0f;
				pPredict[1]=1.0f;
			}

}

void yawEvaluate(float orientationGlobal[3], float angVeloYaw, bool NavigInitDone){
	orientationGlobal[2]+= angVeloYaw*Ts;
	if (NavigInitDone == true) orientationGlobal[2] = 0;
}

void vectTransform(float orientationGlobal[3], float gyroLocal[3],float gyroBias[3],float gyroGlobal[3],float accellLocal[3], float accellGlobal[3]){

	float R[3][3],Rtemp[3][3],Rroll[3][3],Rpitch[3][3],Ryaw[3][3],wGlobal[3][1],wLocal[3][1];
	// Calculate each roll matrix
	R_roll(orientationGlobal[0],Rroll);
	R_pitch(orientationGlobal[1],Rpitch);
	R_yaw(orientationGlobal[2],Ryaw);
	// Calculate general roll matrix
	matrixMultiply(3,3,Ryaw,3,3,Rpitch,Rtemp);// R = Ryaw * Rpitch
	matrixMultiply(3,3,Rtemp,3,3,Rroll,R); // R = R * Rroll

	//wGlobal[0][0] = 0; // unnecessary propably
	//wGlobal[1][0] = 0;
	//wGlobal[2][0] = 0;

	// Calculate Angular Velocity Matrix in global frame based on Rotation Matrix
	wLocal[0][0] = (gyroLocal[0]-gyroBias[0]);
	wLocal[1][0] = (gyroLocal[1]-gyroBias[1]);
	wLocal[2][0] = (gyroLocal[2]-gyroBias[2]);
	matrixMultiply(3,3,R,3,1,wLocal,wGlobal); // transform
	gyroGlobal[0]=wGlobal[0][0];
	gyroGlobal[1]=wGlobal[0][1];
	gyroGlobal[2]=wGlobal[0][2];

	// Calculate Linear acceleration Matrix in global frame based on Rotation Matrix
	wLocal[0][0] = accellLocal[0];
	wLocal[1][0] = accellLocal[1];
	wLocal[2][0] = accellLocal[2];
	matrixMultiply(3,3,R,3,1,wLocal,wGlobal); // transform
	accellGlobal[0]=wGlobal[0][0];
	accellGlobal[1]=wGlobal[0][1];
	accellGlobal[2]=wGlobal[0][2]-1.0; // Substract 1g -> gravity to get pure linear accell

}

void positionEvaluate(float positionGlobal[3],float accellGlobal[3] ,bool NavigInitDone){
	/// Low-Pass Filter here???
	static float velocityGlobal[3]={0.0f};
	//float temp;

	for (int i=0;i<=2;i++){
			//temp=roundf(g*accellGlobal[i]*Ts*10000)/100; // cm/s
			velocityGlobal[i]+=g*accellGlobal[i]*Ts;
			}

	for (int i=0;i<=2;i++){
			positionGlobal[i]+= velocityGlobal[i]*Ts; // cm
			}

	if (NavigInitDone == true){
		for (int i=0;i<=2;i++){
			positionGlobal[i]=0.0f;
			velocityGlobal[i]=0.0f;
		}
	}
}

void velocityLocal(float *velocity, float accelerationLocal[3], float orientationGlobal[3], bool NavigInitDone){
/* Matrix [0;0;1] transform with invers rotation matrix (global to local )
 *                                                                -sin(P)/(cos(P)^2 + sin(P)^2)
(cos(P)*sin(R))/(cos(P)^2*cos(R)^2 + cos(P)^2*sin(R)^2 + cos(R)^2*sin(P)^2 + sin(P)^2*sin(R)^2)
(cos(P)*cos(R))/(cos(P)^2*cos(R)^2 + cos(P)^2*sin(R)^2 + cos(R)^2*sin(P)^2 + sin(P)^2*sin(R)^2)
 *
 */

	//*velocity += g *( accelerationLocal[0] - sin(deg2rad(orientationGlobal[1])) )* Ts;
	float grav_temp = sin(deg2rad(orientationGlobal[0]))*sin(deg2rad(orientationGlobal[2])) + (cos(deg2rad(orientationGlobal[0]))*cos(deg2rad(orientationGlobal[2])) * sin(deg2rad(orientationGlobal[1])));
	*velocity += g *( accelerationLocal[0] + grav_temp )* Ts;
	//〖v_lx〗_k=〖v_lx〗_(k-1)+(a_lx-g(sin(φ)*sin(ψ) + cos(φ)*cos(ψ)*sin(θ))) T_s
	debugMatrix[0]= grav_temp;
	debugMatrix[1]= accelerationLocal[0] + sin(deg2rad(orientationGlobal[1]));
	if (NavigInitDone == true){
		*velocity = 0.0f;
	}
	// Low Pass Filter maybe or use only 2 digits
}




float PIDcontroller(regulator_PID *ctrlParam, float input,float setpoint, bool reset){
	// returns required torque
	float error = input - setpoint;
	float derivative = (ctrlParam->Kd * ctrlParam->Nd * (error - ctrlParam->error_prev)) / (1 + (ctrlParam->Nd * Ts));
	float output = ctrlParam->Kp*error + derivative + ctrlParam->Ki*ctrlParam->integrator;

	ctrlParam->error_prev=error; // store prevoius error

	if (output > ctrlParam->outMax) output = ctrlParam->outMax; // Max output limitation
	else if (output < ctrlParam->outMin) output = ctrlParam->outMin; // Min output limitation
	else ctrlParam-> integrator += error*Ts; // integration with anty wind-up (clamping method)

	if (reset == true){
			ctrlParam->integrator=0.0f;
			ctrlParam->error_prev=0.0f;
			output = 0.0f;
		}

	return output;
}

void PIDcontroller_Reset(regulator_PID *ctrlParam){
	ctrlParam->integrator=0.0f;
	ctrlParam->error_prev=0.0f;
}

void servoInit(TIM_HandleTypeDef *htim){
	// Start Servo PWM
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
}

void servoSet(float servoAngle[2],TIM_HandleTypeDef *htim){
uint16_t OCR_register_Servo_Right, OCR_register_Servo_Left ;
OCR_register_Servo_Right = (uint16_t)((servoAngle[0]*2500.0f/360.0f) + 3749);
OCR_register_Servo_Left = (uint16_t)((servoAngle[1]*2500.0f/360.0f) + 3749);
__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, OCR_register_Servo_Right);
__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, OCR_register_Servo_Left);
}

void motorInit(TIM_HandleTypeDef *htim){
	// Start Servo PWM
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
}

void motorSet(float power_percent,TIM_HandleTypeDef *htim){

uint16_t OCR_register;
OCR_register = (uint16_t)(power_percent);
__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, OCR_register);
}


/*
 float maxTorqueAvailable(torque2angleStruct *scalingParam, float forceArm ,float velocity){
	return waterDensity*hydrofoilFrontArea*scalingParam->maxLiftCoe*forceArm*pow(velocity,2);
}

float torque2angle(float torqueDemand, torque2angleStruct *scalingParam, float forceArm , float velocity ){
	// Calculate right servo angle, left shall be set to the same or negative value
	float Cy=(torqueDemand/forceArm)/(waterDensity*hydrofoilFrontArea*pow(velocity,2));
	float angleRight=0;
		for (int i=0; i<10; i++){
			angleRight = angleRight + scalingParam->coe[i]*pow(Cy,(10-i));
		}
		angleRight=angleRight+scalingParam->coe[10];

		if (angleRight > scalingParam->maxStableAngle ){
			angleRight = scalingParam->maxStableAngle;
		}
		else if (angleRight < -(scalingParam->maxStableAngle) ){
			angleRight = -(scalingParam->maxStableAngle);
		}
		return angleRight;
}
 *
 *
 */


/*
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

float filter(filterStruct *filterData ,float input, bool NavigInitDone){

// If filter type is FIR, int denum = 0 ; float denumMatrix = NULL

	float output=0.0f;

	if (NavigInitDone == true ){
		for (int i=0;i<=10;i++){
			filterData->inRegister[i]=0.0f;
			filterData->outRegister[i]=0.0f;
		}
	}

	// shift in register
	for (int i=filterData->order; i>0 ; i--){
		filterData->inRegister[i]=filterData->inRegister[i-1];
	}

	// add curect input to register
	filterData->inRegister[0]=input;
	// calc output

	for (int i=0; i<= filterData->order ; i++){
		output+= filterData->inRegister[i]*filterData->numMatrix[i];
	}

	if (filterData->IIR == true ){  // - if this is IIR filter

		for (int i=1; i <= filterData->order ; i++){
			output -= filterData->outRegister[i]*filterData->denumMatrix[i];
		}
		// shift output register
		for (int i=filterData->order; i>0 ; i--){
			filterData->outRegister[i]=filterData->outRegister[i-1];
		}
		filterData->outRegister[0]=output;

	}

	return output;
}

*/

