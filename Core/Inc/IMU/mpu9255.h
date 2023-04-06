/*
 * mpu9255.h
 *
 *  Created on: Dec 26, 2021
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit
 */

#ifndef INC_9255_H_
#define INC_9255_H_

#endif /* INC_9255_H_ */

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "mpu9255_defs.h"
typedef struct
{
	volatile float AccelX;
	volatile float AccelY;
	volatile float AccelZ;

	volatile float GyroX;
	volatile float GyroY;
	volatile float GyroZ;

	volatile float MagX;
	volatile float MagY;
	volatile float MagZ;

	volatile float pitch;
	volatile float roll;
	volatile float yaw;

	volatile float axNEDmG,ayNEDmG,azNEDmG;
	volatile float zAccelAverage;

	volatile float gravityCompensatedAccelMg;

} MPU9255_t;

uint8_t MPU9255_Init(I2C_HandleTypeDef *I2Cx, uint8_t isCalibMagReq);

void readAll(I2C_HandleTypeDef *I2Cx, MPU9255_t*DataStruct);

void getMres();
void getGres();
void getAres();

void readAccelData(I2C_HandleTypeDef *I2Cx, int16_t * destination);
void readGyroData(I2C_HandleTypeDef *I2Cx, int16_t * destination);
void readMagData(I2C_HandleTypeDef *I2Cx, int16_t * destination);

void initAK8963(I2C_HandleTypeDef *I2Cx, float * destination);
void initMPU9250(I2C_HandleTypeDef *I2Cx);

void calibrateMPU9250(I2C_HandleTypeDef *I2Cx, float * dest1, float * dest2);
uint8_t calibrateMag(I2C_HandleTypeDef *I2Cx, float *dest1, float *dest2,uint8_t readFromMem);

void MPU9250SelfTest(I2C_HandleTypeDef *I2Cx, float * destination);

void QuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

float imu_gravityCompensatedAccel(float ax, float ay, float az);
