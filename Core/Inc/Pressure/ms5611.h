#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifndef MS5611_H_
#define MS5611_H_
//#define MS5611_TEST

//#define DEBUG_MS56
#define MS5611_SAMPLE_PERIOD_MS         10

#define MS5611_READ_TEMPERATURE 		11
#define MS5611_READ_PRESSURE			22

#define I2C_ID_MS5611 (0x77<<1)
// max conversion time with max OSR (4096) =  9.04mS
#define SENSOR_TIMEOUT      15

#define ALTITUDE_REFERENCE_PRESSURE		1013.250f

typedef enum sensorState{
	MS5611_SENSOR_OK = 0,
	MS5611_SENSOR_ERROR = 1,
	MS5611_PRES_DATA_READY = 2,
	MS5611_TEMP_DATA_READY = 3
}MS5611_State;

typedef enum readingState{
	MS5611_Pressure_Read = 0,
	MS5611_Temp_Read = 1
}MS5611_Reading_State;

typedef struct
{
	I2C_HandleTypeDef *I2Cdev;
	MS5611_Reading_State readingState;
	volatile float Pres_hpa, avgPres_hpa;

	volatile float Temp_C;

}MS5611_typedef;

extern MS5611_State MS5611_Init(volatile MS5611_typedef* ms56dev, I2C_HandleTypeDef *I2Cx);
extern void 		MS5611_AveragedSample(volatile MS5611_typedef* ms56dev ,int nSamples, int avgAltFlag);
MS5611_State 		MS5611_SampleStateMachine(volatile MS5611_typedef* ms56dev);
extern void 		MS5611_InitializeSampleStateMachine(volatile MS5611_typedef* ms56dev);

void 		        __ms56_TriggerPressureSample(volatile MS5611_typedef* ms56dev);
void 		        __ms56_TriggerTemperatureSample(volatile MS5611_typedef* ms56dev);
uint32_t  	        __ms56_ReadPressureSample(volatile MS5611_typedef* ms56dev);
uint32_t  	        __ms56_ReadTemperatureSample(volatile MS5611_typedef* ms56dev);
void 		        __ms56_CalculateTemperatureCx10(void);
float 		        __ms56_CalculatePressure_hPa(void);
void 		        __ms56_CalculateSensorNoisePa(void);
HAL_StatusTypeDef   __ms56_ReadCoefficients(volatile MS5611_typedef* ms56dev);
float 		        __ms56_hPa2Cm(float pa);
void 		        __ms56_Test(int nSamples);
float __getAlt(float P, float celsiusSample, float AltRef);
#endif // MS5611_H_
