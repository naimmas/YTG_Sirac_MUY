#include "Pressure/ms5611.h"
#include <math.h>
#include <stdio.h>

#include "stm32h7xx_hal.h"
#include "userDef/userDef.h"

//#define DEBUG_MS56
//#define MS5611_measure_noise

static uint16_t cal_[6];
static int32_t tref_;
static int64_t offT1_;
static int64_t sensT1_;
static int32_t tempCx100_;
static uint32_t D1_;
static uint32_t D2_;
static int32_t dT_;

MS5611_State MS5611_Init(volatile MS5611_typedef *ms56dev,
		I2C_HandleTypeDef *I2Cx) {
	HAL_StatusTypeDef state;
	ms56dev->Pres_hpa = ms56dev->avgPres_hpa = ms56dev->Temp_C = 0.0f;
	ms56dev->I2Cdev = I2Cx;
	state = __ms56_ReadCoefficients(ms56dev);
#ifdef DEBUG_MS56
	printf("\r\nCalib Coeffs : %d %d %d %d %d %d\r\n",cal_[0],cal_[1],cal_[2],cal_[3],cal_[4],cal_[5]);
#endif
	tref_ = (uint32_t) (cal_[4]) << 8;
	offT1_ = (int64_t) (cal_[1]) << 16;
	sensT1_ = (int64_t) (cal_[0]) << 15;
	return state == HAL_OK ? MS5611_SENSOR_OK : MS5611_SENSOR_ERROR;
}

#ifdef MS5611_MEASURE_NOISE

#define MAX_TEST_SAMPLES    100
extern char gszBuf[];
static float pa[MAX_TEST_SAMPLES];
static float z[MAX_TEST_SAMPLES];

void __ms56_CalculateSensorNoisePa(int nSamples) {
	int n;
	float paMean, zMean, zVariance, paVariance;
	paMean = 0.0f;
	zMean = 0.0f;
	paVariance = 0.0f;
	zVariance = 0.0f;
	for (n = 0; n < nSamples; n++) {
		__ms56_TriggerTemperatureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D2_ = __ms56_ReadTemperatureSample();
		__ms56_CalculateTemperatureCx10();
		__ms56_TriggerPressureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D1_ = __ms56_ReadPressureSample();
		pa[n] = MS5611_CalculatePressurePa();
		z[n] =  MS5611_Pa2Cm(pa[n]);
		paMean += pa[n];
		zMean += z[n];
	}
	paMean /= nSamples;
	zMean /= nSamples;
#ifdef DEBUG_MS56
	printf("paMean = %dPa, zMean = %dcm\r\n",(int)paMean,(int)zMean);
#endif
	for (n = 0; n < nSamples; n++) {
		paVariance += (pa[n]-paMean)*(pa[n]-paMean);
		zVariance += (z[n]-zMean)*(z[n]-zMean);
		//printf(&huart1,"%d %d\r\n",(int)pa[n],(int)z[n]);
	}
	paVariance /= (nSamples-1);
	zVariance /= (nSamples-1);
#ifdef DEBUG_MS56
	printf("\r\npaVariance %d  zVariance %d\r\n",(int)paVariance, (int)zVariance);
#endif
}
#endif

void MS5611_AveragedSample(volatile MS5611_typedef *ms56dev, int nSamples,
		int avgAltFlag) {
	float tAccum, n, pa, pAccum;
	float tc;

	pAccum = 0;
	tAccum = 0;
	float zAccum = 0;
	n = nSamples;
	while (n--) {
		__ms56_TriggerTemperatureSample(ms56dev);
		HAL_Delay(MS5611_SAMPLE_PERIOD_MS);
		D2_ = __ms56_ReadTemperatureSample(ms56dev);
		__ms56_CalculateTemperatureCx10();
		__ms56_TriggerPressureSample(ms56dev);
		HAL_Delay(MS5611_SAMPLE_PERIOD_MS);
		D1_ = __ms56_ReadPressureSample(ms56dev);
		pa = __ms56_CalculatePressure_hPa();

		pAccum += pa;
		tAccum += tempCx100_;
		if (avgAltFlag) {
#ifdef USE_LUT
		zAccum += __ms56_hPa2Cm(pa);
//#else
//			zAccum += __getAlt(pa, tAccum, ALTITUDE_REFERENCE_PRESSURE);
#endif
		}
	}
	tc = (float) tAccum / nSamples;
	ms56dev->Temp_C = (float) (tc >= 0 ? (tc + 50) / 100 : (tc - 50) / 100);
	ms56dev->avgPres_hpa = (float) (pAccum) / nSamples;

	if(avgAltFlag)
		zAccum /= nSamples; //compute avg altitude


#ifdef DEBUG_MS56
printf("Tavg : %.2f C\t",ms56dev->Temp_C);
printf("Pavg : %f hPa\t",(int)ms56dev->avgPres_hpa);
printf("alt : %f cm\t",(int)ms56dev->Altitude_cm);
if(avgAltFlag)
	printf("altavg : %f cm\n",ms56dev->avgAltitude_cm);
else
	printf("\n");
#endif

}

float __getAlt(float P, float celsiusSample, float AltRef) {
return (145366.45*(1-pow((P/AltRef),0.190284)) * 0.3048);
}
/// Fast Lookup+Interpolation method for converting pressure readings to altitude readings.
#include "Pressure/pztbl.txt"

float __ms56_hPa2Cm(float paf) {
	int32_t pa, inx, pa1, z1, z2;
	float zf;
	pa = (int32_t) (paf * 10);

	if (pa > PA_INIT) {
		zf = (float) (gPZTbl[0]);
	} else {
		inx = (PA_INIT - pa) >> 10;
		if (inx >= PZLUT_ENTRIES - 1) {
			zf = (float) (gPZTbl[PZLUT_ENTRIES - 1]);
		} else {
			pa1 = PA_INIT - (inx << 10);
			z1 = gPZTbl[inx];
			z2 = gPZTbl[inx + 1];
			zf = (float) (z1)
					+ (((float) (pa1) - paf) * (float) (z2 - z1)) / 1024.0f;
		}
	}
	return zf;
}

void __ms56_CalculateTemperatureCx10(void) {
	dT_ = D2_ - tref_;
	tempCx100_ = 2000 + (((int64_t) dT_ * (cal_[5])) >> 23);
}

float __ms56_CalculatePressure_hPa(void) {
	int32_t pa, t2;
	int64_t offset, sens, offset2, sens2;
	offset = offT1_ + ((((int64_t) cal_[3]) * dT_) >> 7);
	sens = (int64_t) sensT1_ + ((((int64_t) cal_[2]) * dT_) >> 8);
	if (tempCx100_ < 2000) {
		t2 = ((dT_ * dT_) >> 31);
		offset2 = (5 * (tempCx100_ - 2000) * (tempCx100_ - 2000)) / 2;
		sens2 = offset2 / 2;
	} else {
		t2 = 0;
		sens2 = 0;
		offset2 = 0;
	}
	tempCx100_ -= t2;
	offset -= offset2;
	sens -= sens2;

	pa = ((D1_ * sens) / 2097152 - (offset)) / 32768;
	return (float) pa / 100.0;
}

HAL_StatusTypeDef __ms56_ReadCoefficients(volatile MS5611_typedef *ms56dev) {
	uint8_t cnt;
	uint8_t buf[2];
	HAL_StatusTypeDef state = HAL_OK;
	for (cnt = 0; cnt < 6; cnt++) {
		state |= HAL_I2C_Mem_Read(ms56dev->I2Cdev, I2C_ID_MS5611,
				(0xA2 + cnt * 2),
				I2C_MEMADD_SIZE_8BIT, buf, I2C_MEMADD_SIZE_16BIT,
				SENSOR_TIMEOUT);
		cal_[cnt] = (((uint16_t) buf[0]) << 8) | (uint16_t) buf[1];
	}
	return state;
}

/// Trigger a pressure sample with max oversampling rate
void __ms56_TriggerPressureSample(volatile MS5611_typedef *ms56dev) {
	// i2c_XmtData(I2C_ID_MS5611, 0x48);
	uint8_t ads = 0x44;
	HAL_I2C_Master_Transmit(ms56dev->I2Cdev, I2C_ID_MS5611, &ads,
			I2C_MEMADD_SIZE_8BIT,
			SENSOR_TIMEOUT);

}

/// Trigger a temperature sample with max oversampling rate
void __ms56_TriggerTemperatureSample(volatile MS5611_typedef *ms56dev) {
	uint8_t ads = 0x54;
	HAL_I2C_Master_Transmit(ms56dev->I2Cdev, I2C_ID_MS5611, &ads,
			I2C_MEMADD_SIZE_8BIT,
			SENSOR_TIMEOUT);
}

uint32_t __ms56_ReadTemperatureSample(volatile MS5611_typedef *ms56dev) {
	uint32_t w;
	uint8_t buf[3];
	HAL_I2C_Mem_Read(ms56dev->I2Cdev, I2C_ID_MS5611, 0x00, I2C_MEMADD_SIZE_8BIT,
			buf, 3,
			SENSOR_TIMEOUT);
	w = (((uint32_t) buf[0]) << 16) | (((uint32_t) buf[1]) << 8)
			| (uint32_t) buf[2];
	return w;
}

uint32_t __ms56_ReadPressureSample(volatile MS5611_typedef *ms56dev) {
	uint32_t w;
	uint8_t buf[3];
	HAL_I2C_Mem_Read(ms56dev->I2Cdev, I2C_ID_MS5611, 0x00, I2C_MEMADD_SIZE_8BIT,
			buf, 3,
			SENSOR_TIMEOUT);
	w = (((uint32_t) buf[0]) << 16) | (((uint32_t) buf[1]) << 8)
			| (uint32_t) buf[2];
	return w;
}

void MS5611_InitializeSampleStateMachine(volatile MS5611_typedef *ms56dev) {
	__ms56_TriggerTemperatureSample(ms56dev);
	ms56dev->readingState = MS5611_Temp_Read;
}

MS5611_State MS5611_SampleStateMachine(volatile MS5611_typedef *ms56dev) {
	if (ms56dev->readingState == MS5611_Temp_Read) {
		D2_ = __ms56_ReadTemperatureSample(ms56dev);
		__ms56_TriggerPressureSample(ms56dev);
		__ms56_CalculateTemperatureCx10();
		ms56dev->Temp_C = (float) (
				tempCx100_ >= 0 ?
						(tempCx100_ + 50) / 100 : (tempCx100_ - 50) / 100);
		ms56dev->readingState = MS5611_Pressure_Read;
		return MS5611_TEMP_DATA_READY;
	} else if (ms56dev->readingState == MS5611_Pressure_Read) {
		D1_ = __ms56_ReadPressureSample(ms56dev);
		ms56dev->Pres_hpa = __ms56_CalculatePressure_hPa();

		__ms56_TriggerTemperatureSample(ms56dev);
		ms56dev->readingState = MS5611_Temp_Read;
		return MS5611_PRES_DATA_READY;
	}
	return MS5611_SENSOR_ERROR;
}
