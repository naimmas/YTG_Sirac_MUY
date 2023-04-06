/*
 * Hardwares.c
 *
 *  Created on: Jul 10, 2022
 *      Author: Naim
 */
#include "userDef/userDef.h"

PID_typedef PID1 =
		{ .differentiator = 0.0f, .integrator = 0.0f, .limMax = PID_MAX,
				.limMin = PID_MIN, .prevErr = 0.0f, .prevMeas = 0.0f, .pwm = 0 };
RINGBUF BaroRingBuf = { .buffer = { 0 }, .head = 0 }, AccRingBuf = { .buffer = {
		0 }, .head = 0 }, AltRingBuf = { .buffer = { 0 }, .head = 0 };
float V_SENSOR_MULTIPLIER = 11.039525f;
float tempBat = 0.0f, tempAdc = 0.0f, prevBatV = 0.0f;

/****************************RTC CONFIG*************************************/
void setRTC(uint8_t h, uint8_t m, uint8_t s, uint8_t day, uint8_t month,
		uint8_t date) {
	set_time(h, m, s, day, month, date, 1);
}
void set_time(uint8_t h, uint8_t m, uint8_t s, uint8_t day, uint8_t month,
		uint8_t date, uint8_t setTimeFlag) {
	/*											  *
	 *											  *
	 * ********** RTC Function EDITED *********** *
	 * 											  *
	 *											  */

	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32f2 || setTimeFlag) {
		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;

		/** Initialize RTC and set the Time and Date**/

		sTime.Hours = h;
		sTime.Minutes = m;
		sTime.Seconds = s;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_SET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}
		sDate.WeekDay = day;
		sDate.Month = month;
		sDate.Date = date;
		sDate.Year = 22;

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}
		/* rtc register backup to save rtc settings and not reset on cpu reset */
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32f2);
	}

}

void enableIMU() {
	if (MPU9255_Init(IMU_I2C, 0)) {
#ifdef LCD_EN
		lcd_send_string(0, 0, "IMU ERR", &lcd1);
#endif
		CLEAR_SENS_ERR(siracUydu.donanimDurumu, IMU_ERROR);
		ringbuf_init(&AccRingBuf);
		return;
	}

	//static RINGBUF imuRingBuf;
	SET_SENS_ERR(siracUydu.donanimDurumu, IMU_ERROR);
}

void enableBaro() {
	if (MS5611_Init(&siracUydu.sensorVerisi.baro, BARO_I2C)
			!= MS5611_SENSOR_OK) {
#ifdef LCD_EN
		lcd_send_string(1, 0, "MS5611 ERR", &lcd1);
#endif
		SET_SENS_ERR(siracUydu.donanimDurumu, BARO_ERROR);
		return;
	}

	MS5611_AveragedSample(&siracUydu.sensorVerisi.baro, 100, 0);
	siracUydu.sensorVerisi.irtifHizVerisi.Altitude_cm = __getAlt(
			siracUydu.sensorVerisi.baro.avgPres_hpa,
			siracUydu.sensorVerisi.baro.Temp_C, ALTITUDE_REFERENCE_PRESSURE)
			* 100.0;
	MS5611_InitializeSampleStateMachine(&siracUydu.sensorVerisi.baro);
	siracUydu.sensorVerisi.irtifHizVerisi.GroundAltitude_cm =
			siracUydu.sensorVerisi.irtifHizVerisi.Altitude_cm;
	siracUydu.sensorVerisi.irtifHizVerisi.PrevAltitude_m =
			siracUydu.sensorVerisi.irtifHizVerisi.Altitude_cm * 100.0;
	ringbuf_init(&BaroRingBuf);
	ringbuf_init(&AltRingBuf);
	CLEAR_SENS_ERR(siracUydu.donanimDurumu, BARO_ERROR);
	return;
}

void enableGPS() {
	lwgps_init(&siracUydu.sensorVerisi.gps);
	_managedDelay(200);
	if (siracUydu.sensorVerisi.gps.p.star == 0) {
		SET_SENS_ERR(siracUydu.donanimDurumu, GPS_ERROR);
		return;
	}
	CLEAR_SENS_ERR(siracUydu.donanimDurumu, GPS_ERROR);
}

void enableUV() {
	SET_SENS_ERR(siracUydu.donanimDurumu, UV_ERROR);
	HAL_StatusTypeDef ret = HAL_ADC_Stop(UV_ADC);
	if (ret != HAL_OK) {
		Error_Handler();
	}

	HAL_Delay(10);

	/* Choose type of measurement
	 ADC_SINGLE_ENDED
	 ADC_DIFFERENTIAL_ENDED
	 */
	ret = HAL_ADCEx_Calibration_Start(UV_ADC, ADC_CALIB_OFFSET,
	ADC_SINGLE_ENDED);
	if (ret != HAL_OK) {
		Error_Handler();
	}
	// Small delay to ensure end of calibration procedure
	CLEAR_SENS_ERR(siracUydu.donanimDurumu, UV_ERROR);
	HAL_Delay(100);
	return;
}

void enableFlash() {
	if (w25qxx_Init()) {
		HAL_Delay(5);
		printf("FLASH CHIP ID: %d\n", w25qxx_GetID());
		HAL_Delay(25);
		CLEAR_SENS_ERR(siracUydu.donanimDurumu, FLASH_ERROR);
		return;
	}
	SET_SENS_ERR(siracUydu.donanimDurumu, FLASH_ERROR);
}

void EraseFlash(uint8_t withFormat) {
	if (withFormat) {
		printf("ERASING CHIP AFTER 2Sec...\n");
		HAL_Delay(2000);
		printf("ERASE CHIP VAL: %d\n", W25qxx_EraseChip()); // approx 13 second to execute
	}
	uint8_t datatemp[FLASH_SIZE_OF_PACKET_BYTE];
	sprintf((char*) datatemp, "%d", 0);
	W25qxx_Write(datatemp, FLASH_ADD_STARTING_FOR_PACKET,
	FLASH_SIZE_OF_PACKET_BYTE);
	getPacketNum();
}
void saveFlightState() {
//	uint8_t datatemp[FLASH_SIZE_OF_FLIGHT_STATE_BYTE];
//	sprintf((char*) datatemp, "%d", siracUydu.uyduDurumu);
//	W25qxx_Write(datatemp, FLASH_ADD_STARTING_FOR_FLIGHT_STATE,
//	FLASH_SIZE_OF_FLIGHT_STATE_BYTE);
	return;
}
void getFlightState() {
//	uint8_t recFlash[FLASH_SIZE_OF_FLIGHT_STATE_BYTE];
//	W25qxx_Read(recFlash, FLASH_ADD_STARTING_FOR_FLIGHT_STATE,
//	FLASH_SIZE_OF_FLIGHT_STATE_BYTE);
//	siracUydu.uyduDurumu = atoi((char*) recFlash);
//	if (siracUydu.uyduDurumu > 0) { //printf("%d\n", paket);
//		return;
//	}
	siracUydu.uyduDurumu = BEKLEME;
}
void getPacketNum() {
	uint8_t recFlash[FLASH_SIZE_OF_PACKET_BYTE];
	W25qxx_Read(recFlash, FLASH_ADD_STARTING_FOR_PACKET,
	FLASH_SIZE_OF_PACKET_BYTE);
	siracUydu.telemetri.paketNo = atoi((char*) recFlash);
	if (siracUydu.telemetri.paketNo > 0) { //printf("%d\n", paket);
		return;
	}
	siracUydu.telemetri.paketNo = 1;
}
void savePacketNum() {
	uint8_t datatemp[FLASH_SIZE_OF_PACKET_BYTE];
	sprintf((char*) datatemp, "%d", siracUydu.telemetri.paketNo);
	W25qxx_Write(datatemp, FLASH_ADD_STARTING_FOR_PACKET,
	FLASH_SIZE_OF_PACKET_BYTE);
	siracUydu.telemetri.paketNo++;
}

void enableLCD() {
	lcd1.addrs = 0x38 << 1;

	lcd1.i2c = &hi2c2;

	lcd_init(&lcd1);
	HAL_Delay(20);
	lcd_clear(&lcd1);
	lcd_send_string(0, 0, "YTG-Sirac", &lcd1);
	HAL_Delay(1000);
	lcd_clear(&lcd1);
}

void enableSD() {
	BSP_SD_Init();
	if ((SDMMC_Init(hsd1.Instance, hsd1.Init) != HAL_OK) | (!mount_sd())) {
		printf("SDMMC ERROR.");
		SET_SENS_ERR(siracUydu.donanimDurumu, SD_ERROR);
		return;
	}
	if (check_file(FILE_NAME) == FILE_EXIST) {
		remove_file(FILE_NAME);
	}
	if (create_file(FILE_NAME) != FILE_CREATED) {
		SET_SENS_ERR(siracUydu.donanimDurumu, SD_ERROR);
		return;
	} else {
		if (write_file(FILE_NAME, "") == FILE_WRITTEN) {
			CLEAR_SENS_ERR(siracUydu.donanimDurumu, SD_ERROR);
			return;
		}
	}
}

void enableTelemetry() {
	LoRaE22_Init(&huart6, LORA_AUX_GPIO_Port, LORA_M0_GPIO_Port,
	LORA_M1_GPIO_Port, LORA_AUX_Pin, LORA_M0_Pin, LORA_M1_Pin);
	return;
}
#define VBAT_IIR 95

void getBatVolt() {
	HAL_ADC_Start(VBAT_ADC);
	HAL_ADC_PollForConversion(VBAT_ADC, 1000);
	float adcVal = 0.0;
	for (int i = 0; i < 20; i++) {
		adcVal += (((float) HAL_ADC_GetValue(VBAT_ADC)) * 3.295 / 65535);
	}
	HAL_ADC_Stop(VBAT_ADC);
	adcVal /= 20;
	float BatVal = (adcVal * V_SENSOR_MULTIPLIER);

	/********LOW PASS FILTER 1st***********/
	//vBat = 0.99023048 * tempBat + 0.00488476 * BatVal + 0.00488476 * prevBatV;
	siracUydu.PilGerilimi = (tempBat * (float) VBAT_IIR
			+ BatVal * (100.0f - (float) VBAT_IIR)) / 100.0f;
	tempBat = siracUydu.PilGerilimi;
	return;
}

void getUVSensor() {
	HAL_ADC_Start(UV_ADC);
	HAL_ADC_PollForConversion(UV_ADC, 1000);
	float adcVal = 0.0;
	for (int i = 0; i < 20; i++) {
		adcVal += (((float) HAL_ADC_GetValue(UV_ADC)) * 3.295 / 65535);
	}
	HAL_ADC_Stop(UV_ADC);
	adcVal /= 20;
	siracUydu.sensorVerisi.UV_sensor = mapfloat(adcVal, 0.99, 2.8, 0.0, 15.0);
	/***
	 * geriye kalan hesaplamalar eklenece
	 */
}
float mapfloat(float x, float in_min, float in_max, float out_min,
		float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void vbat_ADC_Calibrate() {
	// Make sure ADC peripheral is stopped
	HAL_StatusTypeDef ret = HAL_ADC_Stop(VBAT_ADC);
	if (ret != HAL_OK) {
		Error_Handler();
	}

	HAL_Delay(10);

	/* Choose type of measurement
	 ADC_SINGLE_ENDED
	 ADC_DIFFERENTIAL_ENDED
	 */
	ret = HAL_ADCEx_Calibration_Start(VBAT_ADC, ADC_CALIB_OFFSET,
	ADC_SINGLE_ENDED);
	if (ret != HAL_OK) {
		Error_Handler();
	}

	// Small delay to ensure end of calibration procedure
	HAL_Delay(100);

	/****************Average for best result****************/
	float t;

	for (uint8_t i = 0; i < 100; i++) {
		HAL_ADC_Start(VBAT_ADC);
		HAL_ADC_PollForConversion(VBAT_ADC, HAL_MAX_DELAY);
		HAL_ADC_GetValue(VBAT_ADC);
		HAL_ADC_Stop(VBAT_ADC);
	}

	for (uint8_t i = 0; i < 150; i++) {
		HAL_ADC_Start(VBAT_ADC);
		HAL_ADC_PollForConversion(VBAT_ADC, HAL_MAX_DELAY);
		t = (((float) HAL_ADC_GetValue(VBAT_ADC)) * 3.295 / 65535);
		tempAdc += t;
		HAL_ADC_Stop(VBAT_ADC);
		HAL_Delay(10);
	}
	tempAdc /= 150;
	//V_SENSOR_MULTIPLIER = AVO_READ / tempAdc;
	return;
}
void motorTahrik(uint16_t hiz1, uint16_t hiz2) {
	ESC1_DUTY = hiz1;
	ESC2_DUTY = hiz2;
}

void servoTahrik(uint8_t komut) {
	switch (komut) {
	case SERVO_AC:
		SERVO2_DUTY = 950;
		return;
	case SERVO_KAPAT:
		SERVO2_DUTY = 1100;
		return;
	}
}

void ESCcalibration() {
	ESC1_DUTY = PWM_MAX;
	ESC2_DUTY = PWM_MAX;
	osDelay(10000);
	ESC1_DUTY = PWM_MIN;
	ESC2_DUTY = PWM_MIN;
	osDelay(2000);
}
