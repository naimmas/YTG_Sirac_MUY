/*
 * Hardwares.h
 *
 *  Created on: Jul 10, 2022
 *      Author: Naim
 */

#ifndef INC_USERDEF_HARDWARES_H_
#define INC_USERDEF_HARDWARES_H_


#define SET_SENS_ERR(errHandle, errFlag) 	(errHandle |= errFlag)
#define CLEAR_SENS_ERR(errHandle, errFlag)	(errHandle &= ~errFlag)
#define CHECK_SENS_ERR(errHandle, errFlag)	(errFlag==(errHandle&errFlag))

#define MOTOR_PWM_CALC(x) ((((x - 0) * (PWM_MAX- PWM_MIN)) / (100 - 0)) + PWM_MIN)

void enableFlash();
void enableSD();
void enableBaro();
void enableIMU();
void enableGPS();
void enableTelemetry();
void enableTimers();
void enableUV();
void setRTC(uint8_t h, uint8_t m, uint8_t s, uint8_t day, uint8_t month,
		uint8_t date);
void set_time(uint8_t h, uint8_t m, uint8_t s, uint8_t day, uint8_t month,
		uint8_t date, uint8_t setTimeFlag);
void EraseFlash(uint8_t withFormat);
void getPacketNum();
void getBatVolt();
void getUVSensor();
void vbat_ADC_Calibrate();
void savePacketNum();
void motorTahrik(uint16_t hiz1, uint16_t hiz2);
void servoTahrik(uint8_t komut);
void ESCcalibration();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

#endif /* INC_USERDEF_HARDWARES_H_ */
