/*
 * common.h
 *
 *  Created on: Jul 1, 2022
 *      Author: Naim
 */

#ifndef INC_COMMON_COMMON_H_
#define INC_COMMON_COMMON_H_

//#include "cmsis_os.h"
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "quadspi.h"
#include "rtc.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include "stdlib.h"
#include <string.h>
#include "ctype.h"
#include "userDef/userDef.h"
#include "IMU/mpu9255.h"
#include "Pressure/ms5611.h"
#include "LCD/i2c-lcd.h"
#include "GPS/lwgps.h"
#include "Kalman/kalmanfilter4d.h"
#include "RingBuffer/ringbuf.h"
#include "Lora/LoRa.h"
#include "Hardwares.h"
#include "w25q/w25qxx_qspi.h"
#include "flags.h"
#include "file_handling.h"
#include "music/music.h"

#define IMU_I2C 	(&hi2c2)
#define BARO_I2C 	(&hi2c2)
#define GPS_UART	(&huart2)
#define LORA_UART	(&huart6)
#define ESP_UART	(&huart3)
#define PC_UART		(&huart1)
#define VBAT_ADC	(&hadc3)
#define UV_ADC		(&hadc2)

#define ESC_HTIM	(htim1)
#define ESC_TIM	(TIM1)
#define ESC1_CHN	(TIM_CHANNEL_1)
#define ESC1_DUTY	(ESC_TIM->CCR1)
#define ESC2_CHN	(TIM_CHANNEL_2)
#define ESC2_DUTY	(ESC_TIM->CCR2)

#define SERVO_HTIM	(htim3)
#define SERVO_TIM	(TIM3)
#define SERVO1_CHN	(TIM_CHANNEL_1)
#define SERVO2_CHN	(TIM_CHANNEL_2)
#define SERVO1_DUTY	(SERVO_TIM->CCR1)
#define SERVO2_DUTY	(SERVO_TIM->CCR2)

#define BUZZER_HTIM	(htim2)
#define BUZZER_TIM	(TIM2)
#define BUZZER_CHN	(TIM_CHANNEL_1)
#define BUZZER_DUTY	(BUZZER_TIM->CCR1)

#define PWM_MAX 4500
#define PWM_MIN 2450
#define PWM_ARM 2300
#define PWM_DISARM 2100

#define FLASH_ADD_STARTING_FOR_PACKET 0
#define FLASH_SIZE_OF_PACKET_BYTE 10
#define FLASH_ADD_STARTING_FOR_MAG (FLASH_SIZE_OF_PACKET_BYTE + 1)
#define FLASH_SIZE_OF_MAG_BYTE  60
//#define FLASH_ADD_STARTING_FOR_FLIGHT_STATE ( FLASH_ADD_STARTING_FOR_MAG + FLASH_SIZE_OF_FLIGHT_STATE_BYTE + 1)
//#define FLASH_SIZE_OF_FLIGHT_STATE_BYTE  10

#define IS_IN_RANGE(x,r1,r2) (x>r2 && x<r1)

//float V_SENSOR_MULTIPLIER = 11.299539171;	//Voltage sensor resistor ratio
#define AVO_READ 12.4f						//Manual reading for right calibration
extern float tempBat, tempAdc;
extern float V_SENSOR_MULTIPLIER;

extern RINGBUF BaroRingBuf, AccRingBuf;

#define RxBuf_SIZE   3
#define MainBuf_SIZE 40
extern uint8_t RxBuf[RxBuf_SIZE];
extern uint8_t MainBuf[MainBuf_SIZE];
/*************QUADSPI FLASH VARS****************/
extern char TEXT_Buffer[128];
#define SIZE sizeof(TEXT_Buffer)
extern char *datatemp;
/**********************************************/
#define MINMEA_MAX_LENGTH 128
#define ESP32_MAX_LENGTH 150
#define LORA_MAX_LENGTH 30 //(MAX_SIZE_TX_PACKET)
#define FILE_NAME ("LOG.csv")
extern uint8_t GPS_line[MINMEA_MAX_LENGTH + 1];
extern uint8_t ESP_line[ESP32_MAX_LENGTH + 1];
extern uint8_t LORA_line[LORA_MAX_LENGTH + 1];

extern uint8_t gps_rx_i, gps_rx_d, esp_rx_i, esp_rx_d, lora_rx_i,
		lora_rx_d, gpsFlag, espFlag, loraFlag, gpsGroundAltFlag;

extern uint32_t clockNow, clockPrevious;

extern osTimerId_t TimeoutTimerHandle;
extern osTimerId_t AskiTimerHandle;
extern osTimerId_t TelemetriTimerHandle;

extern osThreadId_t SensorsTaskHandle;
extern osThreadId_t MotorTaskHandle;
extern osThreadId_t UartLoraHandle;
extern osThreadId_t UartEspHandle;

extern osSemaphoreId_t uartLoraSemaHandle;
extern FIL fileSD;
extern FATFS fatfsSD;
#endif /* INC_COMMON_COMMON_H_ */
