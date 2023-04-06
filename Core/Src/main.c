/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
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
#include "string.h"
#include "userDef/userDef.h"
#include <math.h>
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t gps_rx_i = 0, gps_rx_d = 0, esp_rx_i = 0, esp_rx_d = 0, lora_rx_i = 0,
		lora_rx_d = 0, gpsFlag = 0, espFlag = 0, loraFlag = 0,
		gpsGroundAltFlag = 0;

ModelUydu_typedef siracUydu = { .ArmDurumu = DISARM, .PilGerilimi = 0.0f,
		.ayrilmaDurumu = AYRILMADI, .donanimDurumu = 1023, .motorKontrolDurumu =
				DEVRE_DISI, .uyduDurumu = BEKLEME };
TimersFlag_typedef timersFlag;

//uint8_t GPS_line[MINMEA_MAX_LENGTH + 1];
uint8_t ESP_line[ESP32_MAX_LENGTH + 1];
uint8_t LORA_line[LORA_MAX_LENGTH + 1];

UartFlags_typedef uartFlag;

FIL fileSD;
FATFS fatfsSD;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, 100);
	return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (huart == &huart3) {
		if (esp_rx_d != '\n' && esp_rx_i < sizeof(ESP_line)) //Read until end of senctence
				{
			ESP_line[esp_rx_i++] = esp_rx_d;
		} else {
			/*******Reset variables*********/
			//SET_FLAG(uartFlag, UART_ESP);
		}
		HAL_UART_Receive_IT(&huart3, &esp_rx_d, 1);	//Receive GPS char on every interrupt
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (huart == LORA_UART) {
		HAL_UART_DMAStop(LORA_UART);
		SET_FLAG(uartFlag, UART_LORA);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

void enableINT() {
	HAL_UARTEx_ReceiveToIdle_DMA(LORA_UART, LORA_line, LORA_MAX_LENGTH);
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
	/*
	 HAL_UARTEx_ReceiveToIdle_DMA(ESP_UART, ESP_line, ESP32_MAX_LENGTH);
	 __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	 */
	HAL_UART_Receive_IT(&huart3, &esp_rx_d, 1); //Enable UART INT for gps
}

void enableHardware() {
	enableBaro();
	enableFlash();
//EraseFlash(1);
	enableSD();
	getPacketNum();
	getFlightState();
	enableGPS();
	enableIMU();
	enableUV();
	enableINT();
	enableTelemetry();
	vbat_ADC_Calibrate();
	kalmanFilter4d_configure(KF_ACCELBIAS_VARIANCE, KF_ADAPT,
			siracUydu.sensorVerisi.irtifHizVerisi.Altitude_cm, 0.0f, 0.0f);
//check ground station;

	siracUydu.ESP32Durumu.WifiSinyalGucu = 0;
	siracUydu.ESP32Durumu.videoPaketBilgisi.AktarilmaBilgisi =
	VIDEO_AKTARILAMADI;
	siracUydu.ESP32Durumu.videoPaketBilgisi.DosyaBoyutu = 0;

	siracUydu.sensorVerisi.kararAraliklari.BeklemedeYukseklik_max = 5;

	siracUydu.sensorVerisi.kararAraliklari.InisHizi_max =
	ASKIDAN_ONCE_INIS_HIZI_MAX;
	siracUydu.sensorVerisi.kararAraliklari.InisHizi_min =
	ASKIDAN_ONCE_INIS_HIZI_MIN;

	siracUydu.sensorVerisi.kararAraliklari.PilGerilimi_max = 4.2 * 3;
	siracUydu.sensorVerisi.kararAraliklari.PilGerilimi_min = 2.7 * 3;

	siracUydu.telemetri.GY_telemetriVeriDurumu = TELEMETRI_VERISI_GEREKLI;
	strcpy(siracUydu.telemetri.tasiyiciTelemetri.gps2,
			"0.00000000,0.00000000,0.0");
	siracUydu.telemetri.tasiyiciTelemetri.YerYukseklik = 0.0f;
	siracUydu.telemetri.tasiyiciTelemetri.detect = 0;
}

void checkSensorHealth() {
	if (siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m
			< SAGLIKLI_YUKSEKLIK_MIN
			|| siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m
					> SAGLIKLI_YUKSEKLIK_MAX)
		SET_SENS_ERR(siracUydu.sensorVerisi.sensorVeriSagligi,
				yukseklik_aralik_disinda);
	if (siracUydu.sensorVerisi.baro.Pres_hpa < SAGLIKLI_BASINC_MIN
			|| siracUydu.sensorVerisi.baro.Pres_hpa > SAGLIKLI_BASINC_MAX)
		SET_SENS_ERR(siracUydu.sensorVerisi.sensorVeriSagligi,
				basinc_aralik_disinda);
	if (siracUydu.sensorVerisi.irtifHizVerisi.KFClimbrateMps
			< SAGLIKLI_DIKEYHIZ_MIN
			|| siracUydu.sensorVerisi.irtifHizVerisi.KFClimbrateMps
					> SAGLIKLI_DIKEYHIZ_MAX)
		SET_SENS_ERR(siracUydu.sensorVerisi.sensorVeriSagligi,
				dikeyHiz_aralik_disinda);
	if (siracUydu.sensorVerisi.imu.pitch < SAGLIKLI_PITCH_MIN
			|| siracUydu.sensorVerisi.imu.pitch > SAGLIKLI_PITCH_MAX)
		SET_SENS_ERR(siracUydu.sensorVerisi.sensorVeriSagligi,
				pitch_aralik_disinda);
	if (siracUydu.sensorVerisi.imu.roll < SAGLIKLI_ROLL_MIN
			|| siracUydu.sensorVerisi.imu.roll > SAGLIKLI_ROLL_MAX)
		SET_SENS_ERR(siracUydu.sensorVerisi.sensorVeriSagligi,
				roll_aralik_disinda);
	if (siracUydu.sensorVerisi.baro.Temp_C < SAGLIKLI_SICAKLIK_MIN
			|| siracUydu.sensorVerisi.baro.Temp_C > SAGLIKLI_SICAKLIK_MAX)
		SET_SENS_ERR(siracUydu.sensorVerisi.sensorVeriSagligi,
				sicaklik_aralik_disinda);
}

void HeartBeat() {
	checkSensorHealth();
	char buff[10];
	snprintf(buff, 10, "%d,%d", siracUydu.donanimDurumu,
			siracUydu.sensorVerisi.sensorVeriSagligi);
	strcat(siracUydu.telemetri.telemetriPaketi, buff);
}

void findFlightState() {
	switch (siracUydu.uyduDurumu) {
	case BEKLEME:
		sprintf(siracUydu.telemetri.uyduStatu, "%s", "Bekleme");
		if (siracUydu.ArmDurumu
				& (siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m
						> YUKSELME_IRTIFA_TETIKLEME
						&& siracUydu.sensorVerisi.irtifHizVerisi.KFClimbrateMps
								> YUKSELME_HIZ_TETIKLEME)) {
			siracUydu.uyduDurumu = YUKSELME;
		}
		break;

	case YUKSELME:
		sprintf(siracUydu.telemetri.uyduStatu, "%s", "Yukselme");

		if (siracUydu.sensorVerisi.irtifHizVerisi.KFClimbrateMps < 0
				&& siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m
						< siracUydu.sensorVerisi.irtifHizVerisi.PrevAltitude_m) {
			siracUydu.uyduDurumu = MU_INIS;
		}

		break;

	case MU_INIS:
		sprintf(siracUydu.telemetri.uyduStatu, "%s", "MUinis");
		if (				siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m<AYRILMA_YUKSEKLIGI_MAX) {
			siracUydu.uyduDurumu = AYRILMA;
			CLEAR_FLAG(timersFlag, AyrilmaTimeoutFlag);
		}
		break;

	case AYRILMA:
		sprintf(siracUydu.telemetri.uyduStatu, "%s", "Ayrilma");
		servoTahrik(SERVO_KAPAT);
		//startTimer
		if (!osTimerIsRunning(TimeoutTimerHandle)) {
			selectedFlag = AyrilmaTimeoutFlag;
			osTimerStart(TimeoutTimerHandle,
					pdMS_TO_TICKS(AYRILMA_SURESI_MAX_MS));
		}
		if (siracUydu.ayrilmaDurumu
				!= AYRILDI&& !CHECK_FLAG(timersFlag,AyrilmaTimeoutFlag)) //timer interrupt ile tetiklenecek
				{
			if (HAL_GPIO_ReadPin(AYRILMA_BTN_GPIO_Port, AYRILMA_BTN_Pin)
					== GPIO_PIN_SET) {
				osTimerStop(TimeoutTimerHandle);
				siracUydu.ayrilmaDurumu = AYRILDI;
				siracUydu.uyduDurumu = GY_INIS1;
			}
		}
		if (CHECK_FLAG(timersFlag, AyrilmaTimeoutFlag)) {
			siracUydu.ayrilmaDurumu = MANUEL_AYRILMA_GEREKLI;
			servoTahrik(SERVO_KAPAT);
			_managedDelay(1000);
			servoTahrik(SERVO_AC);
			CLEAR_FLAG(timersFlag, AyrilmaTimeoutFlag);
			_managedDelay(1000);
			if (HAL_GPIO_ReadPin(AYRILMA_BTN_GPIO_Port, AYRILMA_BTN_Pin)
					== GPIO_PIN_SET) {
				siracUydu.ayrilmaDurumu = AYRILDI;
				siracUydu.uyduDurumu = GY_INIS1;
				CLEAR_FLAG(timersFlag, AyrilmaTimeoutFlag);
			}
		}
		break;

	case GY_INIS1:
		sprintf(siracUydu.telemetri.uyduStatu, "%s", "GYinis");
		siracUydu.sensorVerisi.kararAraliklari.InisHizi_max =
		ASKIDAN_ONCE_INIS_HIZI_MAX;
		siracUydu.sensorVerisi.kararAraliklari.InisHizi_min =
		ASKIDAN_ONCE_INIS_HIZI_MIN;
		siracUydu.sensorVerisi.kararAraliklari.InisHizi =
		ASKIDAN_ONCE_INIS_HIZI;
		HAL_Delay(1000);
		siracUydu.motorKontrolDurumu = OTOMATIK_INIS;

		if (siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m
				<= ASKI_GOREV_BASLAMA_YUKSEKLIGI_MAX) {
			siracUydu.uyduDurumu = ASKIDA;
			CLEAR_FLAG(timersFlag, AskiTimeoutFlag);
			selectedFlag = AskiTimeoutFlag;
			siracUydu.sensorVerisi.kararAraliklari.InisHizi_max =
			ASKIDA_INIS_HIZI_MAX;
			siracUydu.sensorVerisi.kararAraliklari.InisHizi_min =
			ASKIDA_INIS_HIZI_MIN;
			siracUydu.sensorVerisi.kararAraliklari.InisHizi = ASKIDA_INIS_HIZI;
		}
		break;

	case ASKIDA:
		if (!osTimerIsRunning(
				TimeoutTimerHandle) && !CHECK_FLAG(timersFlag, AskiTimeoutFlag)) {
			osTimerStart(TimeoutTimerHandle, pdMS_TO_TICKS(10 * 1000));
		}
		if (CHECK_FLAG(timersFlag, AskiTimeoutFlag)) {
			siracUydu.uyduDurumu = GY_INIS2;
			siracUydu.sensorVerisi.kararAraliklari.InisHizi_max =
			ASKIDAN_SONRA_INIS_HIZI_MAX;
			siracUydu.sensorVerisi.kararAraliklari.InisHizi_min =
			ASKIDAN_SONRA_INIS_HIZI_MIN;
			siracUydu.sensorVerisi.kararAraliklari.InisHizi =
			ASKIDAN_SONRA_INIS_HIZI;
		}
		break;

	case GY_INIS2:
		sprintf(siracUydu.telemetri.uyduStatu, "%s", "GYinis");
		siracUydu.motorKontrolDurumu = OTOMATIK_INIS;
		if (siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m
				< KONTROLLU_INIS_BASLAMA_YUKSEKLIGI) {
			siracUydu.uyduDurumu = KONTROLLU_INIS;
			siracUydu.sensorVerisi.kararAraliklari.InisHizi_max =
			KONTROLLU_INIS_HIZI_MAX;
			siracUydu.sensorVerisi.kararAraliklari.InisHizi_min =
			KONTROLLU_INIS_HIZI_MIN;
			siracUydu.sensorVerisi.kararAraliklari.InisHizi =
			KONTROLLU_INIS_HIZI;
		}
		break;

	case KONTROLLU_INIS:
		sprintf(siracUydu.telemetri.uyduStatu, "%s", "Kinis");
		if (siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m < 5) {
			siracUydu.motorKontrolDurumu = DEVRE_DISI;
			siracUydu.uyduDurumu = KURTARMA;
		}
		break;

	case KURTARMA:
		sprintf(siracUydu.telemetri.uyduStatu, "%s", "Kurtarma");
		selectedFlag = kurtarmaTelemetriTimeoutFlag;
		playRescue();
		if (!osTimerIsRunning(
				TimeoutTimerHandle) && !CHECK_FLAG(timersFlag, kurtarmaTelemetriTimeoutFlag)) {
			osTimerStart(TimeoutTimerHandle, pdMS_TO_TICKS(60 * 1000));
		}
		if (CHECK_FLAG(timersFlag, kurtarmaTelemetriTimeoutFlag)) {
			siracUydu.telemetri.GY_telemetriVeriDurumu =
					TELEMETRI_VERISI_GEREKLI_DEGIL;
			osTimerStop(TelemetriTimerHandle);
		}
		break;
	}
}

void sendTelem() {
	char telemPacket1[95] = { '\0' };
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
	char telemP[250] = { '\0' };
	int siz = 0;
	siz = snprintf((char*) telemPacket1, 90,
			"467710,%d,%02d/%02d/%d %02d:%02d:%02d",
			siracUydu.telemetri.paketNo, date.Date, date.Month, date.Year,
			time.Hours, time.Minutes, time.Seconds);
	LoRaE22_sendStruct((uint8_t*) telemPacket1, siz);

	siz = snprintf((char*) telemPacket1, 90, ",%.2f,%.2f,%.2f,%.2f",
			siracUydu.sensorVerisi.baro.avgPres_hpa * (float) 100.0f,
			siracUydu.telemetri.tasiyiciTelemetri.basinc,
			siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m,
			siracUydu.telemetri.tasiyiciTelemetri.yukseklik);
	LoRaE22_sendStruct((uint8_t*) telemPacket1, siz);

	siz = snprintf((char*) telemPacket1, 90, ",%.2f,%.2f,%.2f,%.1f",
			fabs(
					siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m
							- siracUydu.telemetri.tasiyiciTelemetri.yukseklik),
			siracUydu.sensorVerisi.irtifHizVerisi.KFClimbrateMps,
			siracUydu.sensorVerisi.baro.Temp_C, siracUydu.PilGerilimi);
	LoRaE22_sendStruct((uint8_t*) telemPacket1, siz);

	siz = snprintf((char*) telemPacket1, 90, ",%.8f,%.8f,%.2f",
			siracUydu.sensorVerisi.gps.latitude,
			siracUydu.sensorVerisi.gps.longitude,
			siracUydu.sensorVerisi.gps.altitude);
	LoRaE22_sendStruct((uint8_t*) telemPacket1, siz);

	siz = snprintf((char*) telemPacket1, 90, ",%s,%s",
			siracUydu.telemetri.tasiyiciTelemetri.gps2,
			siracUydu.telemetri.uyduStatu);
	LoRaE22_sendStruct((uint8_t*) telemPacket1, siz);

	siz = snprintf((char*) telemPacket1, 90, ",%.2f,%.2f,%.2f,%d,%s,",
			siracUydu.sensorVerisi.imu.pitch, siracUydu.sensorVerisi.imu.roll,
			siracUydu.sensorVerisi.imu.yaw, siracUydu.telemetri.donusSayisi,
			siracUydu.ESP32Durumu.videoPaketBilgisi.AktarilmaBilgisi ?
					"EVET" : "HAYIR");
	LoRaE22_sendStruct((uint8_t*) telemPacket1, siz);

	siz = snprintf((char*) telemPacket1, 90, "%.2f,%d,%d,%d,%d\n",
			siracUydu.sensorVerisi.UV_sensor,
			siracUydu.ESP32Durumu.WifiSinyalGucu, siracUydu.ArmDurumu,
			siracUydu.donanimDurumu, siracUydu.ayrilmaDurumu);
	LoRaE22_sendStruct((uint8_t*) telemPacket1, siz);

	sprintf(siracUydu.telemetri.telemetriPaketi,
			"467710,%d,%02d/%02d/%d %02d:%02d:%02d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.8f,%.8f,%.2f,%s,%s,%.2f,%.2f,%.2f,%d,%s\n",
			siracUydu.telemetri.paketNo, date.Date, date.Month, date.Year,
			time.Hours, time.Minutes, time.Seconds,
			siracUydu.sensorVerisi.baro.avgPres_hpa * (float) 100.0f,
			siracUydu.telemetri.tasiyiciTelemetri.basinc,
			siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m,
			siracUydu.telemetri.tasiyiciTelemetri.yukseklik,
			fabs(
					siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m
							- siracUydu.telemetri.tasiyiciTelemetri.yukseklik),
			siracUydu.sensorVerisi.irtifHizVerisi.KFClimbrateMps,
			siracUydu.sensorVerisi.baro.Temp_C, siracUydu.PilGerilimi,
			siracUydu.sensorVerisi.gps.latitude,
			siracUydu.sensorVerisi.gps.longitude,
			siracUydu.sensorVerisi.gps.altitude,
			siracUydu.telemetri.tasiyiciTelemetri.gps2,
			siracUydu.telemetri.uyduStatu, siracUydu.sensorVerisi.imu.pitch,
			siracUydu.sensorVerisi.imu.roll, siracUydu.sensorVerisi.imu.yaw,
			siracUydu.telemetri.donusSayisi,
			siracUydu.ESP32Durumu.videoPaketBilgisi.AktarilmaBilgisi ?
					"EVET" : "HAYIR");
	return;
}
uint8_t ilkAcilma = 1;
void saveToSD() {
	uint32_t byteswritten;
	f_open(&fileSD, FILE_NAME, FA_OPEN_APPEND | FA_WRITE);
	if (ilkAcilma) {
		char *a =
				"Takim no,Paket no,Gonderim saati,Basinc1,Basinc2,Yukseklik1,Yukseklik2,Irtifa farki,Inis hizi,Sicaklik,Pil gerilimi,GPS1 Latitude,GPS1 Longitude,GPS1 Altitude,GPS2 Latitude,GPS2 Longitude,GPS2 Altitude,Uydu statusu,Pitch,Roll,Yaw,Donus sayisi,Video aktarimi\n";
		f_write(&fileSD, a, strlen(a), (void*) &byteswritten);
		ilkAcilma = 0;
	}
	f_write(&fileSD, siracUydu.telemetri.telemetriPaketi,
			strlen((char*) siracUydu.telemetri.telemetriPaketi),
			(void*) &byteswritten);
	f_close(&fileSD);
	return;
}

void PID(float measurement, float setpoint, uint8_t dt) {
	float err = setpoint - measurement;

	float proportional = PID_Kp * err;

	PID1.integrator = PID1.integrator
			+ 0.5 * PID_Ki * dt * (err + PID1.prevErr);

	if (PID1.integrator > PID1.limMax) {
		PID1.integrator = PID1.limMax;
	} else if (PID1.integrator < PID1.limMin) {
		PID1.integrator = PID1.limMin;
	}

	PID1.differentiator = -(2.0 * PID_Kd * (measurement - PID1.prevMeas)
			+ (2.0 * PID_tau - dt) * PID1.differentiator)
			/ (2.0 * PID_tau + dt);

	PID1.pwm = proportional + PID1.integrator + PID1.differentiator;

	if (PID1.pwm > PID1.limMax) {
		PID1.pwm = PID1.limMax;
	} else if (PID1.pwm < PID1.limMin) {
		PID1.pwm = PID1.limMin;
	} else {
		PID1.pwm = (int) round(PID1.pwm);
	}

	PID1.prevErr = err;
	PID1.prevMeas = measurement;
	return;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals , Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC3_Init();
	MX_I2C2_Init();
	MX_QUADSPI_Init();
	MX_RTC_Init();
	MX_SDMMC1_SD_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
	MX_FATFS_Init();
	MX_ADC2_Init();
	/* USER CODE BEGIN 2 */
	playStartSystem();
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Macro to configure the PLL clock source
	 */
	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 2;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 6;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC
			| RCC_PERIPHCLK_SDMMC | RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART6;
	PeriphClkInitStruct.PLL2.PLL2M = 2;
	PeriphClkInitStruct.PLL2.PLL2N = 12;
	PeriphClkInitStruct.PLL2.PLL2P = 4;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
	PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
	PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_PLL2;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4KB;
	MPU_InitStruct.SubRegionDisable = 0x0;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM8 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM8) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
