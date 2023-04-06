//
// Created by ajanx on 15.05.2022.
//

#include "userDef/userDef.h"

struct LoRaInitConfig initConfig;
LoRaTypedef_MODES _mode = E22_NORMAL_MODE;

void close(void *mem) {
	free(mem);
}

void LoRaE22_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *AuxPort,
		GPIO_TypeDef *M0Port, GPIO_TypeDef *M1Port, uint16_t AuxPin,
		uint16_t M0Pin, uint16_t M1Pin) {
	initConfig.uartDevice = huart;
	initConfig.AUX_Port = AuxPort;
	initConfig.AUX_Pin = AuxPin;
	initConfig.M0_Port = M0Port;
	initConfig.M0_Pin = M0Pin;
	initConfig.M1_Port = M1Port;
	initConfig.M1_Pin = M1Pin;
	LoRaE22_SetMode(E22_NORMAL_MODE);
}

LoRaTypedef_STATUS LoRaE22_SetMode(LoRaTypedef_MODES mode) {
	_managedDelay(10);

	switch (mode) {
	case E22_NORMAL_MODE:
		// Mode 0 | normal operation
		HAL_GPIO_WritePin(initConfig.M0_Port, initConfig.M0_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(initConfig.M1_Port, initConfig.M1_Pin,
				GPIO_PIN_RESET);
		break;
	case E22_WOR_MODE:
		HAL_GPIO_WritePin(initConfig.M0_Port, initConfig.M0_Pin, 1);
		HAL_GPIO_WritePin(initConfig.M1_Port, initConfig.M1_Pin, 0);
		break;
	case E22_CONFIG_MODE:
		//		  case MODE_2_PROGRAM:
		HAL_GPIO_WritePin(initConfig.M0_Port, initConfig.M0_Pin, 0);
		HAL_GPIO_WritePin(initConfig.M1_Port, initConfig.M1_Pin, 1);
		break;
	case E22_SLEEP_MODE:
		// Mode 3 | Setting operation
		HAL_GPIO_WritePin(initConfig.M0_Port, initConfig.M0_Pin, 1);
		HAL_GPIO_WritePin(initConfig.M1_Port, initConfig.M1_Pin, 1);
		break;

	default:
		return ERR_E22_INVALID_PARAM;
	}

	// data sheet says 2ms later control is returned, let's give just a bit more time
	// these modules can take time to activate pins
	_managedDelay(3);

	// wait until aux pin goes back low
	LoRaTypedef_STATUS res = LoRaE22_waitCompleteResponse(1000);

	return res;
}

LoRaTypedef_STATUS LoRaE22_waitCompleteResponse(unsigned long timeout) {

	LoRaTypedef_STATUS result = E22_SUCCESS;

	unsigned long t = HAL_GetTick();

	if (((unsigned long) (t + timeout)) == 0) {
		t = 0;
	}
	while (HAL_GPIO_ReadPin(initConfig.AUX_Port, initConfig.AUX_Pin)
			== GPIO_PIN_RESET) {
		if ((HAL_GetTick() - t) > timeout) {
			result = ERR_E22_TIMEOUT;

			return result;
		}
	}

	_managedDelay(5);
	return result;
}

void _managedDelay(unsigned long timeout) {
	unsigned long t = HAL_GetTick();
	if (((unsigned long) (t + timeout)) == 0) {
		t = 0;
	}
	while ((HAL_GetTick() - t) < timeout)
		;
}

LoRaTypedef_STATUS LoRaE22_sendStruct(uint8_t *structureM, uint16_t size) {
	if (size > MAX_SIZE_TX_PACKET + 2) {
		return ERR_E22_PACKET_TOO_BIG;
	}
	uint8_t header[3] = { 0x05, 0x31, 0x12 };

	HAL_UART_Transmit(LORA_UART, header, 3, 100);
	if (HAL_UART_Transmit_DMA(initConfig.uartDevice, structureM, size)
			!= HAL_OK) {
		return ERR_E22_NO_RESPONSE_FROM_DEVICE;
	}
	HAL_Delay(50);
	return LoRaE22_waitCompleteResponse(RESPONSE_DELAY);
}

LoRaTypedef_STATUS LoRaE22_receiveStruct(uint8_t *structureM, uint16_t size,
		uint8_t RSSI, uint8_t *Vrssi) {
	if (HAL_UART_Receive(initConfig.uartDevice, structureM, MAX_SIZE_TX_PACKET,
	UART_DELAY) != HAL_OK)
		return ERR_E22_NO_RESPONSE_FROM_DEVICE;

	LoRaE22_waitCompleteResponse(RESPONSE_DELAY);
	if (RSSI) {
		uint8_t rssi[1];
		HAL_UART_Receive(initConfig.uartDevice, rssi, 1, UART_DELAY);
		Vrssi = rssi;
	}
	return LoRaE22_waitCompleteResponse(RESPONSE_DELAY);

}

uint32_t parseNumber(uint8_t *msj, uint8_t len) {
	uint8_t *h = msj;
	uint8_t m[len];
	uint8_t *a = m;
	memset(m, 0, len * sizeof(uint8_t));
	while (*h != '\0' && isdigit(*h)) {
		(*a) = *h++;
		a++;
	}
	return (atoi((char*) m));
}

uint8_t parseMotor(void *d, uint8_t len) {
	uint8_t *p = d;
	char *token1;
	char *s1 = " ";
	token1 = strtok((char*) p, s1);
	if (token1 == NULL)
		return 0;
	siracUydu.yerIstKomutu.motorTahrikGucu = parseNumber((uint8_t*) token1,
			strlen(token1));
	token1 = strtok(NULL, s1);
	if (token1 == NULL)
		return 0;
	siracUydu.yerIstKomutu.motorTahrikSuresi = parseNumber((uint8_t*) token1,
			strlen(token1));
	return 1;

}

void parseSetDate(void *d, uint8_t len) {
	uint8_t *p = d;
	char *token1, *token2;
	char *timeData;
	char *dateData;
	char *s1 = " ";
	char *s2 = ":";
	uint8_t h, m, s;
	uint8_t day, month;

	timeData = strtok((char*) p, s1);
	if (timeData == NULL)
		return;
	dateData = strtok(NULL, s1);
	if (dateData == NULL)
		return;
	token1 = strtok((char*) timeData, s2);

	h = parseNumber((uint8_t*) token1, strlen(token1));
	token1 = strtok((char*) NULL, s2);
	if (token1 == NULL)
		return;
	m = parseNumber((uint8_t*) token1, strlen(token1));
	token1 = strtok((char*) NULL, s2);
	if (token1 == NULL)
		return;
	s = parseNumber((uint8_t*) token1, strlen(token1));

	token2 = strtok((char*) dateData, s2);
	day = parseNumber((uint8_t*) token2, strlen(token2));
	token2 = strtok((char*) NULL, s2);
	if (token1 == NULL)
		return;
	month = parseNumber((uint8_t*) token2, strlen(token2));
	token2 = strtok((char*) NULL, s2);
	if (token1 == NULL)
		return;
	setRTC(h, m, s, 1, month, day);
	return;
}
void parseTData(void *p) {
	uint8_t *h = p;
	switch (*h++) {
	case 'g': {
		char *p = h;
		char *s = ",";
		char *token;
		token = strtok(p, s);
		strcpy(siracUydu.telemetri.tasiyiciTelemetri.gps2, token);
		token = strtok(NULL, s);
		strcat(siracUydu.telemetri.tasiyiciTelemetri.gps2, ",");
		strcat(siracUydu.telemetri.tasiyiciTelemetri.gps2, token);
		token = strtok(NULL, s);
		strcat(siracUydu.telemetri.tasiyiciTelemetri.gps2, ",");
		strcat(siracUydu.telemetri.tasiyiciTelemetri.gps2, token);
		token = strtok(NULL, s);
		siracUydu.telemetri.tasiyiciTelemetri.basinc = atof(token) * 100.0;
		if (siracUydu.telemetri.tasiyiciTelemetri.basinc >= 70000.0) {
			siracUydu.telemetri.tasiyiciTelemetri.yukseklik = __getAlt(
					siracUydu.telemetri.tasiyiciTelemetri.basinc / 100.0,
					siracUydu.sensorVerisi.baro.Temp_C,
					ALTITUDE_REFERENCE_PRESSURE)
					- siracUydu.telemetri.tasiyiciTelemetri.YerYukseklik;
			if (siracUydu.telemetri.tasiyiciTelemetri.detect <= 3) {
				siracUydu.telemetri.tasiyiciTelemetri.YerYukseklik +=
						siracUydu.telemetri.tasiyiciTelemetri.yukseklik;
				siracUydu.telemetri.tasiyiciTelemetri.detect++;
			} else if (siracUydu.telemetri.tasiyiciTelemetri.detect == 3) {
				siracUydu.telemetri.tasiyiciTelemetri.YerYukseklik /= 3;
				siracUydu.telemetri.tasiyiciTelemetri.detect++;
			}
		} else
			siracUydu.telemetri.tasiyiciTelemetri.yukseklik = 0.0;

		break;
	}
	default:
		break;
	}
}

void parseTError(void *d) {
	char *p = d;
	char *s = ",";
	char *token;
	token = strtok(p, s);
	int err = atoi(token);
	siracUydu.donanimDurumu |= err;
	return;
}

void parseGSCommand(void *p, uint8_t len) {
	if (len == 0) {
		return;
	}
	uint8_t *h = p;

	switch (*h++) {
	case 's': 	//seperate
		if (siracUydu.uyduDurumu == BEKLEME) {
			servoTahrik(siracUydu.yerIstKomutu.servoKomutu);
			siracUydu.yerIstKomutu.servoKomutu =
					!siracUydu.yerIstKomutu.servoKomutu;
		} else if (siracUydu.ayrilmaDurumu == MANUEL_AYRILMA_GEREKLI) {
			servoTahrik(SERVO_AC);
		}
		break;

	case 'm': 	//manuel motor
		if (siracUydu.uyduDurumu == BEKLEME) {
			if (parseMotor(h, strlen((char*) h))) {
				siracUydu.motorKontrolDurumu = MANUEL;
				CLEAR_FLAG(timersFlag, motorTahrikTimeoutFlag);
			}
		}
		break;

	case 'c': 	//imu calibration
		if (siracUydu.uyduDurumu == BEKLEME) {
			MPU9255_Init(IMU_I2C, 1);
			//printf("imu kalibrasyon\n");
		}
		break;

	case 'p':	//package reset
		if (siracUydu.uyduDurumu == BEKLEME) {
			//printf("paket no reset\n");
			EraseFlash(0);
		}
		break;

	case 't':	//set time
		parseSetDate(h, strlen((char*) h));
		break;

	case 'a': //reset relative altitude
		siracUydu.sensorVerisi.irtifHizVerisi.GroundAltitude_cm =
				siracUydu.sensorVerisi.irtifHizVerisi.Altitude_cm;
		break;
	case 'd': //motors disarm
		HAL_TIM_PWM_Stop(&ESC_HTIM, ESC1_CHN);
		HAL_TIM_PWM_Stop(&ESC_HTIM, ESC2_CHN);
		siracUydu.ArmDurumu = DISARM;
		siracUydu.motorKontrolDurumu = DEVRE_DISI;
		break;

	case 'q': //motors arm
		HAL_TIM_PWM_Start(&ESC_HTIM, ESC1_CHN);
		HAL_TIM_PWM_Start(&ESC_HTIM, ESC2_CHN);
		siracUydu.ArmDurumu = ARM;
		break;
	case 'w': //esc calibration
		ESCcalibration();
		break;
	case 'e': {
		siracUydu.uyduDurumu = AYRILMA;
	}
		break;
	case 'f':
		siracUydu.uyduDurumu = BEKLEME;
		saveFlightState();
		break;
	default:
		break;
	}

}

void parseESP(void *p, uint8_t len) {
	if (len == 0) {
		return;
	}
	char *h = p;
	switch (*h++) {
	case 'g': {
		char *p = h;
		char *s = ",";
		char *token;
		token = strtok(p, s);
		siracUydu.sensorVerisi.gps.sats_in_view = atoi(token);
		if (siracUydu.sensorVerisi.gps.sats_in_view >= 4) {
			CLEAR_SENS_ERR(siracUydu.donanimDurumu, GPS_NOFIX);
		} else {
			SET_SENS_ERR(siracUydu.donanimDurumu, GPS_NOFIX);
		}
		token = strtok(NULL, s);
		siracUydu.sensorVerisi.gps.latitude = atof(token);
		if (siracUydu.sensorVerisi.gps.latitude > 0.0) {
			CLEAR_SENS_ERR(siracUydu.donanimDurumu, GPS_ERROR);
		} else {
			SET_SENS_ERR(siracUydu.donanimDurumu, GPS_ERROR);
		}
		token = strtok(NULL, s);
		siracUydu.sensorVerisi.gps.longitude = atof(token);
		token = strtok(NULL, s);
		siracUydu.sensorVerisi.gps.altitude = atof(token);
		break;
	}
	case 'w': {
		char *p = h;
		char *s = ",";
		char *token;
		token = strtok(p, s);
		siracUydu.ESP32Durumu.videoPaketBilgisi.AktarilmaBilgisi = atoi(token);
		token = strtok(NULL, s);
		if (atoi(token)) {
			CLEAR_SENS_ERR(siracUydu.donanimDurumu, ESP32_WIFI_ERR);
			CLEAR_SENS_ERR(siracUydu.donanimDurumu, ESP32_SD_ERROR);
		} else
			SET_SENS_ERR(siracUydu.donanimDurumu, ESP32_WIFI_ERR);
		token = strtok(NULL, s);
		siracUydu.ESP32Durumu.WifiSinyalGucu = atoi(token);
		break;
	}
	default:
		break;
	}
	return;
}

void UART_decode_process(void *d, uint32_t len) {
	uint8_t *p = d;
	char *token;
	char *s = "$";
	token = strtok((char*) p, s);
	if (token == NULL)
		return;
	token = strtok(NULL, s);
	if (token == NULL)
		return;
	if (!strcmp((char*) p, "TD"))
		parseTData(token);
	else if (!strcmp((char*) p, "TE"))
		parseTError(token);
	else if (!strcmp((char*) p, "GC"))
		parseGSCommand(token, strlen(token) + 1);
	else if (!strcmp((char*) p, "ES"))
		parseESP(token, len);
	return;
}
