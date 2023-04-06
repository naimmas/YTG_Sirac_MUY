/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "userDef/userDef.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
TimersFlag_typedef selectedFlag;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int t1 = 0, t2, t21 = 0, t22, dt1, dt2;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = { .name = "MotorTask", .stack_size =
		256 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for DataComTask */
osThreadId_t DataComTaskHandle;
const osThreadAttr_t DataComTask_attributes =
		{ .name = "DataComTask", .stack_size = 1500 * 4, .priority =
				(osPriority_t) osPriorityAboveNormal, };
/* Definitions for SensorsTask */
osThreadId_t SensorsTaskHandle;
const osThreadAttr_t SensorsTask_attributes = { .name = "SensorsTask",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityRealtime, };
/* Definitions for TelemetriTimer */
osTimerId_t TelemetriTimerHandle;
const osTimerAttr_t TelemetriTimer_attributes = { .name = "TelemetriTimer" };
/* Definitions for TimeoutTimer */
osTimerId_t TimeoutTimerHandle;
const osTimerAttr_t TimeoutTimer_attributes = { .name = "TimeoutTimer" };
/* Definitions for TahrikTimer */
osTimerId_t TahrikTimerHandle;
const osTimerAttr_t TahrikTimer_attributes = { .name = "TahrikTimer" };
/* Definitions for uartLoraSema */
osSemaphoreId_t uartLoraSemaHandle;
const osSemaphoreAttr_t uartLoraSema_attributes = { .name = "uartLoraSema" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMotorTask(void *argument);
void StartDataCom(void *argument);
void StartSensors(void *argument);
void TelemTimCall(void *argument);
void TimeoutTimerCall(void *argument);
void TahrikTimerCall(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of uartLoraSema */
	uartLoraSemaHandle = osSemaphoreNew(1, 1, &uartLoraSema_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* Create the timer(s) */
	/* creation of TelemetriTimer */
	TelemetriTimerHandle = osTimerNew(TelemTimCall, osTimerOnce, NULL,
			&TelemetriTimer_attributes);

	/* creation of TimeoutTimer */
	TimeoutTimerHandle = osTimerNew(TimeoutTimerCall, osTimerOnce,
			(void*) selectedFlag, &TimeoutTimer_attributes);

	/* creation of TahrikTimer */
	TahrikTimerHandle = osTimerNew(TahrikTimerCall, osTimerOnce, NULL,
			&TahrikTimer_attributes);

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of MotorTask */
	MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

	/* creation of DataComTask */
	DataComTaskHandle = osThreadNew(StartDataCom, NULL,
			&DataComTask_attributes);

	/* creation of SensorsTask */
	SensorsTaskHandle = osThreadNew(StartSensors, NULL,
			&SensorsTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMotorTask */
/**
 * @brief  Function implementing the MotorTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument) {
	/* USER CODE BEGIN StartMotorTask */

	/*
	 * ESC1 ve ESC2 PWM baslatilmali arm ile yapilabilir;
	 */
	int dt;
	uint32_t clockPrevious, clockNow;
	HAL_TIM_PWM_Start(&SERVO_HTIM, SERVO2_CHN);
	servoTahrik(SERVO_KAPAT);
	MotorKontrol_typedef prevKontrol = siracUydu.motorKontrolDurumu;
//	uint16_t a = uxTaskGetStackHighWaterMark(NULL);
	/*
	 * STACK SIZE 232
	 */
	//ESCcalibration();
	/* Infinite loop */
	for (;;) {
		if (siracUydu.uyduDurumu >= GY_INIS1) {
			if (siracUydu.sensorVerisi.imu.roll >= 90
					|| siracUydu.sensorVerisi.imu.pitch >= 90) {
				prevKontrol = siracUydu.motorKontrolDurumu;
				siracUydu.motorKontrolDurumu = DEVRE_DISI;
			} else if (siracUydu.motorKontrolDurumu == DEVRE_DISI) {
				siracUydu.motorKontrolDurumu = prevKontrol;
			}
		}
		switch (siracUydu.motorKontrolDurumu) {
		case (OTOMATIK_INIS):
			/*clockPrevious = clockNow;
			clockNow = HAL_GetTick();
			dt = ((float) (clockNow - clockPrevious) / 1000.0)
					+ siracUydu.sensorVerisi.dt;
			if (IS_IN_RANGE(
					siracUydu.sensorVerisi.irtifHizVerisi.KFClimbrateMps,
					siracUydu.sensorVerisi.kararAraliklari.InisHizi_max,
					siracUydu.sensorVerisi.kararAraliklari.InisHizi_min)) {
				PID(siracUydu.sensorVerisi.kararAraliklari.InisHizi,
						siracUydu.sensorVerisi.kararAraliklari.InisHizi, dt);
			} else {
				PID(siracUydu.sensorVerisi.irtifHizVerisi.KFClimbrateMps,
						siracUydu.sensorVerisi.kararAraliklari.InisHizi, dt);
			}
			motorTahrik(PID1.pwm, PID1.pwm);*/
		motorTahrik(MOTOR_PWM_CALC(siracUydu.sensorVerisi.kararAraliklari.InisHizi), MOTOR_PWM_CALC(siracUydu.sensorVerisi.kararAraliklari.InisHizi));
			break;
		case (MANUEL):
			if (siracUydu.uyduDurumu == BEKLEME) {			//timer start
				if (!osTimerIsRunning(
						TahrikTimerHandle) && !CHECK_FLAG(timersFlag, motorTahrikTimeoutFlag)) {
					osTimerStart(TahrikTimerHandle,
							pdMS_TO_TICKS(
									siracUydu.yerIstKomutu.motorTahrikSuresi
											* 1000));
				}
				motorTahrik(
						MOTOR_PWM_CALC(siracUydu.yerIstKomutu.motorTahrikGucu),
						MOTOR_PWM_CALC(siracUydu.yerIstKomutu.motorTahrikGucu));
			}
			break;
		case (DEVRE_DISI):
			motorTahrik(PWM_ARM, PWM_ARM);
			break;
		}
		osDelay(25);
	}
	/* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartDataCom */
/**
 * @brief Function implementing the DataComTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDataCom */
void StartDataCom(void *argument) {
	/* USER CODE BEGIN StartDataCom */
	//uint16_t a = uxTaskGetStackHighWaterMark(NULL);
	/*
	 * STACK SIZE 1765
	 */
	memset(ESP_line, '\0', ESP32_MAX_LENGTH);
	HAL_UART_Receive_IT(&huart3, &esp_rx_d, 1); //Enable UART INT for gps
	/* Infinite loop */
	for (;;) {

		findFlightState();

		if (siracUydu.telemetri.GY_telemetriVeriDurumu
				== TELEMETRI_VERISI_GEREKLI) {
			sendTelem();
			saveToSD();
			savePacketNum();
			saveFlightState();
			siracUydu.telemetri.GY_telemetriVeriDurumu =
					TELEMETRI_VERISI_GEREKLI_DEGIL;

			siracUydu.sensorVerisi.irtifHizVerisi.PrevAltitude_m =
					siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m;
			osTimerStart(TelemetriTimerHandle, pdMS_TO_TICKS(500));
		}
		if (CHECK_FLAG(uartFlag, UART_LORA)) {
			UART_decode_process(LORA_line, strlen((char*) LORA_line));
			memset(LORA_line, '\0', LORA_MAX_LENGTH);
			CLEAR_FLAG(uartFlag, UART_LORA);
			HAL_UARTEx_ReceiveToIdle_DMA(LORA_UART, LORA_line, LORA_MAX_LENGTH);
			__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
		}
		if (LORA_line[29] != '\0') {
			memset(LORA_line, '\0', LORA_MAX_LENGTH);
		}
		osDelay(25);
	}
	/* USER CODE END StartDataCom */
}

/* USER CODE BEGIN Header_StartSensors */
/**
 * @brief Function implementing the SensorsTask thread.
 * @param argument: Not used
 * @retval None
 */
#define ALT_IIR 80
/* USER CODE END Header_StartSensors */
void StartSensors(void *argument) {
	/* USER CODE BEGIN StartSensors */
	enableHardware();
	uint32_t clockNow, clockPrevious;

	MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);
	DataComTaskHandle = osThreadNew(StartDataCom, NULL,
			&DataComTask_attributes);

	clockNow = xTaskGetTickCount();

	float tempAlt_m = 0.0;

//	uint16_t a = uxTaskGetStackHighWaterMark(NULL);
	/*
	 * STACK SIZE 840
	 */
	/* Infinite loop */
	for (;;) {
		readAll(IMU_I2C, &siracUydu.sensorVerisi.imu);
		siracUydu.sensorVerisi.imu.axNEDmG = -siracUydu.sensorVerisi.imu.AccelY
				* 1000;
		siracUydu.sensorVerisi.imu.ayNEDmG = -siracUydu.sensorVerisi.imu.AccelX
				* 1000;
		siracUydu.sensorVerisi.imu.azNEDmG = siracUydu.sensorVerisi.imu.AccelZ
				* 1000;
		siracUydu.sensorVerisi.imu.gravityCompensatedAccelMg =
				imu_gravityCompensatedAccel(siracUydu.sensorVerisi.imu.axNEDmG,
						siracUydu.sensorVerisi.imu.ayNEDmG,
						siracUydu.sensorVerisi.imu.azNEDmG);

		ringbuf_addSample(siracUydu.sensorVerisi.imu.gravityCompensatedAccelMg,
				&AccRingBuf);

		siracUydu.sensorVerisi.imu.zAccelAverage = ringbuf_averageNewestSamples(
				10, &AccRingBuf);

		if (MS5611_SampleStateMachine(&siracUydu.sensorVerisi.baro)
				== MS5611_PRES_DATA_READY) {

			ringbuf_addSample(siracUydu.sensorVerisi.baro.Pres_hpa,
					&BaroRingBuf);

			siracUydu.sensorVerisi.baro.avgPres_hpa =
					ringbuf_averageNewestSamples(12, &BaroRingBuf);

			siracUydu.sensorVerisi.irtifHizVerisi.Altitude_cm = __getAlt(
					siracUydu.sensorVerisi.baro.avgPres_hpa,
					siracUydu.sensorVerisi.baro.Temp_C,
					ALTITUDE_REFERENCE_PRESSURE) * 100.0f;
			clockPrevious = clockNow;
			clockNow = xTaskGetTickCount();
			siracUydu.sensorVerisi.dt = ((float) (clockNow - clockPrevious)
					/ 1000.0f);

			kalmanFilter4d_predict(siracUydu.sensorVerisi.dt);
			kalmanFilter4d_update(
					siracUydu.sensorVerisi.irtifHizVerisi.Altitude_cm,
					siracUydu.sensorVerisi.imu.zAccelAverage,
					&siracUydu.sensorVerisi.irtifHizVerisi);

			siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m =
					(siracUydu.sensorVerisi.irtifHizVerisi.KFAltitudeCm
							- siracUydu.sensorVerisi.irtifHizVerisi.GroundAltitude_cm)
							/ 100.0f;

			siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m =
					(tempAlt_m * (float) ALT_IIR
							+ siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m
									* (100.0f - (float) ALT_IIR)) / 100.0f;
			tempAlt_m =
					siracUydu.sensorVerisi.irtifHizVerisi.RelativeAltitude_m;

			siracUydu.sensorVerisi.irtifHizVerisi.KFClimbrateMps =
					siracUydu.sensorVerisi.irtifHizVerisi.KFClimbrateCps
							/ 100.0f;
		}
		getBatVolt();
		getUVSensor();
		if (esp_rx_d == '\n') {
			UART_decode_process(ESP_line, strlen((char*) ESP_line));
			memset(ESP_line, '\0', ESP32_MAX_LENGTH);
			CLEAR_FLAG(uartFlag, UART_ESP);
			CLEAR_SENS_ERR(siracUydu.donanimDurumu, ESP32_ERROR);
			esp_rx_i = 0;
			esp_rx_d = 0;
			HAL_UART_Receive_IT(&huart3, &esp_rx_d, 1); //Receive GPS char on every interrupt
		}
		osDelay(25);
	}
	/* USER CODE END StartSensors */
}

/* TelemTimCall function */
void TelemTimCall(void *argument) {
	/* USER CODE BEGIN TelemTimCall */
	siracUydu.telemetri.GY_telemetriVeriDurumu = TELEMETRI_VERISI_GEREKLI;
	/* USER CODE END TelemTimCall */
}

/* TimeoutTimerCall function */
void TimeoutTimerCall(void *argument) {
	/* USER CODE BEGIN TimeoutTimerCall */
	SET_FLAG(timersFlag, (TimersFlag_typedef )argument);
	/* USER CODE END TimeoutTimerCall */
}

/* TahrikTimerCall function */
void TahrikTimerCall(void *argument) {
	/* USER CODE BEGIN TahrikTimerCall */
	SET_FLAG(timersFlag, motorTahrikTimeoutFlag);
	siracUydu.motorKontrolDurumu = DEVRE_DISI;
	/* USER CODE END TahrikTimerCall */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

