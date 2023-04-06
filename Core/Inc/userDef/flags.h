/*
 * flags.h
 *
 *  Created on: Jul 6, 2022
 *      Author: Naim
 */

#ifndef INC_USERDEF_FLAGS_H_
#define INC_USERDEF_FLAGS_H_
#include "userDef.h"

#define SET_FLAG(flagHandle, Flag) 	(flagHandle |= Flag)
#define CLEAR_FLAG(flagHandle, Flag)	(flagHandle &= ~Flag)
#define CHECK_FLAG(flagHandle, Flag)	(Flag==(flagHandle&Flag))

typedef enum
    {
    BEKLEME = 1,
    YUKSELME,
    MU_INIS,
    AYRILMA,
    GY_INIS1,
    ASKIDA,
    GY_INIS2,
    KONTROLLU_INIS,
    KURTARMA
    } UyduDurumu_typedef; //Letter code $s

typedef enum
    {
    BARO_ERROR = 1,
    IMU_ERROR = 2,
    UV_ERROR = 4,
    GPS_NOFIX = 8,
    GPS_ERROR = 16,
    ESP32_ERROR = 32,
    ESP32_SD_ERROR = 64,
    ESP32_WIFI_ERR = 128,
    SD_ERROR = 256,
    FLASH_ERROR = 512
    } DonanimDurumu_typedef; //Letter code $e

typedef enum
    {
    UART_LORA = 1,
    UART_ESP = 2,
    UART_GPS = 4
    } UartFlags_typedef;
typedef enum
    {
    DEVRE_DISI = 0,
    OTOMATIK_INIS = 1,
    MANUEL = 2
    } MotorKontrol_typedef;

#define AYRILMA_SURESI_MAX_MS 2000
typedef enum
    {
    AYRILDI = 0,
    AYRILMADI = 1,
    MANUEL_AYRILMA_GEREKLI = 2
    } Ayrilma_typedef;

typedef enum
    {
    TELEMETRI_VERISI_HAZIR = 0,
    TELEMETRI_VERISI_HAZIR_DEGIL = 1,
    TELEMETRI_VERISI_GEREKLI = 2,
    TELEMETRI_VERISI_GEREKLI_DEGIL = 3
    } TelemetriVeri_typedef;

typedef struct
    {
	float basinc;
	float yukseklik;
	float YerYukseklik;
	uint8_t detect;
	char gps2[35];
	TelemetriVeri_typedef TY_telemetriVeriDurumu;
    } TY_Telemetri_typedef;

typedef enum
    {
    SERVO_AC = 0,
    SERVO_KAPAT = 1,
    } ServoKomut_typedef;

typedef struct
    {
	ServoKomut_typedef servoKomutu;
	uint8_t motorTahrikGucu;
	uint8_t motorTahrikSuresi;
    } GSKomut_typedef;

#define TELEMETRI_PAKET_BOYUTU 250
typedef struct
    {
	uint16_t paketNo;
	char uyduStatu[10];
	int donusSayisi;
	char telemetriPaketi[TELEMETRI_PAKET_BOYUTU];
	TY_Telemetri_typedef tasiyiciTelemetri;
	TelemetriVeri_typedef GY_telemetriVeriDurumu;
    } GY_Telemetri_typedef;

#define ASKIDAN_ONCE_INIS_HIZI_MAX	-6
#define ASKIDAN_ONCE_INIS_HIZI		30
#define ASKIDAN_ONCE_INIS_HIZI_MIN	-8

#define ASKIDA_INIS_HIZI_MAX	-1
#define ASKIDA_INIS_HIZI	34
#define ASKIDA_INIS_HIZI_MIN	1

#define ASKIDAN_SONRA_INIS_HIZI_MAX 	-4
#define ASKIDAN_SONRA_INIS_HIZI		35
#define ASKIDAN_SONRA_INIS_HIZI_MIN	-6

#define AYRILMA_YUKSEKLIGI_MAX 415
#define AYRILMA_YUKSEKLIGI_MIN 375

#define ASKI_GOREV_BASLAMA_YUKSEKLIGI_MAX 215
#define ASKI_GOREV_BASLAMA_YUKSEKLIGI_MIN 190

#define KONTROLLU_INIS_BASLAMA_YUKSEKLIGI 20

#define KONTROLLU_INIS_HIZI_MAX -1
#define KONTROLLU_INIS_HIZI	ASKIDAN_SONRA_INIS_HIZI
#define KONTROLLU_INIS_HIZI_MIN -4

#define YUKSELME_IRTIFA_TETIKLEME 5
#define YUKSELME_HIZ_TETIKLEME 7
#define PIL_GERILIMI_MAX 0
#define PIL_GERILIMI_MIN 0
typedef struct
    {

	int16_t InisHizi_max;
	int16_t InisHizi_min;
	int16_t InisHizi;
	uint16_t BeklemedeYukseklik_max;

	int8_t PilGerilimi_max;
	int8_t PilGerilimi_min;
    } Aralik_typedef;

#define SAGLIKLI_YUKSEKLIK_MAX	2
#define SAGLIKLI_YUKSEKLIK_MIN -2
#define SAGLIKLI_SICAKLIK_MAX 	40
#define SAGLIKLI_SICAKLIK_MIN 	10
#define SAGLIKLI_DIKEYHIZ_MAX 	2
#define SAGLIKLI_DIKEYHIZ_MIN 	-2
#define SAGLIKLI_BASINC_MAX		1013
#define SAGLIKLI_BASINC_MIN		890
#define SAGLIKLI_PITCH_MAX		90
#define SAGLIKLI_PITCH_MIN		-90
#define SAGLIKLI_ROLL_MAX		90
#define SAGLIKLI_ROLL_MIN		-90
#define MOTOR_TAHRIK_SURESI_SEC		(3000)
typedef enum
    {
    yukseklik_aralik_disinda = 1,
    sicaklik_aralik_disinda = 2,
    dikeyHiz_aralik_disinda = 4,
    basinc_aralik_disinda = 8,
    pitch_aralik_disinda = 16,
    roll_aralik_disinda = 32
    } sensorSaglik_typedef;

typedef struct
    {
	volatile float Altitude_cm, avgAltitude_cm, GroundAltitude_cm,
		PrevAltitude_m, RelativeAltitude_m;
	volatile float GPSAltitude_m, GPSGroundAltitude_m, GPSPrevAltitude_m,
		GPSRelativeAltitude_m;
	volatile float KFAltitudeCm, KFAltitudeM;
	volatile float KFClimbrateCps, KFClimbrateMps;
    } Altitude_ClimbRate_typedef;

typedef struct
    {
	MPU9255_t imu;
	MS5611_typedef baro;
	lwgps_t gps;
	//UV STRUCT
	Aralik_typedef kararAraliklari;
	sensorSaglik_typedef sensorVeriSagligi;
	Altitude_ClimbRate_typedef irtifHizVerisi;
	float UV_sensor;
	float dt;
    } SensorVerisi_typedef;

#define VERI_SD_KARTA_YAZILDI 1
#define VERI_SD_KARTA_YAZILAMADI 0

typedef struct
    {
	char DosyaAdi[10];
	uint8_t DosyaDurumu;
	uint8_t KartDurumu;
	uint8_t DataDurumu;
    } SDKart_typedef;

#define VIDEO_AKTARILDI 1
#define VIDEO_AKTARILAMADI 0
typedef struct
    {
	uint8_t AktarilmaBilgisi;
	uint16_t DosyaBoyutu;
    } VideoPaketi_typedef;

typedef struct
    {
	uint8_t WifiSinyalGucu;
	VideoPaketi_typedef videoPaketBilgisi;
    } ESP32Durumu_typedef;

typedef enum
    {
    AyrilmaTimeoutFlag = 1,
    AskiTimeoutFlag = 2,
    motorTahrikTimeoutFlag = 4,
    kurtarmaTelemetriTimeoutFlag = 8
    } TimersFlag_typedef;

#define ARM 1
#define DISARM 0
typedef struct
    {
	UyduDurumu_typedef uyduDurumu;
	DonanimDurumu_typedef donanimDurumu;
	MotorKontrol_typedef motorKontrolDurumu;
	Ayrilma_typedef ayrilmaDurumu;
	float PilGerilimi;
	SensorVerisi_typedef sensorVerisi;
	GY_Telemetri_typedef telemetri;
	uint8_t ArmDurumu;
	SDKart_typedef sdKartBilgisi;
	GSKomut_typedef yerIstKomutu;
	ESP32Durumu_typedef ESP32Durumu;
    } ModelUydu_typedef;

#define PID_Kp 128.0f
#define PID_Kd 18.0f
#define PID_Ki 0.5
#define PID_tau 0.02f
#define PID_MAX PWM_MAX
#define PID_MIN PWM_MIN
typedef struct
{
    float integrator, differentiator;
    int limMin, limMax;
    int pwm;
    float prevErr, prevMeas;
}PID_typedef;

extern PID_typedef PID1;
extern ModelUydu_typedef siracUydu;
extern UartFlags_typedef uartFlag;
extern TimersFlag_typedef timersFlag;
extern TimersFlag_typedef selectedFlag;
#endif /* INC_USERDEF_FLAGS_H_ */
