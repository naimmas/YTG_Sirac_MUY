/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void enableHardware();
void checkSensorHealth();
void readAllSensors();
void findFlightState();
void sendTelem();
void saveToSD();
void PID(float measurement, float setpoint, uint8_t dt);
void getFlightState();
void saveFlightState();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LORA_AUX_Pin GPIO_PIN_4
#define LORA_AUX_GPIO_Port GPIOE
#define LORA_M1_Pin GPIO_PIN_5
#define LORA_M1_GPIO_Port GPIOE
#define LORA_M0_Pin GPIO_PIN_6
#define LORA_M0_GPIO_Port GPIOE
#define ADC3_VBAT_Pin GPIO_PIN_2
#define ADC3_VBAT_GPIO_Port GPIOC
#define ADC3_ABAT_Pin GPIO_PIN_3
#define ADC3_ABAT_GPIO_Port GPIOC
#define BUZZER_PWM_Pin GPIO_PIN_0
#define BUZZER_PWM_GPIO_Port GPIOA
#define SERVO_PWM_Pin GPIO_PIN_6
#define SERVO_PWM_GPIO_Port GPIOA
#define AYRILMA_BTN_Pin GPIO_PIN_7
#define AYRILMA_BTN_GPIO_Port GPIOA
#define UV_SENSOR_Pin GPIO_PIN_0
#define UV_SENSOR_GPIO_Port GPIOB
#define ESC1_PWM_Pin GPIO_PIN_9
#define ESC1_PWM_GPIO_Port GPIOE
#define ESC2_PWM_Pin GPIO_PIN_11
#define ESC2_PWM_GPIO_Port GPIOE
#define ESP32_TX_Pin GPIO_PIN_8
#define ESP32_TX_GPIO_Port GPIOD
#define ESP32_RX_Pin GPIO_PIN_9
#define ESP32_RX_GPIO_Port GPIOD
#define LORA_TX_Pin GPIO_PIN_6
#define LORA_TX_GPIO_Port GPIOC
#define LORA_RX_Pin GPIO_PIN_7
#define LORA_RX_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_3
#define LED_B_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_5
#define LED_R_GPIO_Port GPIOD
#define SERVO2_PWM_Pin GPIO_PIN_5
#define SERVO2_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
