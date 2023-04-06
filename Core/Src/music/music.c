/*
 * music.c
 *
 *  Created on: Aug 2, 2022
 *      Author: Naim
 */
#include "userDef/userDef.h"
void playMusic(int *Melody, int dim, int *Time) {
	int i = 0;

	while (i < dim) {
		HAL_TIM_OC_Start(&BUZZER_HTIM, BUZZER_CHN);
		htim2.Instance->ARR = Melody[i];
		_managedDelay(Time[i] / 2);
		_managedDelay(5);
		HAL_TIM_OC_Stop(&BUZZER_HTIM, BUZZER_CHN);
		_managedDelay(5);
		i++;
	}
	HAL_TIM_PWM_Stop(&BUZZER_HTIM, BUZZER_CHN);
	HAL_TIM_OC_Stop(&BUZZER_HTIM, BUZZER_CHN);
}
void playStartSystem() {

	int m[] = { NOTE_FS5, NOTE_FS5, NOTE_D5, NOTE_B4, REST, NOTE_B4, REST,
			NOTE_E5,
			REST, NOTE_E5, REST, NOTE_E5, NOTE_GS5, NOTE_GS5, NOTE_A5, NOTE_B5,
			NOTE_A5, NOTE_A5 };
	int dim = sizeof(m) / sizeof(int);
	int t[dim];
	for (int i = 0; i < dim; i++) {
		t[i] = 95;
	}
	playMusic(m, dim, t);
}

void playStartCalib() {
	int m[5] = { NOTE_A6, NOTE_AS5, NOTE_A1, NOTE_CS5, NOTE_F1 };
	int t[5] = { 100, 100, 100, 50, 75 };
	playMusic(m, 5, t);
}

void playEndCalib() {
	int m[5] = { NOTE_C6, NOTE_CS5, NOTE_C1,NOTE_A6, NOTE_AS5, };
	int t[5] = { 75, 50, 100, 100, 100 };
	playMusic(m, 5, t);
}

void playArm() {
	int m[2] = { NOTE_A4,NOTE_A5 };
	int t[2] = { 400 };
	playMusic(m, 2, t);
}
void playDisarm() {
	int m[3] = { NOTE_FS6, NOTE_FS5, NOTE_AS6 };
	int t[3] = { 400, 300, 250 };
	playMusic(m, 3, t);
}
void playRescue() {
	int m[] = { NOTE_FS5, NOTE_FS5, NOTE_D5, NOTE_B4, REST, NOTE_B4,NOTE_E5, NOTE_GS5, NOTE_GS5, NOTE_A5, NOTE_B5,
			NOTE_A5, NOTE_A5,NOTE_FS5, NOTE_FS5, NOTE_D5 };
	int dim = sizeof(m) / sizeof(int);
		int t[dim];
		for (int i = 0; i < dim; i++) {
			t[i] = 500;
		}
		playMusic(m, dim, t);
}
