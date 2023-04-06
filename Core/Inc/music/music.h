/*
 * music.h
 *
 *  Created on: Aug 2, 2022
 *      Author: Naim
 */

#ifndef INC_MUSIC_MUSIC_H_
#define INC_MUSIC_MUSIC_H_
#include "userDef/userDef.h"

#define FREQ (18000)

#define NOTE_B0  ((225000000) / (FREQ * 31))
#define NOTE_C1  ((225000000) / (FREQ * 33))
#define NOTE_CS1 ((225000000) / (FREQ * 35))
#define NOTE_D1  ((225000000) / (FREQ * 37))
#define NOTE_DS1 ((225000000) / (FREQ * 39))
#define NOTE_E1  ((225000000) / (FREQ * 41))
#define NOTE_F1  ((225000000) / (FREQ * 44))
#define NOTE_FS1 ((225000000) / (FREQ * 46))
#define NOTE_G1  ((225000000) / (FREQ * 49))
#define NOTE_GS1 ((225000000) / (FREQ * 52))
#define NOTE_A1  ((225000000) / (FREQ * 55))
#define NOTE_AS1 ((225000000) / (FREQ * 58))
#define NOTE_B1  ((225000000) / (FREQ * 62))
#define NOTE_C2  ((225000000) / (FREQ * 65))
#define NOTE_CS2 ((225000000) / (FREQ * 69))
#define NOTE_D2  ((225000000) / (FREQ * 73))
#define NOTE_DS2 ((225000000) / (FREQ * 78))
#define NOTE_E2  ((225000000) / (FREQ * 82))
#define NOTE_F2  ((225000000) / (FREQ * 87))
#define NOTE_FS2 ((225000000) / (FREQ * 93))
#define NOTE_G2  ((225000000) / (FREQ * 98))
#define NOTE_GS2 ((225000000) / (FREQ * 104))
#define NOTE_A2  ((225000000) / (FREQ * 110))
#define NOTE_AS2 ((225000000) / (FREQ * 117))
#define NOTE_B2  ((225000000) / (FREQ * 123))
#define NOTE_C3  ((225000000) / (FREQ * 131))
#define NOTE_CS3 ((225000000) / (FREQ * 139))
#define NOTE_D3  ((225000000) / (FREQ * 147))
#define NOTE_DS3 ((225000000) / (FREQ * 156))
#define NOTE_E3  ((225000000) / (FREQ * 165))
#define NOTE_F3  ((225000000) / (FREQ * 175))
#define NOTE_FS3 ((225000000) / (FREQ * 185))
#define NOTE_G3  ((225000000) / (FREQ * 196))
#define NOTE_GS3 ((225000000) / (FREQ * 208))
#define NOTE_A3  ((225000000) / (FREQ * 220))
#define NOTE_AS3 ((225000000) / (FREQ * 233))
#define NOTE_B3  ((225000000) / (FREQ * 247))
#define NOTE_C4  ((225000000) / (FREQ * 262))
#define NOTE_CS4 ((225000000) / (FREQ * 277))
#define NOTE_D4  ((225000000) / (FREQ * 294))
#define NOTE_DS4 ((225000000) / (FREQ * 311))
#define NOTE_E4  ((225000000) / (FREQ * 330))
#define NOTE_F4  ((225000000) / (FREQ * 349))
#define NOTE_FS4 ((225000000) / (FREQ * 370))
#define NOTE_G4  ((225000000) / (FREQ * 392))
#define NOTE_GS4 ((225000000) / (FREQ * 415))
#define NOTE_A4  ((225000000) / (FREQ * 440))
#define NOTE_AS4 ((225000000) / (FREQ * 466))
#define NOTE_B4  ((225000000) / (FREQ * 494))
#define NOTE_C5  ((225000000) / (FREQ * 523))
#define NOTE_CS5 ((225000000) / (FREQ * 554))
#define NOTE_D5  ((225000000) / (FREQ * 587))
#define NOTE_DS5 ((225000000) / (FREQ * 622))
#define NOTE_E5  ((225000000) / (FREQ * 659))
#define NOTE_F5  ((225000000) / (FREQ * 698))
#define NOTE_FS5 ((225000000) / (FREQ * 740))
#define NOTE_G5  ((225000000) / (FREQ * 784))
#define NOTE_GS5 ((225000000) / (FREQ * 831))
#define NOTE_A5  ((225000000) / (FREQ * 880))
#define NOTE_AS5 ((225000000) / (FREQ * 932))
#define NOTE_B5  ((225000000) / (FREQ * 988))
#define NOTE_C6  ((225000000) / (FREQ * 1047))
#define NOTE_CS6 ((225000000) / (FREQ * 1109))
#define NOTE_D6  ((225000000) / (FREQ * 1175))
#define NOTE_DS6 ((225000000) / (FREQ * 1245))
#define NOTE_E6  ((225000000) / (FREQ * 1319))
#define NOTE_F6  ((225000000) / (FREQ * 1397))
#define NOTE_FS6 ((225000000) / (FREQ * 1480))
#define NOTE_G6  ((225000000) / (FREQ * 1568))
#define NOTE_GS6 ((225000000) / (FREQ * 1661))
#define NOTE_A6  ((225000000) / (FREQ * 1760))
#define NOTE_AS6 ((225000000) / (FREQ * 1865))
#define NOTE_B6  ((225000000) / (FREQ * 1976))
#define NOTE_C7  ((225000000) / (FREQ * 2093))
#define NOTE_CS7 ((225000000) / (FREQ * 2217))
#define NOTE_D7  ((225000000) / (FREQ * 2349))
#define NOTE_DS7 ((225000000) / (FREQ * 2489))
#define NOTE_E7  ((225000000) / (FREQ * 2637))
#define NOTE_F7  ((225000000) / (FREQ * 2794))
#define NOTE_FS7 ((225000000) / (FREQ * 2960))
#define NOTE_G7  ((225000000) / (FREQ * 3136))
#define NOTE_GS7 ((225000000) / (FREQ * 3322))
#define NOTE_A7  ((225000000) / (FREQ * 3520))
#define NOTE_AS7 ((225000000) / (FREQ * 3729))
#define NOTE_B7  ((225000000) / (FREQ * 3951))
#define NOTE_C8  ((225000000) / (FREQ * 4186))
#define NOTE_CS8 ((225000000) / (FREQ * 4435))
#define NOTE_D8  ((225000000) / (FREQ * 4699))
#define NOTE_DS8 ((225000000) / (FREQ * 4978))
#define REST      0

void playMusic(int *Melody, int dim, int *Time);
void playStartSystem();
void playStartCalib();
void playEndCalib();
void playArm();
void playDisarm();
void playRescue();
#endif /* INC_MUSIC_MUSIC_H_ */
