//
// Created by mr on 10/12/2024.
//

#ifndef ESP32_S3_WROOM_MAIN_H
#define ESP32_S3_WROOM_MAIN_H

// Define note frequencies
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494


class main {

public:
	static void configurePWM(int pin, int channel, int phaseShift);
	static int smoothPotValue(int newValue, int *buffer, int size);



};


#endif //ESP32_S3_WROOM_MAIN_H
