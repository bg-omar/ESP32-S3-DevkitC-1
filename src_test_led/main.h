//
// Created by mr on 10/12/2024.
//

#ifndef ESP32_S3_WROOM_MAIN_H
#define ESP32_S3_WROOM_MAIN_H


#include "./config.h"

class main {

public:


	static void printPSI_I2O();

	static int readAdc();

	static void ledTest(int delayTime);

	static void rgbBuiltin(int delayTime);

private:


};


#endif //ESP32_S3_WROOM_MAIN_H
