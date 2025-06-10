//
// Created by mr on 3/12/2025.
//

#ifndef ESP32_S3_DEVKITC_1_NEOPIXEL_H
#define ESP32_S3_DEVKITC_1_NEOPIXEL_H


class NeoPixel {
public:
	void updateRGB(int red, int green, int blue);
	void getRGBFromPot(int potValue, int &red, int &green, int &blue);
	void updateRGBBrightness(int &red, int &green, int &blue, int potValue) ;
};


#endif //ESP32_S3_DEVKITC_1_NEOPIXEL_H
