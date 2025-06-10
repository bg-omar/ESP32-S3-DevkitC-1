//
// Created by mr on 3/12/2025.
//

#include "NeoPixel.h"
#include <Adafruit_NeoPixel.h>

#define RGB_LED_PIN 48

// RGB Color Variables
int r = 255, g = 0, b = 0;
int lastRed = -1, lastGreen = -1, lastBlue = -1;

// NeoPixel Configuration
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);



// Update RGB LED
void NeoPixel::updateRGB(int red, int green, int blue) {
	if (red != lastRed || green != lastGreen || blue != lastBlue) {
		strip.setPixelColor(0, Adafruit_NeoPixel::Color(red, green, blue));
		strip.show();
		lastRed = red;
		lastGreen = green;
		lastBlue = blue;
	}
}


void NeoPixel::getRGBFromPot(int potValue, int &red, int &green, int &blue) {
	// Map potentiometer value (0–4095) to hue (0–360 degrees)
	auto hue = static_cast<float>(map(potValue, 0, 2048, 0, 360));

	// Convert hue to RGB
	float c = 1.0;  // Full brightness
	float x = c * (1 - abs(fmod(hue / 60.0, 2) - 1));
	float m = 0.0;

	float rPrime, gPrime, bPrime;

	if (hue < 60) {
		rPrime = c; gPrime = x; bPrime = 0;
	} else if (hue < 120) {
		rPrime = x; gPrime = c; bPrime = 0;
	} else if (hue < 180) {
		rPrime = 0; gPrime = c; bPrime = x;
	} else if (hue < 240) {
		rPrime = 0; gPrime = x; bPrime = c;
	} else if (hue < 300) {
		rPrime = x; gPrime = 0; bPrime = c;
	} else {
		rPrime = c; gPrime = 0; bPrime = x;
	}

	// Scale to 0–255 and cast to int
	red = static_cast<int>((rPrime + m) * 255);
	green = static_cast<int>((gPrime + m) * 255);
	blue = static_cast<int>((bPrime + m) * 255);


	// Constrain to ensure valid values
	red = constrain(red, 0, 255);
	green = constrain(green, 0, 255);
	blue = constrain(blue, 0, 255);
}


// Update RGB brightness based on potentiometer
void NeoPixel::updateRGBBrightness(int &red, int &green, int &blue, int potValue) {
	int amount = map(potValue, 0, 4095, -100, 100);
	double percent = amount * 0.01;

	if (amount > 0) {
		red = static_cast<int>(red + ((255 - red) * percent));
		green = static_cast<int>(green + ((255 - green) * percent));
		blue = static_cast<int>(blue + ((255 - blue) * percent));
	} else if (amount < 0) {
		red = static_cast<int>(red * (1 + percent));
		green = static_cast<int>(green * (1 + percent));
		blue = static_cast<int>(blue * (1 + percent));
	}

	red = constrain(red, 0, 255);
	green = constrain(green, 0, 255);
	blue = constrain(blue, 0, 255);
}









