#include <Arduino.h>
/*
    MOSI: 23  <---
    MISO: 19  <---
    SCK: 18
    SS: 5

    SDA: 19   <---
    SCL: 23   <---
*/
#include "main.h"
#include <SPI.h>

#include <Adafruit_NeoPixel.h>
#include <driver/adc.h>



// Create an instance of the Adafruit_NeoPixel class
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);



const int bufferSize = 240; // Width of the screen for one waveform
int adcBuffer[bufferSize];

void setup() {
//  Set MIDI baud rate:
	Serial.begin(9600);

	// Initialize the NeoPixel library
	strip.begin();
	strip.show(); // Initialize all pixels to 'off'

	adc1_config_width(ADC_WIDTH_BIT_12);  // Set ADC resolution to 12-bit
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // Set attenuation
	main::printPSI_I2O();

}

void loop() {

	for (int i = 0; i < bufferSize; i++) {
		adcBuffer[i] = main::readAdc(); // Sample ADC
		delay(10); // Adjust this to control the time scale
	}

//	main::ledTest();
//	main::rgbBuiltin();
}

int main::readAdc() {
	return adc1_get_raw(ADC1_CHANNEL_0); // Read raw ADC value
}

void main::ledTest(int delayTime) {// Red
	strip.setPixelColor(0, Adafruit_NeoPixel::Color(255, 0, 0)); // Red
	strip.show();
	delay(delayTime);

// Green
	strip.setPixelColor(0, Adafruit_NeoPixel::Color(0, 255, 0)); // Green
	strip.show();
	delay(delayTime);

// Blue
	strip.setPixelColor(0, Adafruit_NeoPixel::Color(0, 0, 255)); // Blue
	strip.show();
	delay(delayTime);

// Yellow
	strip.setPixelColor(0, Adafruit_NeoPixel::Color(255, 255, 0)); // Yellow
	strip.show();
	delay(delayTime);

// Cyan
	strip.setPixelColor(0, Adafruit_NeoPixel::Color(0, 255, 255)); // Cyan
	strip.show();
	delay(delayTime);

// Magenta
	strip.setPixelColor(0, Adafruit_NeoPixel::Color(255, 0, 255)); // Magenta
	strip.show();
	delay(delayTime);

// White
	strip.setPixelColor(0, Adafruit_NeoPixel::Color(255, 255, 255)); // White
	strip.show();
	delay(delayTime);

// Turn off
	strip.setPixelColor(0, Adafruit_NeoPixel::Color(0, 0, 0)); // Off
	strip.show();
	delay(delayTime);
}

 void main::rgbBuiltin(int delayTime) {
	#ifdef RGB_BUILTIN
		digitalWrite(RGB_BUILTIN, HIGH);  // Turn the RGB LED white
		delay(delayTime);
		digitalWrite(RGB_BUILTIN, LOW);  // Turn the RGB LED off
		delay(delayTime);

		Adafruit_NeoPixel::Color(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);  // Red
		delay(delayTime);
		Adafruit_NeoPixel::Color(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);  // Green
		delay(delayTime);
		Adafruit_NeoPixel::Color(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);  // Blue
		delay(delayTime);
		Adafruit_NeoPixel::Color(RGB_BUILTIN, 0, 0, 0);  // Off / black
		delay(delayTime);
	#endif
}

void main::printPSI_I2O() {
	Serial.print("\t SS: ");
	Serial.print(SS);
	Serial.print("\t MOSI: ");
	Serial.print(MOSI);
	Serial.print("\t MISO: ");
	Serial.print(MISO);
	Serial.print("\t SCK: ");
	Serial.println(SCK);

	Serial.print("\t SDA: ");
	Serial.print(SDA);
	Serial.print("\t SCL: ");
	Serial.println(SCL);
	Serial.println("");
}