#include <Arduino.h>
#include "main.h"
#include "NeoPixel.h"
#include <esp_adc_cal.h>
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/touch_pad.h"

// Choosing PWM Frequency and Resolution
// The ESP32 can generate a PWM signal with a frequency of up to 40 MHz, and the PWM resolution can be adjusted from 1 to 16 bits. But this doesn’t mean you can set a frequency of 40 MHz and a resolution of 16 bits at the same time. This is due to the fact that the maximum PWM frequency and resolution are both bound by the clock source.

// To illustrate this, consider a clock (whether it’s a CPU clock or a timer doesn’t matter) running at a frequency of 40 MHz. In this case, the maximum achievable PWM frequency is also 40 MHz. We cannot generate a PWM wave faster than our clock allows.

// And what about the resolution? Well, resolution is really about how finely we can slice up one period of the PWM wave into different duty cycles. And here is the insight: slicing up the PWM wave requires a CPU clock running at PWM_freq * 2PWM_resolution. Why? Because to create those duty cycles, you need to be able to create those time slices.

// From this, two important points become clear:

// PWM_freq * 2PWM_resolution cannot exceed the clock speed.
// PWM frequency and resolution are interdependent. The higher the PWM frequency, the lower the duty cycle resolution (and vice versa).
// According to Espressif documentation, the LEDC low-speed timer clock source is an 80 MHz APB clock. As a general guideline, you should aim to keep PWM_freq * 2PWM_resolution below 80 MHz.

// Additionally, the Espressif documentation includes examples to back this up:

// A PWM frequency of 5 kHz can have a maximum duty resolution of 13 bits, which results in a resolution of ~0.012%, or 213=8192 discrete levels.
// A PWM frequency of 20 MHz can have a maximum duty resolution of 2 bits, which results in a resolution of 25%, or 22=4 discrete levels.
// A PWM frequency of 40 MHz can have a duty resolution of just 1 bit, meaning the duty cycle remains fixed at 50% and cannot be adjusted.
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_SDA 8
#define OLED_SCL 9

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// Define GPIO pins
#define PHASE_1_PIN 35
#define PHASE_2_PIN 36
#define PHASE_3_PIN 37
#define SPEAKER_PIN 18

// Timer and channel configuration
#define LEDC_TIMER LEDC_TIMER_1
#define LEDC_CHANNEL LEDC_CHANNEL_4

#define POTENTIOMETER1_PIN ADC2_CHANNEL_8  // Use GPIO 19
#define POTENTIOMETER2_PIN ADC2_CHANNEL_9 // Use GPIO 20
#define POTENTIOMETER_RX_PIN ADC1_CHANNEL_6  // Use GPIO 7
#define POTENTIOMETER_RY_PIN ADC1_CHANNEL_5 // Use GPIO 6
#define POTENTIOMETER_LX_PIN ADC1_CHANNEL_4  // Use GPIO 5
#define POTENTIOMETER_LY_PIN ADC1_CHANNEL_3 // Use GPIO 4

// GPIO numbers for the analog inputs
#define POTENTIOMETER1_GPIO 19
#define POTENTIOMETER2_GPIO 20
#define POTENTIOMETER_RX_GPIO 7
#define POTENTIOMETER_RY_GPIO 6
#define POTENTIOMETER_LX_GPIO 5
#define POTENTIOMETER_LY_GPIO 4

// --- Buttons ---
#define BUTTON_L_PIN 10           // GPIO10, input_pullup
#define BUTTON_R_PIN 11           // GPIO11,

// PWM Configuration
#define PWM_FREQUENCY 100        // Start with 1 kHz
#define PWM_MAX_FREQUENCY 200000        // Start with 1 kHz
#define PWM_RESOLUTION LEDC_TIMER_8_BIT  // 10-bit resolution (1024 steps)
#define PHASE_SHIFT_120 (1024 / 3)        // 120° phase shift
#define PHASE_SHIFT_240 (2 * PHASE_SHIFT_120)

// Preset frequency ranges
struct FreqPreset {
        float minFreq;
        float maxFreq;
        const char *label;
};

const FreqPreset FREQUENCY_PRESETS[] = {
        {0.1f,    100.0f,   "0.1-100"},
        {1.0f,    1000.0f,  "1-1000"},
        {1.0f,    10000.0f, "1-10000"},
        {1000.0f, 50000.0f, "1k-50k"},
        {10000.0f,150000.0f,"10k-150k"},
        {50000.0f,200000.0f,"50k-200k"}
};

const int NUM_PRESETS = sizeof(FREQUENCY_PRESETS)/sizeof(FREQUENCY_PRESETS[0]);




esp_adc_cal_characteristics_t *adc_chars;
// Variables for PWM and RGB states
int lastFrequency = 0;


// ADC Smoothing buffer
const int bufferSize = 10;
int adcBuffer1[bufferSize];
int adcBuffer2[bufferSize];
int adcBuffer3[bufferSize];
int adcBuffer4[bufferSize];
int adcBuffer5[bufferSize];
int adcBuffer6[bufferSize];

// Staccato timing variables
int burstDuration = 50;  // Burst duration in milliseconds
int pauseDuration = 100; // Pause duration in milliseconds
bool isBurstOn = false;  // Track if we're in a burst or pause phase
unsigned long lastSwitchTime = 0;
int presetIndex = 0;    // Current frequency preset

enum Option {
        OPTION_FREQ_MODE = 0,
        OPTION_BITRATE,
        OPTION_DUTY_CYCLE,
        OPTION_WAVE_SHAPE,
        OPTION_TEST_MODE,
        NUM_OPTIONS
};

const char *OPTION_NAMES[] = {
        "Freq",
        "Bits",
        "Duty",
        "Wave",
        "Test"
};

enum WaveShape {
        WAVE_SAW = 0,
        WAVE_SIN,
        WAVE_SQUARE,
        WAVE_INV_SAW,
        NUM_WAVES
};

const char *WAVE_NAMES[] = {"saw", "sin", "square", "invSaw"};
int sawStep = 0;
int sawMax = 1023;  // For 10-bit PWM
int sawIncrement = 8; // Step per loop; defines slope

const int sineTableSize = 64;
const uint16_t sineTable[sineTableSize] = {
		512, 562, 611, 659, 705, 750, 791, 830,
		866, 899, 928, 953, 974, 991,1003,1011,
		1013,1011,1003, 991, 974, 953, 928, 899,
		866, 830, 791, 750, 705, 659, 611, 562,
		512, 461, 412, 364, 318, 273, 232, 193,
		157, 124,  95,  70,  49,  32,  20,  12,
		10,  12,  20,  32,  49,  70,  95, 124,
		157, 193, 232, 273, 318, 364, 412, 461
};
int sineIndex = 0;

int currentOption = OPTION_FREQ_MODE;
int bitrate = 8;                // default resolution bits
int dutyCycleSetting = 512;     // default duty cycle
int waveShape = WAVE_SAW;

int buttonRState = LOW;
int lastButtonRState = LOW;
int buttonLState = LOW;
int lastButtonLState = LOW;
unsigned long lastDebounceTimeR, lastDebounceTimeL = 0;
unsigned long debounceDelay = 50;  // 50 ms debounce delay

int calculateSafeResolution(int pwmFreq) {
	const uint32_t apb_clk = 80000000; // 80 MHz for ESP32-S3
	int resolution = 12; // Start from maximum 12 bits

	while (resolution > 1) {
		uint32_t div_param = apb_clk / (pwmFreq * (1 << resolution));
		if (div_param > 1 && div_param < 1048576) { // 2^20 = 1048576
			break; // Found a valid resolution
		}
		resolution--; // Otherwise lower the resolution
	}
	return resolution -3;
}

void updatePWM(int timer_num, int freq_hz, int resolution_bits) {
        int safeResolution = resolution_bits > 0 ? resolution_bits : calculateSafeResolution(freq_hz);

	ledc_timer_config_t ledc_timer = {
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.duty_resolution = (ledc_timer_bit_t)safeResolution,
			.timer_num = (ledc_timer_t)timer_num,
			.freq_hz = static_cast<uint32_t>(freq_hz),
			.clk_cfg = LEDC_AUTO_CLK
	};

	esp_err_t result = ledc_timer_config(&ledc_timer);

	if (result == ESP_OK) {
		Serial.print("[PWM] Timer ");
		Serial.print(timer_num);
		Serial.print(" updated to ");
		Serial.print(freq_hz);
		Serial.print(" Hz, resolution: ");
		Serial.print(safeResolution);
		Serial.println(" bits");
	} else {
		Serial.println("[PWM] Timer update FAILED!");
	}
}

static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setAllPWM(int duty) {
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}


void setup() {
	Serial.begin(115200);
//	strip.begin();
//	strip.show();

	Wire.begin(OLED_SDA, OLED_SCL);

	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C is common I2C address
		Serial.println(F("SSD1306 allocation failed"));
		for(;;);
	}
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("3-Phase:");
        display.setCursor(0, 10);
        display.print("P1="); display.print(PHASE_1_PIN);
        display.print(" P2="); display.print(PHASE_2_PIN);
        display.print(" P3="); display.print(PHASE_3_PIN);
        display.setCursor(0, 20);
        display.println("Inputs:");
        display.setCursor(0, 30);
        display.print("BtnL="); display.print(BUTTON_L_PIN);
        display.print(" BtnR="); display.print(BUTTON_R_PIN);
        display.setCursor(0, 40);
        display.print("Pot1="); display.print(POTENTIOMETER1_GPIO);
        display.print(" Pot2="); display.print(POTENTIOMETER2_GPIO);
        display.setCursor(0, 50);
        display.print("Rx="); display.print(POTENTIOMETER_RX_GPIO);
        display.print(" Ry="); display.print(POTENTIOMETER_RY_GPIO);
        display.print(" Lx="); display.print(POTENTIOMETER_LX_GPIO);
        display.print(" Ly="); display.print(POTENTIOMETER_LY_GPIO);
        display.display();
        delay(5000);
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Initializing...");
        display.display();
        delay(500);
	// Disable touch functionality for GPIO 4 (T0)
	touch_pad_deinit();

        int initialFrequency = 1000; // or whatever your starting frequency is
        int safeResolution = bitrate;
	Serial.print("Initial Safe Resolution: ");
	Serial.println(safeResolution);

	ledc_timer_config_t ledc_timer1 = {
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.duty_resolution = (ledc_timer_bit_t)safeResolution,
			.timer_num = LEDC_TIMER,
			.freq_hz = static_cast<uint32_t>(initialFrequency),
			.clk_cfg = LEDC_AUTO_CLK
	};
	ledc_timer_config(&ledc_timer1);

	pinMode(BUTTON_R_PIN, INPUT_PULLUP);
	pinMode(BUTTON_L_PIN, INPUT_PULLUP);
	// Initialize ADC calibration
	adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 3300, adc_chars);


	adc1_config_width(ADC_WIDTH_BIT_12); // Full 12-bit (0-4095)

	adc1_config_channel_atten(POTENTIOMETER_RX_PIN, ADC_ATTEN_11db);
	adc1_config_channel_atten(POTENTIOMETER_RY_PIN, ADC_ATTEN_11db);
	adc1_config_channel_atten(POTENTIOMETER_LX_PIN, ADC_ATTEN_11db);
	adc1_config_channel_atten(POTENTIOMETER_LY_PIN, ADC_ATTEN_11db);

	adc2_config_channel_atten(POTENTIOMETER1_PIN, ADC_ATTEN_11db);
	adc2_config_channel_atten(POTENTIOMETER2_PIN, ADC_ATTEN_11db);

	main::configurePWM(PHASE_1_PIN, LEDC_CHANNEL_0, 0);
	main::configurePWM(PHASE_2_PIN, LEDC_CHANNEL_1, PHASE_SHIFT_120);
	main::configurePWM(PHASE_3_PIN, LEDC_CHANNEL_2, PHASE_SHIFT_240);



	// Configure the PWM channel
	ledc_channel_config_t ledc_channel = {
			.gpio_num = SPEAKER_PIN,
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.channel = LEDC_CHANNEL,
			.timer_sel = LEDC_TIMER,
			.duty = 512, // 50% duty cycle (max is 1024 for 10-bit resolution)
			.hpoint = 0
	};
	ledc_channel_config(&ledc_channel);



        ledc_timer_config_t ledc_timer = {
					.speed_mode = LEDC_LOW_SPEED_MODE,
					.duty_resolution = (ledc_timer_bit_t)bitrate,
					.timer_num = LEDC_TIMER_0,
					.freq_hz = 1000,
					.clk_cfg = LEDC_AUTO_CLK
        };
        ledc_timer_config(&ledc_timer);
}

void loop() {

	display.clearDisplay();
	display.setCursor(0, 0);


	int readingR = digitalRead(BUTTON_R_PIN);
	int readingL = digitalRead(BUTTON_L_PIN);

	// Check if the button state has changed
	if (readingR != lastButtonRState) {
		lastDebounceTimeR = millis();
	}
	if (readingL != lastButtonLState) {
		lastDebounceTimeL = millis();
	}

        // If stable for debounce delay, update state
        if ((millis() - lastDebounceTimeR) > debounceDelay) {
                if (readingR != buttonRState) {
                        buttonRState = readingR;

                        // Print button state
                        if (buttonRState == LOW) {
                                currentOption = (currentOption + 1) % NUM_OPTIONS;
                                Serial.print("Option: ");
                                Serial.println(currentOption);
                        }
                }
        }

        // If stable for debounce delay, update state
        if ((millis() - lastDebounceTimeL) > debounceDelay) {
                if (readingL != buttonLState) {
                        buttonLState = readingL;

                        // Print button state
                        if (buttonLState == LOW) {
                                currentOption = (currentOption + NUM_OPTIONS - 1) % NUM_OPTIONS;
                                Serial.print("Option: ");
                                Serial.println(currentOption);
                        }
                }
        }

        lastButtonRState = readingR;
        lastButtonLState = readingL;
        if (currentOption == OPTION_TEST_MODE) {
                display.print("L: "); display.print(lastButtonLState ? "Off" : "On");
                display.print(" \t R: "); display.println(lastButtonRState ? "Off" : "On");
        }
        display.print("Opt: "); display.print(OPTION_NAMES[currentOption]);
        display.print(" ");
        switch(currentOption) {
                case OPTION_FREQ_MODE:
                        display.println(FREQUENCY_PRESETS[presetIndex].label);
                        break;
                case OPTION_BITRATE:
                        display.print(bitrate);
                        display.println(" bits");
                        break;
                case OPTION_DUTY_CYCLE:
                        display.print(dutyCycleSetting);
                        display.println("%");
                        break;
                case OPTION_WAVE_SHAPE:
                        display.println(WAVE_NAMES[waveShape]);
                        break;
                case OPTION_TEST_MODE:
                        display.println();
                        break;
        }

        display.print("Bit: "); display.print(bitrate);
        display.print("  Duty: "); display.print(dutyCycleSetting);
        display.println("%");

	int pot1Raw = 0;
	esp_err_t pot1Status = adc2_get_raw(POTENTIOMETER1_PIN, ADC_WIDTH_BIT_12, &pot1Raw);
	if (pot1Status != ESP_OK) {
		Serial.println("Warning: Failed to read pot2Raw (ADC2)!");
		pot1Raw = 0;
	}

	int pot2Raw = 0;
	esp_err_t pot2Status = adc2_get_raw(POTENTIOMETER2_PIN, ADC_WIDTH_BIT_12, &pot2Raw);
	if (pot2Status != ESP_OK) {
		Serial.println("Warning: Failed to read pot2Raw (ADC2)!");
		pot2Raw = 0;
	}

	int potRxRaw = adc1_get_raw(POTENTIOMETER_RX_PIN);
	int potRyRaw = adc1_get_raw(POTENTIOMETER_RY_PIN);
	int potLxRaw = adc1_get_raw(POTENTIOMETER_LX_PIN);
	int potLyRaw = adc1_get_raw(POTENTIOMETER_LY_PIN);

// Now smooth them:
	int pot1Value = main::smoothPotValue(pot1Raw, adcBuffer1, bufferSize);
	int pot2Value = main::smoothPotValue(pot2Raw, adcBuffer2, bufferSize);
	pot1Value= map(pot1Value, 0, 2047, 0, 4095);
	pot2Value= map(pot2Value, 0, 2047, 0, 4095);

        if (currentOption == OPTION_TEST_MODE) {
                display.print("1: "); display.print(pot1Value);
                display.print(" \t 2: "); display.println(pot2Value);
        }

        switch(currentOption) {
                case OPTION_FREQ_MODE:
                        presetIndex = map(pot2Value, 0, 4095, 0, NUM_PRESETS-1);
                        break;
                case OPTION_BITRATE:
                        bitrate = map(pot2Value, 0, 4095, 1, 12);
                        break;
                case OPTION_DUTY_CYCLE:
                        dutyCycleSetting = map(pot2Value, 0, 4095, 0, 100);
                        break;
                case OPTION_WAVE_SHAPE:
                        waveShape = map(pot2Value, 0, 4095, 0, NUM_WAVES-1);
                        break;
                case OPTION_TEST_MODE:
                        break;
        }

	int potRxValue = main::smoothPotValue(potRxRaw, adcBuffer3, bufferSize);
	int potRyValue = main::smoothPotValue(potRyRaw, adcBuffer4, bufferSize);
	int potLxValue = main::smoothPotValue(potLxRaw, adcBuffer5, bufferSize);
	int potLyValue = main::smoothPotValue(potLyRaw, adcBuffer6, bufferSize);

// Left Stick X (Lx)
	int lxMapped;
	if (potLxValue <= 122) {
		lxMapped = map(potLxValue, 0, 122, 0, 2047);
	} else {
		lxMapped = map(potLxValue, 122, 470, 2048, 4095);
	}
	lxMapped = constrain(lxMapped, 0, 4095);

// Left Stick Y (Ly)
	int lyMapped;
	if (potLyValue <= 14) {
		lyMapped = map(potLyValue, 0, 14, 0, 2047);
	} else {
		lyMapped = map(potLyValue, 14, 240, 2048, 4095);
	}
	lyMapped = constrain(lyMapped, 0, 4095);

// Right Stick X (Rx)
	int rxMapped;
	if (potRxValue <= 1060) {
		rxMapped = map(potRxValue, 0, 1060, 0, 2047);
	} else {
		rxMapped = map(potRxValue, 1060, 2047, 2048, 4095);
	}
	rxMapped = constrain(rxMapped, 0, 4095);

// Right Stick Y (Ry)
	int ryMapped;
	if (potRyValue <= 340) {
		ryMapped = map(potRyValue, 0, 340, 0, 2047);
	} else {
		ryMapped = map(potRyValue, 340, 1100, 2048, 4095);
	}
	ryMapped = constrain(ryMapped, 0, 4095);


        if (currentOption == OPTION_TEST_MODE) {
                display.print("Lx: "); display.print(lxMapped);
                display.print(" \t Ly: "); display.println(lyMapped);
                display.print("Rx: "); display.print(rxMapped);
                display.print(" \t Ry: "); display.println(ryMapped);
        }



	static int minPotValue = 4095;
	static int maxPotValue = 0;

	// Update min/max values dynamically
	if (pot1Value < minPotValue) minPotValue = pot1Value;
	if (pot1Value > maxPotValue) maxPotValue = pot1Value;

        // Base frequency and duty cycle calculation using preset range
        float maxFreq = FREQUENCY_PRESETS[presetIndex].maxFreq;
        float minFreq = FREQUENCY_PRESETS[presetIndex].minFreq;
        float baseFrequency = mapFloat((float)pot1Value,
                                       (float)minPotValue,
                                       (float)maxPotValue,
                                       0.0f,
                                       maxFreq);
        baseFrequency = constrain(baseFrequency, minFreq, maxFreq);

        int dutyCycle = map(dutyCycleSetting, 0, 100, 0, 1023);

	// Adjust frequency and duty cycle by ±5% using RX and RY potentiometers
	int freqAdjustment = map(rxMapped, 0, 4095, -100, 100); // Map to ±5%
	int dutyAdjustment = map(ryMapped, 0, 4095, -15, 15); // Map to ±5%

        baseFrequency += baseFrequency * freqAdjustment / 100;
        dutyCycle += dutyCycle * dutyAdjustment / 100;

        baseFrequency = constrain(baseFrequency, minFreq, maxFreq);
        dutyCycle = constrain(dutyCycle, 0, 1023);

        updatePWM(LEDC_TIMER, static_cast<int>(baseFrequency), bitrate);

	if (waveShape == WAVE_SAW) {
		sawStep += sawIncrement;
		if (sawStep >= sawMax) sawStep = 0;
		setAllPWM(sawStep);
		sawStep += sawIncrement;
		sawIncrement = map(pot1Value, 0, 4095, 1, 64); // slow → fast ramp

		if (sawStep >= sawMax) sawStep = 0; // Reset on overflow

		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, sawStep);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, sawStep);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, sawStep);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
	} else

	if (waveShape == WAVE_INV_SAW) {
		sawStep -= sawIncrement;
		if (sawStep <= 0) sawStep = sawMax;
		setAllPWM(sawStep);
	} else

	if (waveShape == WAVE_SIN) {
		static unsigned long lastSineUpdate = 0;
		int sineDelay = map(pot1Value, 0, 4095, 2, 50);  // From fast to slow (~Hz)

		if (millis() - lastSineUpdate >= sineDelay) {
			lastSineUpdate = millis();

			int indexA = sineIndex;
			int indexB = (sineIndex + sineTableSize / 3) % sineTableSize;      // +120°
			int indexC = (sineIndex + 2 * sineTableSize / 3) % sineTableSize;  // +240°

			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, sineTable[indexA]);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, sineTable[indexB]);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, sineTable[indexC]);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

			sineIndex = (sineIndex + 1) % sineTableSize;
		}
	} else
		if (waveShape == WAVE_SQUARE) {
		static bool toggle = false;
		toggle = !toggle;
		int value = toggle ? sawMax : 0;
		setAllPWM(value);

		} else {

		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyCycle);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycle);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, dutyCycle);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
	}

	// Adjust phase offset by ±5% using LX and LY potentiometers
	int phaseOffset = map(lyMapped, 0, 4095, -5, 5); // Map to ±5%

	// Apply the phase offset to the PWM frequency
        float phase1Frequency = baseFrequency;
        float phase2Frequency = baseFrequency + baseFrequency * phaseOffset / 100.0f;
        float phase3Frequency = baseFrequency - baseFrequency * phaseOffset / 100.0f;

        phase2Frequency = constrain(phase2Frequency, minFreq, maxFreq);
        phase3Frequency = constrain(phase3Frequency, minFreq, maxFreq);

	display.print("phase1: "); display.println(phase1Frequency);
	display.print("phase2: "); display.print(phase2Frequency);
	display.print(" + "); display.println(phaseOffset);
	display.print("phase3: "); display.print(phase3Frequency);
	display.print(" - "); display.println(phaseOffset);

	// Handle burst and pause timing
	unsigned long currentTime = millis();
	if (isBurstOn) {
		// During burst: Update PWM frequency and duty cycle
		if (currentTime - lastSwitchTime > burstDuration) {
			isBurstOn = false;
			lastSwitchTime = currentTime;

			// Turn off PWM (0 duty cycle)
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
		} else {
			// During burst: Ensure PWM is running
			// Update PWM outputs
                        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, static_cast<int>(phase1Frequency));
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyCycle);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

                        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, static_cast<int>(phase2Frequency));
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycle);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

                        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, static_cast<int>(phase3Frequency));
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, dutyCycle);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
		}
	} else {
		// During pause
		if (currentTime - lastSwitchTime > pauseDuration) {
			isBurstOn = true;
			lastSwitchTime = currentTime;
		}
	}

        long div_param = (APB_CLK_FREQ) / (static_cast<int>(phase1Frequency) * (2 ^ 4096));
	display.print("div_param: "); display.println(div_param);

	// Set frequency for the current note
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER, static_cast<int>(baseFrequency));

	// Set duty cycle to 50% (turn on the speaker)
	int speakerDutyCycle = constrain(2 * dutyCycle, 0, 1023);
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, speakerDutyCycle);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);


	// Turn off the speaker (0 duty cycle)
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);

	display.display();
}



// Configure a PWM channel
void main::configurePWM(int pin, int channel, int phaseShift) {
	ledc_channel_config_t ledc_channel = {
			.gpio_num = pin,
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.channel = static_cast<ledc_channel_t>(channel),
			.timer_sel = LEDC_TIMER_0,
			.duty = 512,  // Default 50% duty cycle
			.hpoint = phaseShift
	};
	ledc_channel_config(&ledc_channel);
}



// Smooth ADC values
int main::smoothPotValue(int newValue, int *buffer, int size) {
	static int index = 0;
	buffer[index] = newValue;
	index = (index + 1) % size;

	int sum = 0;
	for (int i = 0; i < size; i++) sum += buffer[i];
	return sum / size;
}

