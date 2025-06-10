#include <Arduino.h>
#include "main_lolin.h"

#include <esp_adc_cal.h>
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/touch_pad.h"
#include <esp_task_wdt.h>
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


// Define GPIO pins
#define PHASE_1_PIN 10
#define PHASE_2_PIN 11
#define PHASE_3_PIN 12
#define RGB_LED_PIN 48
#define SPEAKER_PIN 13

// Timer and channel configuration
#define LEDC_TIMER LEDC_TIMER_1
#define LEDC_CHANNEL LEDC_CHANNEL_4
#define POTENTIOMETER1_PIN ADC1_CHANNEL_0  // Use GPIO 32 instead of GPIO 4
#define POTENTIOMETER2_PIN ADC1_CHANNEL_1 // Use GPIO 2
#define POTENTIOMETER_RX_PIN ADC1_CHANNEL_2  // Use GPIO 32 instead of GPIO 4
#define POTENTIOMETER_RY_PIN ADC1_CHANNEL_3 // Use GPIO 2
#define POTENTIOMETER_LX_PIN ADC1_CHANNEL_4  // Use GPIO 32 instead of GPIO 4
#define POTENTIOMETER_LY_PIN ADC1_CHANNEL_5 // Use GPIO 2

#define BUTTON_R_PIN 10
#define BUTTON_L_PIN 11

// PWM Configuration
#define PWM_FREQUENCY 1        // Start with 1 kHz
#define PWM_MAX_FREQUENCY 5000        // Start with 1 kHz
#define PWM_RESOLUTION LEDC_TIMER_10_BIT  // 10-bit resolution (1024 steps)
#define PHASE_SHIFT_120 (1024 / 3)        // 120° phase shift
#define PHASE_SHIFT_240 (2 * PHASE_SHIFT_120)


esp_adc_cal_characteristics_t *adc_chars;
// Variables for PWM and RGB states
int lastFrequency = 0;
int lastRed = -1, lastGreen = -1, lastBlue = -1;

// ADC Smoothing buffer
const int bufferSize = 10;
int adcBuffer1[bufferSize];
int adcBuffer2[bufferSize];

// RGB Color Variables
int r = 255, g = 0, b = 0;

// Staccato timing variables
int burstDuration = 50;  // Burst duration in milliseconds
int pauseDuration = 100; // Pause duration in milliseconds
bool isBurstOn = false;  // Track if we're in a burst or pause phase
unsigned long lastSwitchTime = 0;

int buttonRState = LOW;
int lastButtonRState = LOW;
int buttonLState = LOW;
int lastButtonLState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // 50 ms debounce delay

void setup() {
	Serial.begin(115200);

	pinMode(BUTTON_R_PIN, INPUT_PULLUP);
	pinMode(BUTTON_L_PIN, INPUT_PULLUP);
	// Initialize ADC calibration
	adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 3300, adc_chars);

	adc1_config_width(ADC_WIDTH_BIT_12);  // 12-bit resolution
	adc1_config_channel_atten(POTENTIOMETER1_PIN, ADC_ATTEN_11db);
	adc1_config_channel_atten(POTENTIOMETER2_PIN, ADC_ATTEN_11db);

	main::configurePWM(PHASE_1_PIN, LEDC_CHANNEL_0, 0);
	main::configurePWM(PHASE_2_PIN, LEDC_CHANNEL_1, PHASE_SHIFT_120);
	main::configurePWM(PHASE_3_PIN, LEDC_CHANNEL_2, PHASE_SHIFT_240);


	// Configure the PWM timer
	ledc_timer_config_t ledc_timer1 = {
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.duty_resolution = PWM_RESOLUTION,
			.timer_num = LEDC_TIMER,
			.freq_hz = PWM_FREQUENCY,
			.clk_cfg = LEDC_AUTO_CLK
	};
	ledc_timer_config(&ledc_timer1);

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
			.duty_resolution = PWM_RESOLUTION,
			.timer_num = LEDC_TIMER_0,
			.freq_hz = 1000,
			.clk_cfg = LEDC_AUTO_CLK
	};
	ledc_timer_config(&ledc_timer);
	// Initialize the watchdog timer
	esp_task_wdt_init(10, true); // 10 seconds timeout
	esp_task_wdt_add(NULL); // Add current task to watchdog

}

void loop() {
	int pot1Raw = adc1_get_raw(POTENTIOMETER1_PIN);
	int pot2Raw = adc1_get_raw(POTENTIOMETER2_PIN);

	int potRxRaw = adc1_get_raw(POTENTIOMETER_RX_PIN); // For frequency adjustment
	int potRyRaw = adc1_get_raw(POTENTIOMETER_RY_PIN); // For duty cycle adjustment
	int potLxRaw = adc1_get_raw(POTENTIOMETER_LX_PIN); // For phase offset adjustment
	int potLyRaw = adc1_get_raw(POTENTIOMETER_LY_PIN); // For phase offset adjustment

	int pot1Value = main::smoothPotValue(pot1Raw, adcBuffer1, bufferSize);
	int pot2Value = main::smoothPotValue(pot2Raw, adcBuffer2, bufferSize);

	int potRxValue = main::smoothPotValue(potRxRaw, adcBuffer1, bufferSize);
	int potRyValue = main::smoothPotValue(potRyRaw, adcBuffer2, bufferSize);
	int potLxValue = main::smoothPotValue(potLxRaw, adcBuffer1, bufferSize);
	Serial.print("potLxValue: "); Serial.println(potLxValue);
	int potLyValue = main::smoothPotValue(potLyRaw, adcBuffer2, bufferSize);

	static int minPotValue = 4095;
	static int maxPotValue = 0;

	// Update min/max values dynamically
	if (pot1Value < minPotValue) minPotValue = pot1Value;
	if (pot1Value > maxPotValue) maxPotValue = pot1Value;

	// Base frequency and duty cycle calculation
	int baseFrequency = map(pot1Value, minPotValue, maxPotValue, 0, PWM_MAX_FREQUENCY);
	baseFrequency = constrain(baseFrequency, 1, PWM_MAX_FREQUENCY);

	int dutyCycle = map(pot2Value, 0, 4095, 0, 1023);
	dutyCycle = constrain(dutyCycle, 0, 1023);

	// Adjust frequency and duty cycle by ±5% using RX and RY potentiometers
	int freqAdjustment = map(potRxValue, 0, 4095, -5, 5); // Map to ±5%
	int dutyAdjustment = map(potRyValue, 0, 4095, -5, 5); // Map to ±5%

	baseFrequency += baseFrequency * freqAdjustment / 100;
	dutyCycle += dutyCycle * dutyAdjustment / 100;

	baseFrequency = constrain(baseFrequency, 1, PWM_MAX_FREQUENCY);
	dutyCycle = constrain(dutyCycle, 0, 1023);

	// Adjust phase offset by ±1% using LX and LY potentiometers
	int phaseOffset = map(potLxValue, -4095, 4095, -1, 1); // Map to ±1%

	// Apply the phase offset to the PWM frequency
	int phase1Frequency = baseFrequency;
	int phase2Frequency = baseFrequency + baseFrequency * phaseOffset / 100;
	int phase3Frequency = baseFrequency - baseFrequency * phaseOffset / 100;

	phase2Frequency = constrain(phase2Frequency, 1, PWM_MAX_FREQUENCY);
	phase3Frequency = constrain(phase3Frequency, 1, PWM_MAX_FREQUENCY);

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
			ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, phase1Frequency);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyCycle);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

			ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, phase2Frequency);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycle);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

			ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, phase3Frequency);
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


	// Set frequency for the current note
	ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER, baseFrequency);

	// Set duty cycle to 50% (turn on the speaker)
	int speakerDutyCycle = constrain(2 * dutyCycle, 0, 1023);
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, speakerDutyCycle);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);


	// Turn off the speaker (0 duty cycle)
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);

	int readingR = digitalRead(BUTTON_R_PIN);
	int readingL = digitalRead(BUTTON_L_PIN);

	// Check if the button state has changed
	if (readingR != lastButtonRState) {
		lastDebounceTime = millis();
	}
	// Check if the button state has changed
	if (readingR != lastButtonLState) {
		lastDebounceTime = millis();
	}

	// If stable for debounce delay, update state
	if ((millis() - lastDebounceTime) > debounceDelay) {
		if (readingR != buttonRState) {
			buttonRState = readingR;

			// Print button state
			if (buttonRState == LOW) {
				Serial.println("Button R Pressed");
			} else {
				Serial.println("Button R Released");
			}
		}
	}

	// If stable for debounce delay, update state
	if ((millis() - lastDebounceTime) > debounceDelay) {
		if (readingL != buttonLState) {
			buttonLState = readingL;

			// Print button state
			if (buttonLState == LOW) {
				Serial.println("Button L Pressed");
			} else {
				Serial.println("Button L Released");
			}
		}
	}

	lastButtonRState = readingR;
	lastButtonLState = readingL;

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

// Update RGB LED

// Smooth ADC values
int main::smoothPotValue(int newValue, int *buffer, int size) {
	static int index = 0;
	buffer[index] = newValue;
	index = (index + 1) % size;

	int sum = 0;
	for (int i = 0; i < size; i++) sum += buffer[i];
	return sum / size;
}
