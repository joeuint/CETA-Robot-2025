/// @file    WS2816.ino
/// @brief   A blink example using the WS2816 controller
/// @example WS2816.ino
/// Note that the WS2816 has a 4 bit gamma correction built in. As of 3.9.12, no gamma
/// correction in FastLED is applied. However, in the future this could change to improve
/// the color accuracy.

#include <FastLED.h>
#include <math.h>

// How many leds in your strip?
#define NUM_LEDS 64
#define LED_STRIPS_LEDS 30

// Pins
#define DATA_PIN 0
#define INDICATOR_LED 16
#define MODE_BUTTON 17
#define PIEZO_PIN 15

// Task Delays
#define INDICATOR_TASK_DELAY 100
#define MATRIX_TASK_DELAY 50
#define PIEZO_TASK_DELAY 2500

// 0-255
#define INDICATOR_BRIGHTNESS 255

const int ledPattern[4][8] = {
  { HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW },  // Vroom Vroom Mode
  { HIGH, HIGH, HIGH, HIGH, LOW, LOW, LOW, LOW },  // Calibrate
  { HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW },    // Turning
  { HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW },     // Idle
};

uint8_t smileyFace[64] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0, 1, 0,
  0, 0, 1, 1, 1, 1, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0
};

uint8_t sadFace[64] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 1, 1, 1, 1, 0, 0,
  0, 1, 0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0
};

uint8_t neutralFace[64] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 1, 1, 1, 1, 1, 1, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0
};

uint8_t* currentImage;

#define MODE_COUNT 4

enum DriveMode {
  Vroom,
  Calibrate,
  Turning,
  Idle,
};

DriveMode driveMode = Idle;

static time_t prevSampleTime;
static time_t currentSampleTime;
static time_t delta;

// Define the array of leds
CRGB leds[NUM_LEDS];
CRGB ledStrips[30];

void setup() {
  currentImage = smileyFace;
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<SK6812, 13, GRB>(ledStrips, LED_STRIPS_LEDS).setCorrection(TypicalLEDStrip);
  pinMode(INDICATOR_LED, OUTPUT);
  pinMode(MODE_BUTTON, INPUT_PULLDOWN);
  pinMode(PIEZO_PIN, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), modeButton, FALLING);
}

void loop() {
  // Timing Boiler Plate
  currentSampleTime = millis();
  delta = currentSampleTime - prevSampleTime;

  // START TASKS //
  matrix();         // 8x8 LED matrix
  indicator_led();  // Status indicator LED
  piezo();          // Piezo Buzzer

  // END TASKS

  // More timing boilerplate
  prevSampleTime = currentSampleTime;
}

void modeButton() {
  static time_t buttonDebounceTime;

  if (millis() - buttonDebounceTime < 100) {
    return;
  }

  buttonDebounceTime = millis();
  noInterrupts();

  if (static_cast<int>(driveMode) == 3) {
    driveMode = static_cast<DriveMode>(0);
  } else {
    driveMode = static_cast<DriveMode>(static_cast<int>(driveMode) + 1);
  }

  Serial.print("Mode: ");
  Serial.println(driveMode);
  interrupts();
}

void indicator_led() {
  // Task Header
  static time_t ledClock;
  ledClock += delta;
  if (ledClock < INDICATOR_TASK_DELAY) {
    return;
  }

  // START TASK
  static uint8_t patternStep;

  analogWrite(INDICATOR_LED, INDICATOR_BRIGHTNESS * ledPattern[(int)driveMode][patternStep]);
  patternStep++;
  if (patternStep >= 8) {
    patternStep = 0;
  }
  // END TASK

  // Task Footer
  ledClock = 0;
}

void piezo() {
  // Task Header
  static time_t piezoClock;
  piezoClock += delta;
  if (piezoClock < PIEZO_TASK_DELAY) {
    return;
  }

  tone(PIEZO_PIN, 4200, 40);

  // Task Footer
  piezoClock = 0;
}

// Our 8x8 LED matrix
void matrix() {
  // Task Header
  static time_t matrixClock;
  matrixClock += delta;
  if (matrixClock < MATRIX_TASK_DELAY) {
    return;
  }

  // START TASK
  static uint8_t r = 0, g = 0, b = 0;
  static time_t t = 0;

  r = sin(t * 0.1 + 0) * 127 + 128;
  g = sin(t * 0.1 + 2) * 127 + 128;
  b = sin(t * 0.1 + 4) * 127 + 128;


  uint32_t colour = ((((r << 8) | g) << 8) | b);

  // Turn the LED on, then pause
  for (int i = 0; i < NUM_LEDS; i++) {
    if (currentImage[i] == 0) {
      leds[i] = 0;
      continue;
    }
    leds[i] = colour;
  }

  for (int i = 0; i < LED_STRIPS_LEDS; i++) {
    ledStrips[i] = colour;
  }
  
  FastLED.show();
  t++;
  // END TASK

  // Task Footer
  matrixClock = 0;
}