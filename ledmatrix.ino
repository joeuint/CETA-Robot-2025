/// @file    WS2816.ino
/// @brief   A blink example using the WS2816 controller
/// @example WS2816.ino
/// Note that the WS2816 has a 4 bit gamma correction built in. As of 3.9.12, no gamma
/// correction in FastLED is applied. However, in the future this could change to improve
/// the color accuracy.

#include <FastLED.h>
#include <math.h>
#include <Servo.h>
#include <WiFi.h>
#include <ArduinoMqttClient.h>

#include "aio.h"

// How many leds in your strip?
#define NUM_LEDS 64
#define LED_STRIPS_LEDS 30

// Pins
#define MATRIX_DATA_PIN 12
#define INDICATOR_LED 16
#define MODE_BUTTON 17
#define PIEZO_PIN 9
#define STRIP_DATA_PIN 13

// Task Delays
#define INDICATOR_TASK_DELAY 400
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

Servo motor1;
Servo motor2;

// Wi-Fi and MQTT
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

void mqttFaceChange(String message) {
  if (message == "SAD") {
    currentImage = sadFace;
  } else if (message == "NEUTRAL") {
    currentImage = neutralFace;
  } else if (message == "HAPPY") {
    currentImage = smileyFace;
  }
}

void onMQTTMessage(int messageSize) {
  // Only supports 255 chars + 1 NUL
  char buf[256];
  int i = 0;
  while (mqttClient.available() && i < sizeof(buf) - 1) {
    buf[i++] = (char)mqttClient.read();
  }

  buf[i] = '\0';

  String message = String(buf);

  Serial.print("MQTT Message: ");
  Serial.println(message);

  mqttFaceChange(message);
}

void initWifi() {
  Serial.print("\nConnecting to network: ");
  Serial.println(WIFI_SSID);
  Serial.print("Key starting with: ");
  Serial.println(WIFI_KEY[0]);
  WiFi.begin(WIFI_SSID, WIFI_KEY);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
    digitalWrite(MQTT_LED, HIGH);
    delay(100);
    digitalWrite(MQTT_LED, LOW);

    //
    delay(800);
  }

  Serial.println("Welcome to the network! :)");

  // MQTT still not ready
  digitalWrite(MQTT_LED, LOW);
}

void initMqtt() {
  pinMode(MQTT_LED, OUTPUT);

  // Connect to Wi-Fi
  initWifi();

  Serial.println("Connecting to MQTT");

  // It's MQTT'ing time
  mqttClient.setId(MQTT_CLIENT_ID);
  mqttClient.setUsernamePassword(IO_USERNAME, IO_KEY);

  if (!mqttClient.connect(MQTT_BROKER, MQTT_PORT)) {
    Serial.println("panic: Failed to connect to MQTT broker!");
    while(true) {
      tone(PIEZO_PIN, 5000, 100);
      digitalWrite(MQTT_LED, HIGH);
      delay(1000);
      digitalWrite(MQTT_LED, LOW);
      delay(500);
    };
  };

  mqttClient.onMessage(onMQTTMessage);

  if (mqttClient.subscribe(MQTT_FEED_TOPIC, 0)) {
      Serial.println("Liked and subscribed to feed!");
      digitalWrite(MQTT_LED, HIGH);
  } else {
      Serial.println("Failed to like and subscribe to feed.");
      digitalWrite(MQTT_LED, LOW); // Continuing
  }

  Serial.println("Welcome to MQTT! :D");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hello World!");

  // LEDs
  FastLED.addLeds<WS2812B, MATRIX_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<SK6812, STRIP_DATA_PIN, GRB>(ledStrips, LED_STRIPS_LEDS).setCorrection(TypicalLEDStrip);

  // Pins
  pinMode(INDICATOR_LED, OUTPUT);
  pinMode(MODE_BUTTON, INPUT_PULLDOWN);
  pinMode(PIEZO_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), modeButton, FALLING);
  
  // Motors
  motor1.attach(1);
  motor2.attach(2);
  motor1.write(90); // STOP
  motor2.write(90); // STOP

  initMqtt();

  currentImage = smileyFace;
}

void loop() {
  // Timing Boiler Plate
  currentSampleTime = millis();
  delta = currentSampleTime - prevSampleTime;

  // START TASKS //
  matrix();         // 8x8 LED matrix
  indicator_led();  // Status indicator LED
  piezo();          // Piezo Buzzer
  iot();

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

// TASKS //
void iot() {

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

  tone(PIEZO_PIN, 3200, 40);

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