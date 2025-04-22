#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

uint16_t irBuffer[100];  // infrared LED sensor data
uint16_t redBuffer[100]; // red LED sensor data

int32_t bufferLength; 
int32_t spo2; 
int8_t validSPO2; 
int32_t heartRate; 
int8_t validHeartRate; 

byte pulseLED = 11; // Must be on PWM pin
byte readLED = 13;  // Blinks with each data read

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started...");

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0); // Wait until user presses a key
  Serial.read();

  byte ledBrightness = 60;     // 0 = off to 255 = max current
  byte sampleAverage = 4;      // 1, 2, 4, 8, 16, 32
  byte ledMode = 2;            // 1 = red, 2 = red+IR, 3 = red+IR+green
  byte sampleRate = 100;       // 50–3200 Hz
  int pulseWidth = 411;        // 69–411 µs
  int adcRange = 4096;         // 2048–16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void loop() {
  bufferLength = 100;

  // Read initial 100 samples
  for (byte i = 0; i < bufferLength; i++) {
    while (!particleSensor.available())
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();

    Serial.print(F("red=")); Serial.print(redBuffer[i]);
    Serial.print(F(", ir=")); Serial.println(irBuffer[i]);
  }

  // First calculation
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
                                         &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Continue sampling
  while (1) {
    // Shift last 75 samples to beginning
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    // Read 25 new samples
    for (byte i = 75; i < 100; i++) {
      while (!particleSensor.available())
        particleSensor.check();

      digitalWrite(readLED, !digitalRead(readLED));

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();

      Serial.print(F("red=")); Serial.print(redBuffer[i]);
      Serial.print(F(", ir=")); Serial.print(irBuffer[i]);
      Serial.print(F(", HR=")); Serial.print(heartRate);
      Serial.print(F(", HRvalid=")); Serial.print(validHeartRate);
      Serial.print(F(", SPO2=")); Serial.print(spo2);
      Serial.print(F(", SPO2Valid=")); Serial.println(validSPO2);
    }

    // Recalculate
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
                                           &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}
