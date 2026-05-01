#include <Arduino.h>
#include "RXInput.h"

// Pin assignments
#define THROTTLE_PIN 0
#define ELEVON_PIN 2
#define AILERON_PIN 3

// 'volatile' because written in ISR, read in loop()
// Initialised to safe/low value so ESCs get a valid signal before RX locks on.
volatile uint16_t throttleRaw = 1000;
volatile uint16_t elevonRaw = 1500;
volatile uint16_t aileronRaw = 1500;

static uint32_t throttleRisingEdge = 0;
static uint32_t elevonRisingEdge = 0;
static uint32_t aileronRisingEdge = 0;

// Any value outside this range is just noise
static const uint16_t rxMin = 900;
static const uint16_t rxMax = 2100;

void rxSetup() {
  // RX input pins
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(ELEVON_PIN,   INPUT);
  pinMode(AILERON_PIN,  INPUT);

  // Set up ISRS
  // attachInterrupt(digitalPinToInterrupt(pin), ISR, mode)
  // throttleISR(), elevonISR(), and aileronISR() defined in RXInput
  // -> writing throttleISR, elevonISR, or aileronISR, gives direct memory address to throttleISR(), elevonISR(), and aileronISR() to begin executing
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELEVON_PIN),   elevonISR,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(AILERON_PIN),  aileronISR,  CHANGE);
}

void throttleISR() {
  if (digitalRead(THROTTLE_PIN)) { // if thorttle RX pin goes high (rising edge of PWM pulse)
    throttleRisingEdge = micros(); // take current time
  } else { 
    uint16_t width = (uint16_t)(micros() - throttleRisingEdge);
    if (width > rxMin && width < rxMax) {
      throttleRaw = width;
    }
  }
}

void elevonISR() {
  if (digitalRead(ELEVON_PIN)) {
    elevonRisingEdge = micros();
  } else {
    uint16_t width = (uint16_t)(micros() - elevonRisingEdge);
    if (width > rxMin && width < rxMax) {
      elevonRaw = width;
    }
  }
}

void aileronISR() {
  if (digitalRead(AILERON_PIN)) {
    aileronRisingEdge = micros();
  } else {
    uint16_t width = (uint16_t)(micros() - aileronRisingEdge);
    if (width > rxMin && width < rxMax) {
      aileronRaw = width;
    }
  }
}

// Debug receiver inputs
void debugPrint(uint16_t thr, uint16_t elv, uint16_t ail) {
  static uint32_t lastDebugMs = 0;
  uint32_t now = millis();
  if (now - lastDebugMs < 100) {
    return;   // print at 10 Hz, non-blocking
  }
  lastDebugMs = now;

  Serial.print("Throttle: "); Serial.print(thr);
  Serial.print("  Elevon: ");  Serial.print(elv);
  Serial.print("  Aileron: "); Serial.println(ail);
}