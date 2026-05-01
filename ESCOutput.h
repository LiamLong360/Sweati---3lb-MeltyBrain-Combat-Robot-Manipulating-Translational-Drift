#pragma once
#include <Arduino.h>

// Pin assignments
#define ESC1_PIN 1
#define ESC2_PIN 8

// Helper to convert microseconds to a 16-bit duty cycle at 400Hz
// Period at 400Hz = 2500us
// duty = (pulse_us / 2500us) * 65535
// inline means instead of doing normal function call mechanism (save registers,
// jump to function address, return), it should just copy & paste the function's
// code directly in where it's called
inline uint16_t usToDuty(uint16_t us) {
  return (uint32_t)us * 65535UL / 2500UL;
}
void escOutputSetupAndArm();