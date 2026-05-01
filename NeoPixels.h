#pragma once
#include <Arduino.h>

void ledsBegin();
void ledsSetIdle();
void ledsSetCalibration();
void ledsSetLocked();
void ledsUpdateFromThrottle(int us);

// Flashes LEDs for the forward half of rotation.
// angle: current integrated angle 0–360 degrees
// headingOffset: driver-adjusted offset, also 0–360 degrees
void ledsUpdateFromHeading(float angle, float headingOffset);