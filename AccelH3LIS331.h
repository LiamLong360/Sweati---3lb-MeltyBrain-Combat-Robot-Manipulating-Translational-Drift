#pragma once
#include <Arduino.h>

bool accelBegin();
void accelTick();

// Runtime calibration — called from EEPROM load on boot and calibration mode
void  accelSetSensorSeparation(float sep);
float accelGetSensorSeparation();

// Returns angular velocity in degrees/second
float accelGetOmegaDegrees();

// Integrates omega into a 0-360 angle. Call every loop after accelTick().
void  accelUpdateAngle();

// Returns current integrated heading angle (0–360 degrees)
float accelGetCurrentAngle();