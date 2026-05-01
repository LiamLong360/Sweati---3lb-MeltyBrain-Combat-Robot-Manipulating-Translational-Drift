#pragma once
#include <Arduino.h>

// Shared volatile RC values — written by ISRs, read in loop()
extern volatile uint16_t throttleRaw;
extern volatile uint16_t elevonRaw;
extern volatile uint16_t aileronRaw;

void rxSetup();

void throttleISR();
void elevonISR();
void aileronISR();

void debugPrint(uint16_t thr, uint16_t elv, uint16_t ail);