#include <Arduino.h>
#include "ESCOutput.h"

void escOutputSetupAndArm() {
  // Setup ESC Output
  analogWriteFrequency(ESC1_PIN, 400);
  analogWriteFrequency(ESC2_PIN, 400);
  analogWriteResolution(16);

  // Arming sequence
  analogWrite(ESC1_PIN, usToDuty(1040));
  analogWrite(ESC2_PIN, usToDuty(1040));
  delay(600);
  analogWrite(ESC1_PIN, usToDuty(1960));
  analogWrite(ESC2_PIN, usToDuty(1960));
  delay(600);
  analogWrite(ESC1_PIN, usToDuty(1040));
  analogWrite(ESC2_PIN, usToDuty(1040));
  delay(600);
}