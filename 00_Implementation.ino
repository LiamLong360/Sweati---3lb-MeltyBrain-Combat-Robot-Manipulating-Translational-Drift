#include <Arduino.h>
#include <EEPROM.h>
#include "NeoPixels.h"
#include "AccelH3LIS331.h"
#include "RXInput.h"
#include "ESCOutput.h"

// EEPROM layout
// Address 0: magic number (uint32_t) — if 0xDEADBEEF, EEPROM has been written
// Address 4: SENSOR_SEPARATION (float)
static const uint32_t EEPROM_MAGIC        = 0xDEADBEEF;
static const int      EEPROM_ADDR_MAGIC   = 0;
static const int      EEPROM_ADDR_SEP     = 4;

// Default sensor separation in meters (2.0 in = 0.0508 m).
// Overwritten by EEPROM value on boot if EEPROM has been written before.
static const float SENSOR_SEPARATION_DEFAULT = 0.0508f;

// Calibration mode
// Enter: throttle at idle AND elevon held at max for 3 seconds
// Exit:  elevon at min
// While active: aileron left/right adjusts SENSOR_SEPARATION and saves to EEPROM

enum RobotState {
  STATE_NORMAL,
  STATE_CALIBRATION,
  STATE_LOCKED          // safety watchdog — requires power cycle to exit
};

static RobotState robotState = STATE_NORMAL;

// SAFETY WATCHDOG
// If no RC channel changes by more than WATCHDOG_DELTA_US for
// WATCHDOG_TIMEOUT_MS, the robot locks: motors idle, LEDs solid
// blue. Only a full power cycle can clear this state.
static const uint32_t WATCHDOG_TIMEOUT_MS = 30000;   // 30 seconds
static const uint16_t WATCHDOG_DELTA_US   =   100;   // 100µs change threshold

static uint32_t watchdogLastChangeMs = 0;  // timestamp of last significant RC change
static uint16_t watchdogLastThr      = 0;  // previous throttle snapshot
static uint16_t watchdogLastElv      = 0;  // previous elevon snapshot
static uint16_t watchdogLastAil      = 0;  // previous aileron snapshot

// How long elevon must be held at max to enter calibration (ms)
static const uint32_t CAL_ENTRY_HOLD_MS = 3000;

// Aileron adjusts separation at this rate (meters per second at full stick)
static const float SEP_ADJUST_RATE = 0.001f;   // 1mm/s at full stick

// Aileron stick centre and deadband
static const uint16_t AIL_CENTER   = 1500;
static const uint16_t AIL_DEADBAND =   30;

// Elevon thresholds for calibration entry/exit
static const uint16_t ELV_MAX_THRESHOLD  = 1900;   // "elevon max" = above this
static const uint16_t ELV_MIN_THRESHOLD  = 1100;   // "elevon min" = below this
static const uint16_t THR_IDLE_THRESHOLD = 1100;   // "throttle idle" = below this

// Heading offset
// The driver trims this left/right with the aileron stick until the LED beacon
// appears to point away from them — that direction becomes "forward".
// Units: degrees. Accumulated every loop, wraps 0–360.
static float headingOffset = 0.0f;

// How many degrees per second the aileron stick can shift the heading offset.
// Tune this so a full stick deflection feels responsive but not twitchy.
static const float HEADING_TRIM_RATE = 180.0f;   // deg/s at full stick

// EEPROM helpers
static void eepromSaveSeparation(float sep) {
  uint32_t magic = EEPROM_MAGIC;
  EEPROM.put(EEPROM_ADDR_MAGIC, magic);
  EEPROM.put(EEPROM_ADDR_SEP,   sep);
  Serial.print("EEPROM saved SENSOR_SEPARATION = "); 
  Serial.println(sep, 6);
}

static float eepromLoadSeparation() {
  uint32_t magic = 0;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);
  if (magic == EEPROM_MAGIC) {
    float sep = 0.0f;
    EEPROM.get(EEPROM_ADDR_SEP, sep);
    // Sanity check — reject obviously garbage values
    if (sep > 0.005f && sep < 0.5f) {
      Serial.print("EEPROM loaded SENSOR_SEPARATION = "); 
      Serial.println(sep, 6);
      return sep;
    }
  }
  Serial.print("EEPROM empty or invalid — using default: ");
  Serial.println(SENSOR_SEPARATION_DEFAULT, 6);
  return SENSOR_SEPARATION_DEFAULT;
}

// Setup 
void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial.println("---- Sequence Begin ----");

  // Load sensor separation from EEPROM (or default if first boot)
  float sep = eepromLoadSeparation();
  accelSetSensorSeparation(sep);

  // Setup Analogwrite & arm ESCs
  escOutputSetupAndArm();

  // Rx
  rxSetup();

  // LEDs (DMA-driven, non-blocking)
  ledsBegin();
  ledsSetIdle();

  // Accelerometer
  accelBegin();

  // Seed watchdog with initial RC values and current time so the
  // 30-second countdown starts from the moment setup completes.
  {
    noInterrupts();
    watchdogLastThr = throttleRaw;
    watchdogLastElv = elevonRaw;
    watchdogLastAil = aileronRaw;
    interrupts();
  }
  watchdogLastChangeMs = millis();

  Serial.println("---- Ready ----");
}

void loop() {

  // dt for heading trim and calibration adjust rates
  static uint32_t lastLoopUs = 0;
  uint32_t nowUs = micros();
  float dtSeconds = (nowUs - lastLoopUs) / 1e6f;
  lastLoopUs = nowUs;

  // Atomic RC snapshot
  noInterrupts();
  uint16_t thr = throttleRaw;
  uint16_t elv = elevonRaw;
  uint16_t ail = aileronRaw;
  interrupts();

  // SAFETY WATCHDOG
  // STATE_LOCKED is a roach-motel: once entered it cannot be
  // exited without a full power cycle. No code path below will
  // run — we return immediately after killing the motors.
  if (robotState == STATE_LOCKED) {
    analogWrite(ESC1_PIN, usToDuty(1000));
    analogWrite(ESC2_PIN, usToDuty(1000));
    ledsSetLocked();            // solid blue
    return;
  }

  // Check for significant RC activity to keep the watchdog alive.
  // Any channel changing by >= WATCHDOG_DELTA_US resets the timer.
  bool rcActive =
    (abs((int)thr - (int)watchdogLastThr) >= WATCHDOG_DELTA_US) ||
    (abs((int)elv - (int)watchdogLastElv) >= WATCHDOG_DELTA_US) ||
    (abs((int)ail - (int)watchdogLastAil) >= WATCHDOG_DELTA_US);

  if (rcActive) {
    watchdogLastChangeMs = millis();
    watchdogLastThr      = thr;
    watchdogLastElv      = elv;
    watchdogLastAil      = ail;
  } else if ((millis() - watchdogLastChangeMs) >= WATCHDOG_TIMEOUT_MS) {
    // 30 seconds of inactivity — lock the robot
    robotState = STATE_LOCKED;
    analogWrite(ESC1_PIN, usToDuty(1000));
    analogWrite(ESC2_PIN, usToDuty(1000));
    ledsSetLocked();            // solid blue
    Serial.println("!!! WATCHDOG TIMEOUT — ROBOT LOCKED. POWER CYCLE TO RESET !!!");
    return;
  }

  // Accelerometer read + angle integration
  accelTick();
  accelUpdateAngle();
  float currentAngle = accelGetCurrentAngle();
  float omegaDeg     = accelGetOmegaDegrees();

  // Calibration state machine
  static uint32_t elvMaxStartMs = 0;

  if (robotState == STATE_NORMAL) {

    // Entry condition: throttle idle + elevon max held for 3s
    bool thrIdle  = (thr < THR_IDLE_THRESHOLD);
    bool elvAtMax = (elv > ELV_MAX_THRESHOLD);

    if (thrIdle && elvAtMax) {
      if (elvMaxStartMs == 0) {
        elvMaxStartMs = millis();
      } else if (millis() - elvMaxStartMs >= CAL_ENTRY_HOLD_MS) {
        robotState    = STATE_CALIBRATION;
        elvMaxStartMs = 0;
        Serial.println(">>> CALIBRATION MODE ENTERED <<<");
        Serial.print("Current SENSOR_SEPARATION = ");
        Serial.println(accelGetSensorSeparation(), 6);
        ledsSetCalibration();   // solid magenta
      }
    } else {
      elvMaxStartMs = 0;        // reset hold timer if conditions break
    }

  } else {
    // STATE_CALIBRATION 

    // Exit: elevon pulled to min
    if (elv < ELV_MIN_THRESHOLD) {
      robotState = STATE_NORMAL;
      Serial.println(">>> CALIBRATION MODE EXITED <<<");
      ledsSetIdle();
    } else {
      // Aileron trims SENSOR_SEPARATION up/down
      int16_t ailCentered = (int16_t)ail - AIL_CENTER;
      if (abs(ailCentered) > AIL_DEADBAND) {
        float ailNorm    = constrain(ailCentered / 500.0f, -1.0f, 1.0f);
        float currentSep = accelGetSensorSeparation();
        float newSep     = currentSep + ailNorm * SEP_ADJUST_RATE * dtSeconds;
        newSep           = constrain(newSep, 0.005f, 0.5f);
        accelSetSensorSeparation(newSep);
        eepromSaveSeparation(newSep);
      }

      // Motors at idle — robot must not be spinning during calibration
      analogWrite(ESC1_PIN, usToDuty(1040));
      analogWrite(ESC2_PIN, usToDuty(1040));
      ledsSetCalibration();
      debugPrint(thr, elv, ail);
      return;   // skip all normal mode logic below
    }
  }

  //  Normal mode 

  // Heading offset trim from aileron
  int16_t ailCentered = (int16_t)ail - AIL_CENTER;
  if (abs(ailCentered) > AIL_DEADBAND) {
    float ailNorm = constrain(ailCentered / 500.0f, -1.0f, 1.0f);
    headingOffset -= ailNorm * HEADING_TRIM_RATE * dtSeconds;
    while (headingOffset >= 360.0f) headingOffset -= 360.0f;
    while (headingOffset <    0.0f) headingOffset += 360.0f;
  }

  // Motor mix
  if (omegaDeg > 360.0f) {
    float headingError = currentAngle - headingOffset;
    while (headingError >  180.0f) headingError -= 360.0f;
    while (headingError < -180.0f) headingError += 360.0f;

    // elevon: 1500us = zero, 2000us = full forward, 1000us = full backward
    float translationScale = constrain(
      (float)((int16_t)elv - 1500) / 500.0f, -1.0f, 1.0f);

    // cos: max boost when headingError = 0 (pointed at target), zero at ±90°
    float mix = cosf(headingError * PI / 180.0f) * translationScale * 200.0f;

    uint16_t motor1 = (uint16_t)constrain((int)thr + (int)mix, 1016, 2016);
    uint16_t motor2 = (uint16_t)constrain((int)thr - (int)mix, 1016, 2016);
    analogWrite(ESC1_PIN, usToDuty(motor1));
    analogWrite(ESC2_PIN, usToDuty(motor2));

  } else {
    // Not spinning fast enough — passthrough, no mix
    uint16_t motor1 = (uint16_t)constrain(thr, 1016, 2016);
    uint16_t motor2 = (uint16_t)constrain(thr, 1016, 2016);
    analogWrite(ESC1_PIN, usToDuty(motor1));
    analogWrite(ESC2_PIN, usToDuty(motor2));
  }

  // LED update
  if (omegaDeg > 360.0f) {
    ledsUpdateFromHeading(currentAngle, headingOffset);
  } else {
    ledsUpdateFromThrottle(thr);
  }

  debugPrint(thr, elv, ail);
}

