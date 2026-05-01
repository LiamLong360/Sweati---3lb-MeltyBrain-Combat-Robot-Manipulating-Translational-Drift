#include "AccelH3LIS331.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define H3LIS331_SCK 13
#define H3LIS331_MISO 12
#define H3LIS331_MOSI 11
// Used for hardware & software SPI
#define H3LIS331_CS1 9
#define H3LIS331_CS2 10

Adafruit_H3LIS331 lis1 = Adafruit_H3LIS331();
Adafruit_H3LIS331 lis2 = Adafruit_H3LIS331();

// Distance from each accelerometer to the center of rotation in meters.
// Measure this from CAD for each sensor — if they're at equal radii use the same value.
// omega = sqrt(a / r)
// Heading lags  → omega too small → r is too large  → decrease SENSOR_RADIUS
// Heading leads → omega too large → r is too small  → increase SENSOR_RADIUS
static float SENSOR_RADIUS = 0.0508f;   // mutable — loaded from EEPROM at boot

// Minimum RPM before we trust the integrated angle.
// Below this the centripetal signal is too weak / noisy.
static const float MIN_OMEGA_DEGREES = 360.0f;   // 1 rev/s = 60 RPM

static float _omegaDegrees  = 0.0f;
static float _omegaDegreesPrev = 0.0f;   // previous step — needed for trapezoidal
static float _currentAngle  = 0.0f;
static uint32_t _lastAngleUs = 0;

static void configureSensor(Adafruit_H3LIS331 &lis, const char *name) {
  lis.setRange(H3LIS331_RANGE_100_G);
  lis.setDataRate(LIS331_DATARATE_1000_HZ);

  Serial.print(name); Serial.print(" range: ");
  switch (lis.getRange()) {
    case H3LIS331_RANGE_100_G: Serial.println("100 g"); break;
    case H3LIS331_RANGE_200_G: Serial.println("200 g"); break;
    case H3LIS331_RANGE_400_G: Serial.println("400 g"); break;
  }
  Serial.print(name); Serial.print(" data rate: ");
  switch (lis.getDataRate()) {
    case LIS331_DATARATE_POWERDOWN:       Serial.println("Powered Down"); break;
    case LIS331_DATARATE_50_HZ:           Serial.println("50 Hz"); break;
    case LIS331_DATARATE_100_HZ:          Serial.println("100 Hz"); break;
    case LIS331_DATARATE_400_HZ:          Serial.println("400 Hz"); break;
    case LIS331_DATARATE_1000_HZ:         Serial.println("1000 Hz"); break;
    case LIS331_DATARATE_LOWPOWER_0_5_HZ: Serial.println("0.5 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_1_HZ:   Serial.println("1 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_2_HZ:   Serial.println("2 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_5_HZ:   Serial.println("5 Hz Low Power"); break;
    case LIS331_DATARATE_LOWPOWER_10_HZ:  Serial.println("10 Hz Low Power"); break;
  }
}

void accelSetSensorSeparation(float r) {
  SENSOR_RADIUS = r;
}

float accelGetSensorSeparation() {
  return SENSOR_RADIUS;
}

bool accelBegin() {
  Serial.println("Initialising accelerometers...");

  if (!lis1.begin_SPI(H3LIS331_CS1)) {
    Serial.println("Accel A not found!");
    while (1) yield();
  }
  Serial.println("Accel A found!");
  configureSensor(lis1, "A");

  if (!lis2.begin_SPI(H3LIS331_CS2)) {
    Serial.println("Accel B not found!");
    while (1) yield();
  }
  Serial.println("Accel B found!");
  configureSensor(lis2, "B");

  return true;
}

float accelGetOmegaDegrees() {
  return _omegaDegrees;
}

float accelGetCurrentAngle() {
  return _currentAngle;
}

void accelTick() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 1) return;   // cap at 1000 Hz (sensor max)
  lastMs = now;

  sensors_event_t event1, event2;
  lis1.getEvent(&event1);
  lis2.getEvent(&event2);

  // Centripetal magnitude for each sensor: sqrt(ax^2 + ay^2)
  // Using both axes makes the result independent of chip orientation in the spin plane.
  float mag1 = sqrtf(event1.acceleration.x * event1.acceleration.x +
                     event1.acceleration.y * event1.acceleration.y);
  float mag2 = sqrtf(event2.acceleration.x * event2.acceleration.x +
                     event2.acceleration.y * event2.acceleration.y);

  // Per-sensor absolute method: a = omega^2 * r  →  omega = sqrt(a / r)
  // Average the two to reduce noise. Both sensors assumed at equal radius
  // from center of rotation (SENSOR_RADIUS). Adjust SENSOR_RADIUS via
  // calibration mode until heading tracks correctly.
  float omega1 = (mag1 > 0.01f) ? sqrtf(mag1 / SENSOR_RADIUS) : 0.0f;
  float omega2 = (mag2 > 0.01f) ? sqrtf(mag2 / SENSOR_RADIUS) : 0.0f;
  float omegaRad = (omega1 + omega2) / 2.0f;

  // Convert to degrees
  _omegaDegrees = omegaRad * (180.0f / PI);

  // Debug at ~5 Hz
  static uint32_t lastDebugMs = 0;
  if (now - lastDebugMs >= 200) {
    lastDebugMs = now;
    float rpm = omegaRad * 60.0f / (2.0f * PI);
    Serial.print("omega: "); Serial.print(_omegaDegrees);
    Serial.print(" deg/s  RPM: "); Serial.print(rpm);
    Serial.print("  angle: "); Serial.println(_currentAngle);
  }
}

void accelUpdateAngle() {
  // Only integrate when spinning fast enough for a reliable signal
  if (_omegaDegrees < MIN_OMEGA_DEGREES) {
    // Not spinning — reset angle so heading starts fresh on next spin-up
    _currentAngle = 0.0f;
    _omegaDegreesPrev = 0.0f;   // clean initial condition for trapezoidal on re-entry
    _lastAngleUs  = micros();
    return;
  }

  uint32_t nowUs = micros();
  float dtSeconds = (nowUs - _lastAngleUs) / 1e6f;
  _lastAngleUs = nowUs;

  // Trapezoidal integration: average omega at start and end of step.
  // Error per step is O(dt^3) vs Euler's O(dt^2) — one order better —
  // for the cost of one extra addition and a multiply.
  // During spin-up/down where omega is changing, Euler accumulates error
  // in the same direction every step. Trapezoidal largely cancels it.
  _currentAngle += ((_omegaDegreesPrev + _omegaDegrees) / 2.0f) * dtSeconds;
  _omegaDegreesPrev = _omegaDegrees;

  // Wrap to 0–360
  while (_currentAngle >= 360.0f) {
    _currentAngle -= 360.0f;
  }
  while (_currentAngle <    0.0f) {
    _currentAngle += 360.0f;
  }
}