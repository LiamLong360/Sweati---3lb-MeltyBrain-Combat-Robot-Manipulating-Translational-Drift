#include "NeoPixels.h"
#include <WS2812Serial.h>

// LED CONFIG
constexpr int LED_COUNT = 60;
constexpr int LED_PIN   = 14;   // MUST be a WS2812Serial-supported pin
constexpr int COLOR_ORDER = WS2812_GRB;

// DMA buffers (required)
DMAMEM byte displayMemory[LED_COUNT * 12];
byte drawingMemory[LED_COUNT * 3];

WS2812Serial leds(
  LED_COUNT,
  displayMemory,
  drawingMemory,
  LED_PIN,
  COLOR_ORDER
);

static uint32_t makeColor(uint8_t r, uint8_t g, uint8_t b) {
  return (r << 16) | (g << 8) | b;
}

void ledsBegin() {
  leds.begin();
  leds.show(); // clear
}

static bool s_calLatchSet = false;

void ledsSetCalibration() {
  // Solid magenta — indicates calibration mode is active.
  // Latched so we don't hammer DMA every loop iteration.
  if (s_calLatchSet) {
    return;
  }

  s_calLatchSet = true;
  uint32_t magenta = makeColor(40, 0, 40);
  for (int i = 0; i < LED_COUNT; i++)
    leds.setPixel(i, magenta);
  leds.show();
}

void ledsSetLocked() {
  // Solid blue — indicates watchdog timer has been counted down.
  // Latched so we don't hammer DMA every loop iteration.
  if (s_calLatchSet) {
    return;
  }

  s_calLatchSet = true;
  uint32_t blue = makeColor(0, 0, 40);
  for (int i = 0; i < LED_COUNT; i++)
    leds.setPixel(i, blue);
  leds.show();
}

void ledsSetIdle() {
  s_calLatchSet = false;   // reset so ledsSetCalibration works on next entry
  for (int i = 0; i < LED_COUNT; i++)
    leds.setPixel(i, makeColor(0, 0, 10));
  leds.show();
}

void ledsUpdateFromThrottle(int us) {
  uint32_t color;
  uint32_t color2;

  if (us < 1055) {
    color = makeColor(40, 24, 0); // yellow
    color2 = makeColor(0, 0, 40); // blue
  } else if (us < 1200) {
    color = makeColor(0, 0, 40); // blue
  } else if (us < 1600) {
    color = makeColor(0, 40, 0); // green
  } else {
    color = makeColor(40, 0, 0); // red
  }

  static uint32_t lastColor = 0;
  if (color == lastColor) return;    // avoid unnecessary updates
  lastColor = color;

  for (int i = 0; i < LED_COUNT; i++) {
    if (us < 1055) {
      leds.setPixel(i, (i % 2 == 0) ? color : color2);
    } else {
      leds.setPixel(i, color);
    }
  }

  leds.show();   // DMA-driven, minimal timing impact
}

void ledsUpdateFromHeading(float angle, float headingOffset) {
  // Shift the raw angle by the driver's heading offset (saved aileron inputs), keep in 0–360
  float adjusted = angle + headingOffset;
  while (adjusted >= 360.0f) {
    adjusted -= 360.0f;
  }
  while (adjusted < 0.0f) {
    adjusted += 360.0f;
  }

  uint32_t onColor  = makeColor(0, 40, 0);   // green — forward half
  uint32_t offColor = makeColor(40, 0, 0);   // red   — rear half

  uint32_t color = (adjusted < 180.0f) ? onColor : offColor;

  for (int i = 0; i < LED_COUNT; i++) {
    leds.setPixel(i, color);
  }
  leds.show();

}
