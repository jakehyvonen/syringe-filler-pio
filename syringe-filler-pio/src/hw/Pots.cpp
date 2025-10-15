#include "hw/Pots.hpp"
#include "hw/Drivers.hpp"
#include "hw/Pins.hpp"
#include <Arduino.h>

namespace Pots {

static constexpr uint8_t POT_EMA_SHIFT     = 3;   // 1/8 new sample
static constexpr uint8_t POT_REPORT_HYST   = 3;   // print only if big change
static constexpr uint8_t POT_PINS[NUM_POTS] = { 0 }; // ADS1115 ch 0

static bool inited = false;
static uint16_t pot_raw[NUM_POTS];
static uint16_t pot_filt[NUM_POTS];
static uint16_t pot_last_reported[NUM_POTS];

static inline uint16_t readScaledADC(uint8_t potIndex) {
  if (!Drivers::hasADS()) return 0;

  int ch = POT_PINS[potIndex];
  int16_t v = Drivers::ADS.readADC_SingleEnded(ch);
  if (v < 0) v = 0;
  // ADS1115 GAIN_ONE: 0..32767 for 0..+4.096V
  uint32_t scaled = (uint32_t)v * 1023UL / 32767UL;
  if (scaled > 1023UL) scaled = 1023UL;
  return (uint16_t)scaled;
}

void init() {
  if (!Drivers::hasADS()) {
    for (uint8_t i = 0; i < NUM_POTS; ++i) {
      pot_raw[i] = pot_filt[i] = pot_last_reported[i] = 0;
    }
    inited = true;
    Serial.println("Pots: ADS not present; readings disabled.");
    return;
  }

  // Seed
  for (uint8_t i = 0; i < NUM_POTS; ++i) {
    uint16_t r = readScaledADC(i);
    pot_raw[i]  = r;
    pot_filt[i] = r;
  }
  // Quick settle
  for (uint8_t k = 0; k < 8; ++k) {
    for (uint8_t i = 0; i < NUM_POTS; ++i) {
      uint16_t r = readScaledADC(i);
      pot_filt[i] = (pot_filt[i] + r) >> 1;
    }
    delay(2);
  }
  for (uint8_t i = 0; i < NUM_POTS; ++i) {
    pot_last_reported[i] = pot_filt[i];
    Serial.print("pot["); Serial.print(i); Serial.print("](init)=");
    Serial.println(pot_filt[i]);
  }
  inited = true;
}

void poll() {
  if (!inited) init();
  if (!Drivers::hasADS()) return;

  for (uint8_t i = 0; i < NUM_POTS; ++i) {
    uint16_t r = readScaledADC(i);
    pot_raw[i] = r;
    pot_filt[i] += (int16_t(r) - int16_t(pot_filt[i])) >> POT_EMA_SHIFT;

    int diff = int(pot_filt[i]) - int(pot_last_reported[i]);
    if (diff < 0) diff = -diff;
    if (diff >= POT_REPORT_HYST) {
      pot_last_reported[i] = pot_filt[i];
      Serial.print("pot["); Serial.print(i); Serial.print("]=");
      Serial.println(pot_filt[i]);
    }
  }
}

uint16_t raw(uint8_t i) {
  if (i >= NUM_POTS) return 0;
  return pot_raw[i];
}

uint16_t filt(uint8_t i) {
  if (i >= NUM_POTS) return 0;
  return pot_filt[i];
}

float percent(uint8_t i) {
  if (i >= NUM_POTS) return 0.0f;
  return (pot_filt[i] * 100.0f) / 1023.0f;
}

uint16_t readScaled(uint8_t i) {
  if (i >= NUM_POTS) return 0;
  return readScaledADC(i);
}

} // namespace Pots
