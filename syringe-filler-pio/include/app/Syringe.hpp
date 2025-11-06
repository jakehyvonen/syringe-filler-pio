#pragma once
#include <Arduino.h>

namespace App {

// just to tag what the syringe is doing in the system
enum class SyringeRole : uint8_t {
  Unknown = 0,
  Base,
  Toolhead
};

// this is the thing we persist in NVS (simple 2-point + capacity)
struct PotCalibration {
  uint16_t adcEmpty = 0;     // raw ADC at 0 mL
  uint16_t adcFull  = 4095;  // raw ADC at max mL
  float    mlFull   = 10.0f; // how many mL does adcFull mean?

  float rawToMl(uint16_t raw) const {
    int32_t span = (int32_t)adcFull - (int32_t)adcEmpty;
    if (span <= 0) return 0.0f;
    float t = ((int32_t)raw - (int32_t)adcEmpty) / (float)span;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t * mlFull;
  }
};

// this is your runtime object — what the controller works with
struct Syringe {
  uint32_t     rfid = 0;          // what we read from the tag
  SyringeRole  role = SyringeRole::Unknown;
  PotCalibration cal;             // loaded from NVS, per RFID
  float        currentMl = 0.0f;  // last measured by pot
  uint8_t      slot = 0;          // which base position, for base syringes
};

} // namespace App
