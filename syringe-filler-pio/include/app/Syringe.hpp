#pragma once
#include <Arduino.h>

namespace App {

// ----------------------------------------------------
// Roles
// ----------------------------------------------------
enum class SyringeRole : uint8_t {
  Unknown = 0,
  Base,
  Toolhead
};

// ----------------------------------------------------
// Calibration (persisted in NVS)
// ----------------------------------------------------
struct PotCalibration {
  uint16_t adcEmpty = 0;     // raw ADC at 0 mL
  uint16_t adcFull  = 4095;  // raw ADC at max mL
  float    mlFull   = 10.1f; // how many mL does adcFull mean?

  float rawToMl(uint16_t raw) const {
    int32_t span = (int32_t)adcFull - (int32_t)adcEmpty;
    if (span <= 0) return 0.0f;
    float t = ((int32_t)raw - (int32_t)adcEmpty) / (float)span;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t * mlFull;
  }
};

// ----------------------------------------------------
// Syringe (runtime object)
// ----------------------------------------------------
struct Syringe {
  uint32_t     rfid      = 0;          // unique tag ID
  SyringeRole  role      = SyringeRole::Unknown;
  PotCalibration cal;                  // loaded from NVS (per RFID)
  float        currentMl = 0.0f;       // last measured volume
  uint8_t      slot      = 0;          // which base position (for base syringes)
  String       colorName;              // e.g. "Cobalt Blue" (optional)
  String       colorHex;               // e.g. "#0047AB" (optional)

  void clearColor() {
    colorName = "";
    colorHex  = "";
  }

  bool hasColor() const {
    return colorName.length() > 0 || colorHex.length() > 0;
  }

  // Optional: compact summary for debug prints
  void printTo(Stream& s) const {
    s.printf("[Syringe] role=%s slot=%u RFID=0x%08lX vol=%.2f mL ",
             (role == SyringeRole::Base ? "Base" :
              role == SyringeRole::Toolhead ? "Toolhead" : "Unknown"),
             slot, rfid, currentMl);
    if (hasColor()) {
      s.print("color=");
      if (colorName.length()) s.print(colorName);
      if (colorHex.length()) {
        if (colorName.length()) s.print(" ");
        s.print("(");
        s.print(colorHex);
        s.print(")");
      }
    } else {
      s.print("color=<none>");
    }
    s.println();
  }
};

} // namespace App
