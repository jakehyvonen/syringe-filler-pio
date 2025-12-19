#pragma once
#include <Arduino.h>
#include <math.h>

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
  struct CalibrationPoint {
    float volume_ml = 0.0f;
    float ratio = 0.0f;
  };

  static constexpr uint8_t kMaxPoints = 8;

  uint16_t adcEmpty = 0;     // raw ADC at 0 mL
  uint16_t adcFull  = 4095;  // raw ADC at max mL
  float    mlFull   = 10.1f; // how many mL does adcFull mean?
  float    steps_mL = 1.01f; //calibrated steps per mL
  bool     legacy   = false;
  struct CalibrationPoint {
    float volume_ml = 0.0f;
    float ratio     = 0.0f; // V_channel / V_ref (0..1 or 0..100)
  };

  static constexpr uint8_t kMaxPoints = 8;
  uint8_t         pointCount = 0;
  CalibrationPoint points[kMaxPoints];
  float           steps_mL = 1.01f; // calibrated steps per mL

  bool addPoint(float volume_ml, float ratio) {
    if (!isfinite(volume_ml) || !isfinite(ratio)) return false;
    constexpr float kEps = 1e-4f;
    for (uint8_t i = 0; i < pointCount; ++i) {
      if (fabsf(points[i].ratio - ratio) <= kEps) {
        points[i].volume_ml = volume_ml;
        return true;
      }
    }
    if (pointCount >= kMaxPoints) return false;
    uint8_t insertAt = pointCount;
    while (insertAt > 0 && ratio < points[insertAt - 1].ratio) {
      points[insertAt] = points[insertAt - 1];
      --insertAt;
    }
    points[insertAt] = {volume_ml, ratio};
    ++pointCount;
    return true;
  }

  float ratioToMl(float ratio) const {
    if (pointCount == 0) return 0.0f;
    if (pointCount == 1) return points[0].volume_ml;
    if (ratio <= points[0].ratio) return points[0].volume_ml;
    for (uint8_t i = 1; i < pointCount; ++i) {
      if (ratio <= points[i].ratio) {
        const float r0 = points[i - 1].ratio;
        const float r1 = points[i].ratio;
        const float v0 = points[i - 1].volume_ml;
        const float v1 = points[i].volume_ml;
        if (fabsf(r1 - r0) <= 1e-6f) return v0;
        const float t = (ratio - r0) / (r1 - r0);
        return v0 + (v1 - v0) * t;
      }
    }
    return points[pointCount - 1].volume_ml;
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
