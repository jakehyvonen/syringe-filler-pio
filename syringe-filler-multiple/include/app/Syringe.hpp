/**
 * @file Syringe.hpp
 * @brief Syringe roles, calibration data, and runtime state.
 */
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
    float ratio     = 0.0f; // V_channel / V_ref (0..1)
  };

  static constexpr uint8_t kMaxPoints = 17;

  // Single-span fields (kept for compatibility / convenience)
  uint16_t adcEmpty = 0;     // raw ADC at 0 mL
  uint16_t adcFull  = 4095;  // raw ADC at max mL
  float    mlFull   = 10.1f; // how many mL does adcFull mean?

  // Current multi-point model
  uint8_t          pointCount = 0;
  CalibrationPoint points[kMaxPoints];
  float            steps_mL   = 1.01f; // calibrated steps per mL

  // Add or update a calibration point for the given ratio.
  bool addPoint(float volume_ml, float ratio) {
    if (!isfinite(volume_ml) || !isfinite(ratio)) return false;
    constexpr float kEps = 1e-4f;

    // Update existing point if ratio matches closely
    for (uint8_t i = 0; i < pointCount; ++i) {
      if (fabsf(points[i].ratio - ratio) <= kEps) {
        points[i].volume_ml = volume_ml;
        return true;
      }
    }

    if (pointCount >= kMaxPoints) return false;

    // Insert sorted by ratio (ascending)
    uint8_t insertAt = pointCount;
    while (insertAt > 0 && ratio < points[insertAt - 1].ratio) {
      points[insertAt] = points[insertAt - 1];
      --insertAt;
    }

    points[insertAt].volume_ml = volume_ml;
    points[insertAt].ratio     = ratio;
    ++pointCount;
    return true;
  }

  // Convert a ratio to milliliters using a least-squares quadratic fit.
  // Falls back to a line fit when only two points are available.
  float ratioToMl(float ratio) const {
    if (pointCount == 0) return 0.0f;
    if (pointCount == 1) return points[0].volume_ml;

    double s1 = 0.0;
    double sx = 0.0;
    double sx2 = 0.0;
    double sx3 = 0.0;
    double sx4 = 0.0;
    double sy = 0.0;
    double sxy = 0.0;
    double sx2y = 0.0;

    for (uint8_t i = 0; i < pointCount; ++i) {
      const double x = points[i].ratio;
      const double y = points[i].volume_ml;
      const double x2 = x * x;
      s1 += 1.0;
      sx += x;
      sx2 += x2;
      sx3 += x2 * x;
      sx4 += x2 * x2;
      sy += y;
      sxy += x * y;
      sx2y += x2 * y;
    }

    const double det =
      s1 * (sx2 * sx4 - sx3 * sx3) -
      sx * (sx * sx4 - sx3 * sx2) +
      sx2 * (sx * sx3 - sx2 * sx2);

    constexpr double kDetEps = 1e-10;
    if (fabs(det) <= kDetEps) {
      const double denom = (s1 * sx2) - (sx * sx);
      if (fabs(denom) <= kDetEps) return points[0].volume_ml;
      const double b = ((s1 * sxy) - (sx * sy)) / denom;
      const double c = (sy - b * sx) / s1;
      return (float)(b * ratio + c);
    }

    const double detA =
      sy * (sx2 * sx4 - sx3 * sx3) -
      sx * (sxy * sx4 - sx3 * sx2y) +
      sx2 * (sxy * sx3 - sx2 * sx2y);
    const double detB =
      s1 * (sxy * sx4 - sx3 * sx2y) -
      sy * (sx * sx4 - sx3 * sx2) +
      sx2 * (sx * sx2y - sxy * sx2);
    const double detC =
      s1 * (sx2 * sx2y - sxy * sx3) -
      sx * (sx * sx2y - sxy * sx2) +
      sy * (sx * sx3 - sx2 * sx2);

    const double a = detA / det;
    const double b = detB / det;
    const double c = detC / det;
    return (float)(a * ratio * ratio + b * ratio + c);
  }
};

struct CalibrationPoint {
  float volumeMl = 0.0f;
  float ratio = 0.0f;
};

struct CalibrationPoints {
  static constexpr uint8_t kMaxPoints = 17;
  CalibrationPoint points[kMaxPoints];
  uint8_t count = 0;
};

// ----------------------------------------------------
// Syringe (runtime object)
// ----------------------------------------------------
struct Syringe {
  uint32_t     rfid      = 0;                 // unique tag ID
  SyringeRole  role      = SyringeRole::Unknown;
  PotCalibration cal;                  // loaded from NVS (per RFID)
  CalibrationPoints calPoints;         // ratiometric calibration points
  float        currentMl = 0.0f;       // last measured volume
  uint8_t      slot      = 0;          // which base position (for base syringes)
  String       colorName;              // e.g. "Cobalt Blue" (optional)
  String       colorHex;               // e.g. "#0047AB" (optional)

  // Clear stored color metadata.
  void clearColor() {
    colorName = "";
    colorHex  = "";
  }

  // Return true when any color metadata is present.
  bool hasColor() const {
    return colorName.length() > 0 || colorHex.length() > 0;
  }

  // Print the syringe state to the provided stream.
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
