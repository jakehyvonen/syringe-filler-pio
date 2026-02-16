/**
 * @file SyringeCalibration.cpp
 * @brief Calibration routines for toolhead and base syringes.
 */
#include "app/SyringeCalibration.hpp"

#include "hw/Bases.hpp"
#include "hw/Pots.hpp"
#include "util/Storage.hpp"
#include <Arduino.h>
#include <math.h>

namespace App {

namespace {
  constexpr bool CAL_DBG = true;
  constexpr uint8_t TOOLHEAD_POT_IDX = 0;  // toolhead pot index

  // Sync legacy ADC fields from the current calibration points.
  void syncLegacyFields(PotCalibration& cal) {
    if (cal.pointCount == 0) return;
    cal.adcEmpty = Pots::countsFromRatio(cal.points[0].ratio);
    if (cal.pointCount > 1) {
      cal.adcFull = Pots::countsFromRatio(cal.points[cal.pointCount - 1].ratio);
      cal.mlFull  = cal.points[cal.pointCount - 1].volume_ml;
    } else {
      cal.adcFull = cal.adcEmpty;
      cal.mlFull  = cal.points[0].volume_ml;
    }
  }
}

// Build a calibration helper for toolhead and base syringes.
SyringeCalibration::SyringeCalibration(Syringe& toolhead,
                                       Syringe* bases,
                                       uint8_t baseCount,
                                       const uint8_t* baseToPot,
                                       int8_t& currentSlot)
  : m_toolhead(toolhead),
    m_bases(bases),
    m_baseCount(baseCount),
    m_baseToPot(baseToPot),
    m_currentSlot(currentSlot) {}

// Load or create calibration data for a base syringe tag.
void SyringeCalibration::initializeBaseFromTag(uint8_t slot, uint32_t tag) {
  if (slot >= m_baseCount) return;

  Syringe& s = m_bases[slot];
  s.rfid = tag;
  s.role = SyringeRole::Base;
  s.slot = slot;

  Util::BaseMeta meta;
  PotCalibration cal;
  App::CalibrationPoints points;
  if (Util::loadBase(tag, meta, cal, points)) {
    s.cal = cal;
    s.calPoints = points;
    if (CAL_DBG) {
      Serial.print("[SFC] scanBaseSyringe(): existing base loaded from NVS for tag 0x");
      Serial.println(tag, HEX);
    }
  } else {
    if (CAL_DBG) {
      Serial.print("[SFC] scanBaseSyringe(): NEW base, creating NVS entry for tag 0x");
      Serial.println(tag, HEX);
    }
    memset(&meta, 0, sizeof(meta));
    s.calPoints.count = 0;
    Util::saveBase(tag, meta, s.cal, s.calPoints);
  }
}

// Load toolhead calibration data for a tag if available.
bool SyringeCalibration::initializeToolheadFromTag(uint32_t tag) {
  m_toolhead.rfid = tag;
  m_toolhead.role = SyringeRole::Toolhead;

  PotCalibration cal;
  if (Util::loadCalibration(tag, cal)) {
    m_toolhead.cal = cal;
    if (CAL_DBG) {
      Serial.print("[SFC] loaded toolhead cal for 0x");
      Serial.println(tag, HEX);
    }
    return true;
  }

  if (CAL_DBG) {
    Serial.print("[SFC] no toolhead cal for 0x");
    Serial.println(tag, HEX);
  }
  return false;
}

// Resolve a base slot to its pot index.
int8_t SyringeCalibration::getBasePotIndex(uint8_t baseSlot) const {
  if (baseSlot >= m_baseCount) return -1;
  uint8_t p = m_baseToPot[baseSlot];
  return (p == 0xFF) ? -1 : (int8_t)p;
}

// Read the current toolhead pot ratio.
float SyringeCalibration::readToolheadRatio() {
  Pots::poll();
  float ratio = Pots::percent(TOOLHEAD_POT_IDX);
  if (CAL_DBG) {
    Serial.print("[SFC] readToolheadRatio(): pot=");
    Serial.print(TOOLHEAD_POT_IDX);
    Serial.print(" ratio=");
    Serial.println(ratio, 3);
  }
  return ratio;
}

// Capture a toolhead calibration point at the given volume.
bool SyringeCalibration::captureToolheadCalibrationPoint(float ml, String& message) {
  if (ml < 0.0f) {
    message = "volume must be >= 0";
    return false;
  }
  if (m_toolhead.rfid == 0) {
    message = "no toolhead RFID; scan the toolhead first";
    return false;
  }

  float ratio = readToolheadRatio();
  if (CAL_DBG) {
    uint16_t counts = Pots::readCounts(TOOLHEAD_POT_IDX);
    float percent = Pots::ratioFromCounts(counts);
    float normalized = percent / 100.0f;
    Serial.print("[SFC] cal.tool.point: toolhead RFID=0x");
    Serial.print(m_toolhead.rfid, HEX);
    Serial.print(" pot=");
    Serial.print(TOOLHEAD_POT_IDX);
    Serial.print(" counts=");
    Serial.print(counts);
    Serial.print(" ratioUsed=");
    Serial.print(ratio, 4);
    Serial.print(" ratioNormalized=");
    Serial.print(normalized, 4);
    Serial.print(" percent=");
    Serial.println(percent, 2);
  }
  bool ok = m_toolhead.cal.addPoint(ml, ratio);
  if (!ok) {
    message = (m_toolhead.cal.pointCount >= PotCalibration::kMaxPoints)
                ? "toolhead calibration point list full"
                : "failed to add toolhead calibration point";
    return false;
  }

  syncLegacyFields(m_toolhead.cal);
  bool saved = Util::saveCalibration(m_toolhead.rfid, m_toolhead.cal);
  if (!saved) {
    message = "failed to save toolhead syringe calibration point";
    return false;
  }
  if (m_toolhead.cal.pointCount < 2) {
    message = "toolhead syringe point saved; add at least 2 points to enable interpolation";
  } else {
    message = "toolhead syringe point saved";
  }
  return true;
}

// Set steps-per-mL for a base syringe slot.
bool SyringeCalibration::setBaseStepsPermL(uint8_t slot, float stepsPermL, String& message) {
  if (slot >= m_baseCount) {
    message = "base slot out of range";
    return false;
  }
  if (stepsPermL <= 0.0f) {
    message = "steps_mL must be > 0";
    return false;
  }

  Syringe& sy = m_bases[slot];
  if (sy.rfid == 0) {
    message = "base has no RFID; scan the base first";
    return false;
  }

  sy.cal.steps_mL = stepsPermL;
  Util::BaseMeta meta;
  App::PotCalibration cal;
  App::CalibrationPoints savedPoints;
  if (!Util::loadBase(sy.rfid, meta, cal, savedPoints)) {
    memset(&meta, 0, sizeof(meta));
  }
  (void)cal;
  (void)savedPoints;
  bool ok = Util::saveBase(sy.rfid, meta, sy.cal, sy.calPoints);
  message = ok ? "base steps_mL updated" : "failed to update base steps_mL";
  return ok;
}

// Clear calibration points for the current base slot.
bool SyringeCalibration::clearCurrentBaseCalibrationPoints(String& message) {
  if (m_currentSlot < 0 || m_currentSlot >= (int)m_baseCount) {
    message = "no current base";
    return false;
  }

  Syringe& sy = m_bases[m_currentSlot];
  if (sy.rfid == 0) {
    message = "base has no RFID; scan the base first";
    return false;
  }

  sy.calPoints = {};
  Util::BaseMeta meta;
  App::PotCalibration cal;
  App::CalibrationPoints savedPoints;
  if (!Util::loadBase(sy.rfid, meta, cal, savedPoints)) {
    memset(&meta, 0, sizeof(meta));
  }
  (void)cal;
  (void)savedPoints;
  bool ok = Util::saveBase(sy.rfid, meta, sy.cal, sy.calPoints);
  message = ok ? "base calibration points cleared" : "failed to clear base calibration points";
  return ok;
}

// Clear calibration points for the toolhead.
bool SyringeCalibration::clearToolheadCalibrationPoints(String& message) {
  if (m_toolhead.rfid == 0) {
    message = "no toolhead RFID; scan the toolhead first";
    return false;
  }

  m_toolhead.cal.pointCount = 0;
  m_toolhead.cal.adcEmpty = 0;
  m_toolhead.cal.adcFull = 0;
  m_toolhead.cal.mlFull = 0.0f;
  bool ok = Util::saveCalibration(m_toolhead.rfid, m_toolhead.cal);
  message = ok ? "toolhead calibration points cleared" : "failed to clear toolhead calibration points";
  return ok;
}

// Read the current base pot ratio for a slot.
float SyringeCalibration::readBaseRatio(uint8_t slot) {
  if (slot >= m_baseCount) {
    if (CAL_DBG) {
      Serial.print("[SFC] readBaseRatio(): slot OOR ");
      Serial.println(slot);
    }
    return 0.0f;
  }

  int8_t potIdx = getBasePotIndex(slot);
  if (potIdx < 0) {
    if (CAL_DBG) {
      Serial.print("[SFC] readBaseRatio(): no pot mapped for base ");
      Serial.println(slot);
    }
    return 0.0f;
  }

  Pots::poll();
  float ratio = Pots::percent((uint8_t)potIdx);

  if (CAL_DBG) {
    Serial.print("[SFC] readBaseRatio(base=");
    Serial.print(slot);
    Serial.print(" pot=");
    Serial.print(potIdx);
    Serial.print("): ratio=");
    Serial.print(ratio, 3);
  }

  return ratio;
}

// Read the base pot ratio and return it as 0..1.
bool SyringeCalibration::readBasePotRatio(uint8_t slot, float& ratio, String& message) const {
  if (slot >= m_baseCount) {
    message = "base slot out of range";
    return false;
  }

  int8_t potIdx = getBasePotIndex(slot);
  if (potIdx < 0) {
    message = "no pot mapped for base";
    return false;
  }

  Pots::poll();
  float percent = Pots::percent((uint8_t)potIdx);
  ratio = percent / 100.0f;
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  return true;
}

// Interpolate volume from calibration points using a ratio.
float SyringeCalibration::interpolateVolumeFromPoints(const App::CalibrationPoints& points, float ratio, bool& ok) {
  ok = false;
  if (points.count < 2) return NAN;

  if (ratio <= points.points[0].ratio) {
    ok = true;
    return points.points[0].volumeMl;
  }
  if (ratio >= points.points[points.count - 1].ratio) {
    ok = true;
    return points.points[points.count - 1].volumeMl;
  }

  for (uint8_t i = 0; i + 1 < points.count; ++i) {
    const CalibrationPoint& a = points.points[i];
    const CalibrationPoint& b = points.points[i + 1];
    if (ratio >= a.ratio && ratio <= b.ratio) {
      float denom = b.ratio - a.ratio;
      if (denom <= 0.0f) return NAN;
      float t = (ratio - a.ratio) / denom;
      ok = true;
      return a.volumeMl + t * (b.volumeMl - a.volumeMl);
    }
  }
  return NAN;
}

// Capture a base calibration point for a slot.
bool SyringeCalibration::captureBaseCalibrationPoint(uint8_t slot, float ml, String& message) {
  if (slot >= m_baseCount) {
    message = "base slot out of range";
    return false;
  }
  if (ml < 0.0f) {
    message = "volume must be >= 0";
    return false;
  }

  Syringe& sy = m_bases[slot];
  if (sy.rfid == 0) {
    message = "base has no RFID; scan the base first";
    return false;
  }

  float ratio = 0.0f;
  String ratioMessage;
  if (!readBasePotRatio(slot, ratio, ratioMessage)) {
    message = ratioMessage;
    return false;
  }
  if (CAL_DBG) {
    int8_t potIdx = getBasePotIndex(slot);
    uint16_t counts = (potIdx >= 0) ? Pots::readCounts((uint8_t)potIdx) : 0;
    float percent = Pots::ratioFromCounts(counts);
    Serial.print("[SFC] cal.base.point: base slot=");
    Serial.print(slot);
    Serial.print(" RFID=0x");
    Serial.print(sy.rfid, HEX);
    Serial.print(" pot=");
    Serial.print(potIdx);
    Serial.print(" counts=");
    Serial.print(counts);
    Serial.print(" ratio=");
    Serial.print(ratio, 4);
    Serial.print(" percent=");
    Serial.println(percent, 2);
  }

  CalibrationPoints& points = sy.calPoints;
  const float kRatioEpsilon = 0.0025f;
  for (uint8_t i = 0; i < points.count; ++i) {
    if (fabsf(points.points[i].ratio - ratio) <= kRatioEpsilon) {
      points.points[i].volumeMl = ml;
      Util::BaseMeta meta;
      App::PotCalibration cal;
      App::CalibrationPoints savedPoints;
      if (!Util::loadBase(sy.rfid, meta, cal, savedPoints)) {
        memset(&meta, 0, sizeof(meta));
      }
      (void)cal;
      (void)savedPoints;
      bool ok = Util::saveBase(sy.rfid, meta, sy.cal, sy.calPoints);
      message = ok ? "updated calibration point" : "failed to save calibration point";
      return ok;
    }
  }

  if (points.count >= CalibrationPoints::kMaxPoints) {
    message = "calibration point list full";
    return false;
  }

  uint8_t insertAt = 0;
  while (insertAt < points.count && points.points[insertAt].ratio < ratio) {
    insertAt++;
  }
  for (uint8_t i = points.count; i > insertAt; --i) {
    points.points[i] = points.points[i - 1];
  }
  points.points[insertAt].volumeMl = ml;
  points.points[insertAt].ratio = ratio;
  points.count++;

  Util::BaseMeta meta;
  App::PotCalibration cal;
  App::CalibrationPoints savedPoints;
  if (!Util::loadBase(sy.rfid, meta, cal, savedPoints)) {
    memset(&meta, 0, sizeof(meta));
  }
  (void)cal;
  (void)savedPoints;
  bool ok = Util::saveBase(sy.rfid, meta, sy.cal, sy.calPoints);
  if (!ok) {
    message = "failed to save calibration point";
    return false;
  }

  if (points.count < 2) {
    message = "point saved; add at least 2 points to enable interpolation";
  } else {
    message = "point saved";
  }
  return true;
}

// Print base calibration info for a slot.
void SyringeCalibration::printBaseInfo(uint8_t slot, Stream& s) {
  if (slot >= m_baseCount) {
    s.println("[SFC] base info: slot OOR");
    return;
  }
  const Syringe& sy = m_bases[slot];
  s.print("[SFC] base "); s.print(slot);
  s.print(" RFID=0x"); s.print(sy.rfid, HEX);
  s.print(" points="); s.print(sy.cal.pointCount);
  for (uint8_t i = 0; i < sy.cal.pointCount; ++i) {
    s.print(" [");
    s.print(i);
    s.print(":");
    s.print(sy.cal.points[i].volume_ml, 3);
    s.print("ml@");
    s.print(sy.cal.points[i].ratio, 3);
    s.print("]");
  }
  s.print(" steps_mL=");   s.print(sy.cal.steps_mL, 3);
  s.print(" calPoints="); s.print(sy.calPoints.count);
  if (sy.calPoints.count > 0) {
    s.print(" [");
    for (uint8_t i = 0; i < sy.calPoints.count; ++i) {
      if (i > 0) s.print(", ");
      s.print(sy.calPoints.points[i].volumeMl, 3);
      s.print("ml@");
      s.print(sy.calPoints.points[i].ratio * 100.0f, 2);
      s.print("%");
    }
    s.print("]");
  }
  s.println();
}

// Convert a ratio into mL using the provided calibration.
float SyringeCalibration::mlFromRatio_(const App::PotCalibration& cal, float ratio) {
  return cal.ratioToMl(ratio);
}

// Print toolhead calibration info to the output stream.
void SyringeCalibration::printToolheadInfo(Stream& out) {
  out.println(F("[SFC] Toolhead calibration:"));

  if (m_toolhead.rfid == 0) {
    out.println(F("  RFID: (none)  tip: scan the toolhead first"));
    return;
  }

  out.print(F("  RFID: 0x"));
  out.println(m_toolhead.rfid, HEX);

  App::PotCalibration calTmp;
  bool ok = Util::loadCalibration(m_toolhead.rfid, calTmp);
  m_toolCalValid = ok;
  if (ok) m_toolCal = calTmp;

  out.print(F("  cal loaded from memory: "));
  out.println(ok ? F("yes") : F("no"));

  if (ok) {
    out.print(F("  cal.points(")); out.print(m_toolCal.pointCount); out.println(F("):"));
    for (uint8_t i = 0; i < m_toolCal.pointCount; ++i) {
      out.print(F("    ["));
      out.print(i);
      out.print(F("] "));
      out.print(m_toolCal.points[i].volume_ml, 3);
      out.print(F(" ml @ "));
      out.print(m_toolCal.points[i].ratio, 3);
      out.println(F(" ratio"));
    }
    out.print(F("  cal.steps_mL     : ")); out.println(m_toolCal.steps_mL, 3);
  }

  float ratio = Pots::percent(TOOLHEAD_POT_IDX);
  uint16_t scaled = Pots::readScaled(TOOLHEAD_POT_IDX);

  out.print(F("  pot.ratio: ")); out.println(ratio, 3);
  out.print(F("  pot.scaled: ")); out.println(scaled);

  if (ok) {
    float ml = mlFromRatio_(m_toolCal, ratio);
    out.print(F("  computed mL: "));
    if (isnan(ml)) out.println(F("(n/a)"));
    else out.println(ml, 3);
  }
}

// Compute the current toolhead volume in mL.
float SyringeCalibration::readToolheadVolumeMl() {
  float ratio = readToolheadRatio();
  float ml = m_toolhead.cal.ratioToMl(ratio);
  if (CAL_DBG) {
    Serial.print("[SFC] readToolheadVolumeMl(): ");
    Serial.println(ml, 3);
  }
  return ml;
}

// Compute the current base volume in mL for a slot.
float SyringeCalibration::readBaseVolumeMl(uint8_t slot) {
  if (slot >= m_baseCount) {
    if (CAL_DBG) {
      Serial.print("[SFC] readBaseVolumeMl(): slot OOR ");
      Serial.println(slot);
    }
    return 0.0f;
  }

  float ratio = 0.0f;
  String ratioMessage;
  bool ratioOk = readBasePotRatio(slot, ratio, ratioMessage);
  if (ratioOk && m_bases[slot].calPoints.count >= 2) {
    bool ok = false;
    float ml = interpolateVolumeFromPoints(m_bases[slot].calPoints, ratio, ok);
    if (ok) {
      if (CAL_DBG) {
        Serial.print("[SFC] readBaseVolumeMl(slot=");
        Serial.print(slot);
        Serial.print("): ratio=");
        Serial.print(ratio, 4);
        Serial.print(" -> ");
        Serial.println(ml, 3);
      }
      return ml;
    }
  } else if (CAL_DBG && ratioOk) {
    Serial.print("[SFC] readBaseVolumeMl(slot=");
    Serial.print(slot);
    Serial.println("): insufficient calibration points for interpolation");
    return 0.0f;
  }

  return 0.0f;
}

// Build a JSON volume report for toolhead and bases.
bool SyringeCalibration::buildVolumesReport(String& data, String& message) {
  String out = "{";
  bool any = false;

  if (m_toolhead.rfid != 0) {
    out += "\"toolhead\":{";
    out += "\"rfid\":\"0x" + String(m_toolhead.rfid, HEX) + "\"";
    out += ",\"calPoints\":" + String(m_toolhead.cal.pointCount);
    if (m_toolhead.cal.pointCount >= 2) {
      float percent = 0.0f;
      String potMessage;
      bool potOk = readToolheadPotPercent(percent, potMessage);
      if (potOk) {
        float ml = readToolheadVolumeMl();
        m_toolhead.currentMl = ml;
        out += ",\"percent\":" + String(percent, 3);
        out += ",\"volumeMl\":" + String(ml, 3);
        out += ",\"ok\":true";
      } else {
        out += ",\"ok\":false";
        out += ",\"message\":\"" + potMessage + "\"";
      }
    } else {
      out += ",\"ok\":false";
      out += ",\"message\":\"insufficient calibration points\"";
    }
    out += "}";
    any = true;
  }

  bool anyBases = false;
  String bases = "\"bases\":[";
  for (uint8_t i = 0; i < m_baseCount; ++i) {
    Syringe& sy = m_bases[i];
    if (sy.rfid == 0) {
      continue;
    }
    if (anyBases) {
      bases += ",";
    }
    bases += "{";
    bases += "\"slot\":" + String(i);
    bases += ",\"rfid\":\"0x" + String(sy.rfid, HEX) + "\"";
    bases += ",\"calPoints\":" + String(sy.calPoints.count);
    if (sy.calPoints.count >= 2) {
      float percent = 0.0f;
      String potMessage;
      bool potOk = readBasePotPercent(i, percent, potMessage);
      if (potOk) {
        float ml = readBaseVolumeMl(i);
        sy.currentMl = ml;
        bases += ",\"percent\":" + String(percent, 3);
        bases += ",\"volumeMl\":" + String(ml, 3);
        bases += ",\"ok\":true";
      } else {
        bases += ",\"ok\":false";
        bases += ",\"message\":\"" + potMessage + "\"";
      }
    } else {
      bases += ",\"ok\":false";
      bases += ",\"message\":\"insufficient calibration points\"";
    }
    bases += "}";
    anyBases = true;
  }
  bases += "]";

  if (anyBases) {
    if (any) {
      out += ",";
    }
    out += bases;
    any = true;
  }

  out += "}";

  if (!any) {
    message = "no scanned syringes";
    data = "";
    return false;
  }

  data = out;
  message = "volumes reported";
  return true;
}

// Read toolhead pot percent (0..100).
bool SyringeCalibration::readToolheadPotPercent(float& percent, String& message) {
  percent = readToolheadRatio();
  message = "";
  return true;
}

// Read base pot percent (0..100) for a slot.
bool SyringeCalibration::readBasePotPercent(uint8_t slot, float& percent, String& message) {
  float ratio = 0.0f;
  if (!readBasePotRatio(slot, ratio, message)) {
    return false;
  }
  percent = ratio * 100.0f;
  return true;
}

} // namespace App
