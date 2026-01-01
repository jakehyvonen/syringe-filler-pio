#include "app/SyringeCalibration.hpp"

#include "hw/Bases.hpp"
#include "hw/Pots.hpp"
#include "util/Storage.hpp"
#include <Arduino.h>
#include <math.h>

namespace App {

namespace {
  constexpr bool CAL_DBG = true;
  constexpr uint8_t TOOLHEAD_POT_IDX = 0;  // TODO: set your toolhead pot index

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

int8_t SyringeCalibration::getBasePotIndex(uint8_t baseSlot) const {
  if (baseSlot >= m_baseCount) return -1;
  uint8_t p = m_baseToPot[baseSlot];
  return (p == 0xFF) ? -1 : (int8_t)p;
}

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

bool SyringeCalibration::saveToolheadCalibration() {
  if (m_toolhead.rfid == 0) {
    if (CAL_DBG) Serial.println("[SFC] saveToolheadCalibration(): no toolhead RFID");
    return false;
  }
  syncLegacyFields(m_toolhead.cal);
  bool ok = Util::saveCalibration(m_toolhead.rfid, m_toolhead.cal);
  if (CAL_DBG) {
    Serial.print("[SFC] saveToolheadCalibration(): ");
    Serial.println(ok ? "OK" : "FAIL");
  }
  return ok;
}

bool SyringeCalibration::captureToolheadCalibrationPoint(float ml, String& message) {
  if (ml < 0.0f) {
    message = "volume must be >= 0";
    return false;
  }
  if (m_toolhead.rfid == 0) {
    message = "no toolhead RFID; run sfc.scanTool first";
    return false;
  }

  float ratio = readToolheadRatio();
  bool ok = m_toolhead.cal.addPoint(ml, ratio);
  if (!ok) {
    message = (m_toolhead.cal.pointCount >= PotCalibration::kMaxPoints)
                ? "toolhead calibration point list full"
                : "failed to add toolhead calibration point";
    return false;
  }

  syncLegacyFields(m_toolhead.cal);
  if (m_toolhead.cal.pointCount < 2) {
    message = "point saved; add at least 2 points to enable interpolation";
  } else {
    message = "point saved";
  }
  return true;
}

bool SyringeCalibration::clearCurrentBaseCalibrationPoints(String& message) {
  if (m_currentSlot < 0 || m_currentSlot >= (int)m_baseCount) {
    message = "no current base";
    return false;
  }

  Syringe& sy = m_bases[m_currentSlot];
  if (sy.rfid == 0) {
    message = "base has no RFID; run sfc.scanbase first";
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

bool SyringeCalibration::clearToolheadCalibrationPoints(String& message) {
  if (m_toolhead.rfid == 0) {
    message = "no toolhead RFID; run sfc.scanTool first";
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

bool SyringeCalibration::forceCurrentBaseCalibrationZero(String& message) {
  if (m_currentSlot < 0 || m_currentSlot >= (int)m_baseCount) {
    message = "no current base";
    return false;
  }

  Syringe& sy = m_bases[m_currentSlot];
  if (sy.rfid == 0) {
    message = "base has no RFID; run sfc.scanbase first";
    return false;
  }
  if (sy.calPoints.count == 0) {
    message = "no base calibration points to update";
    return false;
  }

  sy.calPoints.points[0].volumeMl = 0.0f;
  Util::BaseMeta meta;
  App::PotCalibration cal;
  App::CalibrationPoints savedPoints;
  if (!Util::loadBase(sy.rfid, meta, cal, savedPoints)) {
    memset(&meta, 0, sizeof(meta));
  }
  (void)cal;
  (void)savedPoints;
  bool ok = Util::saveBase(sy.rfid, meta, sy.cal, sy.calPoints);
  message = ok ? "base calibration forced through 0 mL" : "failed to update base calibration";
  return ok;
}

bool SyringeCalibration::forceToolheadCalibrationZero(String& message) {
  if (m_toolhead.rfid == 0) {
    message = "no toolhead RFID; run sfc.scanTool first";
    return false;
  }
  if (m_toolhead.cal.pointCount == 0) {
    message = "no toolhead calibration points to update";
    return false;
  }

  m_toolhead.cal.points[0].volume_ml = 0.0f;
  syncLegacyFields(m_toolhead.cal);
  bool ok = Util::saveCalibration(m_toolhead.rfid, m_toolhead.cal);
  message = ok ? "toolhead calibration forced through 0 mL" : "failed to update toolhead calibration";
  return ok;
}

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
    message = "base has no RFID; run sfc.scanbase first";
    return false;
  }

  float ratio = 0.0f;
  String ratioMessage;
  if (!readBasePotRatio(slot, ratio, ratioMessage)) {
    message = ratioMessage;
    return false;
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

bool SyringeCalibration::saveCurrentBaseToNVS() {
  if (m_currentSlot < 0 || m_currentSlot >= (int8_t)m_baseCount) {
    if (CAL_DBG) Serial.println("[SFC] saveCurrentBaseToNVS(): no current slot");
    return false;
  }

  Syringe& sy = m_bases[m_currentSlot];
  if (sy.rfid == 0) {
    if (CAL_DBG) Serial.println("[SFC] saveCurrentBaseToNVS(): no RFID cached for this slot");
    return false;
  }

  Util::BaseMeta meta;
  PotCalibration cal;
  App::CalibrationPoints points;
  if (Util::loadBase(sy.rfid, meta, cal, points)) {
    cal = sy.cal;
    if (CAL_DBG) {
      Serial.print("[SFC] saveCurrentBaseToNVS(): updating existing base 0x");
      Serial.println(sy.rfid, HEX);
    }
    (void)points;
    return Util::saveBase(sy.rfid, meta, cal, sy.calPoints);
  }

  if (CAL_DBG) {
    Serial.print("[SFC] saveCurrentBaseToNVS(): creating new base 0x");
    Serial.println(sy.rfid, HEX);
  }
  memset(&meta, 0, sizeof(meta));
  return Util::saveBase(sy.rfid, meta, sy.cal, sy.calPoints);
}

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

float SyringeCalibration::mlFromRatio_(const App::PotCalibration& cal, float ratio) {
  return cal.ratioToMl(ratio);
}

void SyringeCalibration::printToolheadInfo(Stream& out) {
  out.println(F("[SFC] Toolhead calibration:"));

  if (m_toolhead.rfid == 0) {
    out.println(F("  RFID: (none)  tip: run 'sfc.scanTool'"));
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

float SyringeCalibration::readToolheadVolumeMl() {
  float ratio = readToolheadRatio();
  float ml = m_toolhead.cal.ratioToMl(ratio);
  if (CAL_DBG) {
    Serial.print("[SFC] readToolheadVolumeMl(): stub -> ");
    Serial.println(ml, 3);
  }
  return ml;
}

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

bool SyringeCalibration::readToolheadPotPercent(float& percent, String& message) {
  percent = readToolheadRatio();
  message = "";
  return true;
}

bool SyringeCalibration::readBasePotPercent(uint8_t slot, float& percent, String& message) {
  float ratio = 0.0f;
  if (!readBasePotRatio(slot, ratio, message)) {
    return false;
  }
  percent = ratio * 100.0f;
  return true;
}

} // namespace App
