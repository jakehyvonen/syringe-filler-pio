#include "app/SyringeFillController.hpp"

#include "hw/Bases.hpp"
#include "hw/Pots.hpp"
#include "motion/Axis.hpp"
#include "hw/RFID.hpp"
#include "hw/BaseRFID.hpp"
#include "util/Storage.hpp"   // Util::loadBase, Util::saveBase, Util::loadRecipe, ...
#include <Arduino.h>

namespace App {

namespace {
  constexpr bool SFC_DBG = true;
  constexpr uint8_t TOOLHEAD_POT_IDX = 0;  // TODO: set your toolhead pot index

  // used by BaseRFID listener
  struct BaseTagCapture {
    bool     got    = false;
    uint32_t packed = 0;
  };

  // listener that BaseRFID will call
  void baseTagHandler(const uint8_t* uid, uint8_t len, void* user) {
    auto* cap = static_cast<BaseTagCapture*>(user);
    if (!cap) return;

    uint32_t v = 0;
    for (uint8_t i = 0; i < len && i < 4; ++i) {
      v = (v << 8) | uid[i];
    }
    cap->packed = v;
    cap->got    = true;
  }

  void dbg(const char* msg) {
    if (SFC_DBG) {
      Serial.print("[SFC] ");
      Serial.println(msg);
    }
  }
}

// ------------------------------------------------------------
// ctor
// ------------------------------------------------------------
SyringeFillController::SyringeFillController() {
  if (SFC_DBG) {
    Serial.println("[SFC] ctor: init base slots");
  }
  for (uint8_t i = 0; i < Bases::kCount; ++i) {
    m_bases[i] = Syringe{};
    m_bases[i].slot = i;
  }
  m_currentSlot = -1;
//physical mapping of pots
  m_baseToPot[0] = 3;
  m_baseToPot[1] = 4;
  m_baseToPot[2] = 5;
  m_baseToPot[3] = 0;
  m_baseToPot[4] = 1;
}


int8_t SyringeFillController::getBasePotIndex(uint8_t baseSlot) const {
  if (baseSlot >= Bases::kCount) return -1;
  uint8_t p = m_baseToPot[baseSlot];
  return (p == 0xFF) ? -1 : (int8_t)p;
}

// ------------------------------------------------------------
// toolhead calibration helpers
// ------------------------------------------------------------
float SyringeFillController::readToolheadRatio() {
  float ratio = Pots::percent(TOOLHEAD_POT_IDX);
  if (SFC_DBG) {
    Serial.print("[SFC] readToolheadRatio(): pot=");
    Serial.print(TOOLHEAD_POT_IDX);
    Serial.print(" ratio=");
    Serial.println(ratio, 3);
  }
  return ratio;
}

bool SyringeFillController::captureToolheadEmpty() {
  if (m_toolhead.rfid == 0) {
    if (SFC_DBG) Serial.println("[SFC] captureToolheadEmpty(): no toolhead RFID yet");
    return false;
  }
  float ratio = readToolheadRatio();
  bool ok = m_toolhead.cal.addPoint(0.0f, ratio);
  if (SFC_DBG) {
    Serial.print("[SFC] toolhead empty ratio = ");
    Serial.print(ratio, 3);
    Serial.print(" -> ");
    Serial.println(ok ? "saved" : "FAILED");
  }
  return ok;
}

bool SyringeFillController::captureToolheadFull(float mlFull) {
  if (m_toolhead.rfid == 0) {
    if (SFC_DBG) Serial.println("[SFC] captureToolheadFull(): no toolhead RFID yet");
    return false;
  }
  float ratio = readToolheadRatio();
  bool ok = m_toolhead.cal.addPoint(mlFull, ratio);
  if (SFC_DBG) {
    Serial.print("[SFC] toolhead full ratio = ");
    Serial.print(ratio, 3);
    Serial.print("  mlFull = ");
    Serial.print(mlFull, 3);
    Serial.print(" -> ");
    Serial.println(ok ? "saved" : "FAILED");
  }
  return ok;
}

bool SyringeFillController::saveToolheadCalibration() {
  if (m_toolhead.rfid == 0) {
    if (SFC_DBG) Serial.println("[SFC] saveToolheadCalibration(): no toolhead RFID");
    return false;
  }
  bool ok = Util::saveCalibration(m_toolhead.rfid, m_toolhead.cal);
  if (SFC_DBG) {
    Serial.print("[SFC] saveToolheadCalibration(): ");
    Serial.println(ok ? "OK" : "FAIL");
  }
  return ok;
}

// ------------------------------------------------------------
// scan all / scan one base
// ------------------------------------------------------------
void SyringeFillController::scanAllBaseSyringes() {
  dbg("scanAllBaseSyringes() start");
  for (uint8_t i = 0; i < Bases::kCount; ++i) {
    if (!scanBaseSyringe(i)) {
      if (SFC_DBG) {
        Serial.print("[SFC] WARN: scanBaseSyringe(");
        Serial.print(i);
        Serial.println(") failed");
      }
    }
  }
  dbg("scanAllBaseSyringes() done");
}

// --------------------------------------------------
// scan *one* base – now auto-creates in NVS
// --------------------------------------------------
bool SyringeFillController::scanBaseSyringe(uint8_t slot) {
  if (slot >= Bases::kCount) {
    if (SFC_DBG) {
      Serial.print("[SFC] scanBaseSyringe(): slot out of range ");
      Serial.println(slot);
    }
    return false;
  }

  if (!goToBase(slot)) {
    if (SFC_DBG) {
      Serial.print("[SFC] scanBaseSyringe(): goToBase(");
      Serial.print(slot);
      Serial.println(") failed");
    }
    return false;
  }

  if (SFC_DBG) Serial.println("[SFC] scanBaseSyringe(): calling readBaseRFIDBlocking()");
  uint32_t tag = readBaseRFIDBlocking(2000); // 2s timeout
  if (tag == 0) {
    if (SFC_DBG) {
      Serial.print("[SFC] scanBaseSyringe(): no tag at slot ");
      Serial.println(slot);
    }
    return false;
  }

  if (SFC_DBG) {
    Serial.print("[SFC] scanBaseSyringe(): slot ");
    Serial.print(slot);
    Serial.print(" -> tag 0x");
    Serial.println(tag, HEX);
  }

  // update runtime object
  Syringe& s = m_bases[slot];
  s.rfid = tag;
  s.role = SyringeRole::Base;
  s.slot = slot;
// try to load from NVS
Util::BaseMeta meta;
PotCalibration cal;
if (Util::loadBase(tag, meta, cal)) {
  s.cal = cal;
  if (SFC_DBG) {
    Serial.print("[SFC] scanBaseSyringe(): existing base loaded from NVS for tag 0x");
    Serial.println(tag, HEX);
  }
} else {
  // NEW BASE: create minimal meta and save immediately (no slot stored)
  if (SFC_DBG) {
    Serial.print("[SFC] scanBaseSyringe(): NEW base, creating NVS entry for tag 0x");
    Serial.println(tag, HEX);
  }
  memset(&meta, 0, sizeof(meta));
  Util::saveBase(tag, meta, s.cal);
}


  // also update current volume (stub for now)
  s.currentMl = readBaseVolumeMl(slot);
  if (SFC_DBG) {
    Serial.print("[SFC] scanBaseSyringe(): measured ~");
    Serial.print(s.currentMl, 3);
    Serial.println(" mL");
  }

  return true;
}
bool SyringeFillController::setCurrentBaseMlFull(float ml) {
  if (m_currentSlot < 0 || m_currentSlot >= (int)Bases::kCount) {
    if (SFC_DBG) Serial.println("[SFC] setCurrentBaseMlFull(): no current base slot");
    return false;
  }
  if (ml <= 0.0f) {
    if (SFC_DBG) Serial.println("[SFC] setCurrentBaseMlFull(): ml must be > 0");
    return false;
  }

  Syringe &sy = m_bases[m_currentSlot];
  if (sy.rfid == 0) {
    if (SFC_DBG) Serial.println("[SFC] setCurrentBaseMlFull(): base has no RFID, run sfc.scanbase N first");
    return false;
  }

  // update RAM
  float ratio = readBaseRatio((uint8_t)m_currentSlot);
  bool ok = sy.cal.addPoint(ml, ratio);

  if (SFC_DBG) {
    Serial.print("[SFC] setCurrentBaseMlFull(): slot=");
    Serial.print(m_currentSlot);
    Serial.print(" mlFull=");
    Serial.print(ml, 3);
    Serial.print(" ratio=");
    Serial.print(ratio, 3);
    Serial.print(" -> ");
    Serial.println(ok ? "saved" : "FAILED");
  }

  // keep meta, overwrite cal in NVS
  Util::BaseMeta meta;
  App::PotCalibration dummy;
  Util::loadBase(sy.rfid, meta, dummy);
  bool saved = ok && Util::saveBase(sy.rfid, meta, sy.cal);
  if (SFC_DBG) Serial.println(saved ? "[SFC] setCurrentBaseMlFull(): saved to NVS"
                                    : "[SFC] setCurrentBaseMlFull(): FAILED to save to NVS");
  return saved;
}
bool SyringeFillController::setToolheadMlFull(float ml) {
  if (ml <= 0.0f) {
    if (SFC_DBG) Serial.println("[SFC] setToolheadMlFull(): ml must be > 0");
    return false;
  }
  if (m_toolhead.rfid == 0) {
    if (SFC_DBG) Serial.println("[SFC] setToolheadMlFull(): no toolhead RFID, scan toolhead first");
    return false;
  }

  float ratio = readToolheadRatio();
  bool ok = m_toolhead.cal.addPoint(ml, ratio);

  if (SFC_DBG) {
    Serial.print("[SFC] setToolheadMlFull(): mlFull=");
    Serial.print(ml, 3);
    Serial.print(" ratio=");
    Serial.print(ratio, 3);
    Serial.print(" -> ");
    Serial.println(ok ? "saved" : "FAILED");
  }

  bool saved = ok && Util::saveCalibration(m_toolhead.rfid, m_toolhead.cal);
  if (SFC_DBG) Serial.println(saved ? "[SFC] ...toolhead cal saved to NVS"
                                    : "[SFC] ...FAILED to save toolhead cal");
  return saved;
}


float SyringeFillController::readBaseRatio(uint8_t slot) {
  if (slot >= Bases::kCount) {
    if (SFC_DBG) {
      Serial.print("[SFC] readBaseRatio(): slot OOR ");
      Serial.println(slot);
    }
    return 0.0f;
  }

  int8_t potIdx = getBasePotIndex(slot);
  if (potIdx < 0) {
    if (SFC_DBG) {
      Serial.print("[SFC] readBaseRatio(): no pot mapped for base ");
      Serial.println(slot);
    }
    return 0.0f;
  }

  float ratio = Pots::percent((uint8_t)potIdx);

  if (SFC_DBG) {
    Serial.print("[SFC] readBaseRatio(base=");
    Serial.print(slot);
    Serial.print(" pot=");
    Serial.print(potIdx);
    Serial.print("): ratio=");
    Serial.print(ratio, 3);
  }

  return ratio;
}

// --------------------------------------------------
// capture base EMPTY (pot @ fully empty)
// --------------------------------------------------
bool SyringeFillController::captureBaseEmpty(uint8_t slot) {
  if (slot >= Bases::kCount) {
    if (SFC_DBG) {
      Serial.print("[SFC] captureBaseEmpty(): slot OOR ");
      Serial.println(slot);
    }
    return false;
  }

  // read the actual pot channel for this base
  float ratio = readBaseRatio(slot);
  bool ok = m_bases[slot].cal.addPoint(0.0f, ratio);

  if (SFC_DBG) {
    Serial.print("[SFC] captureBaseEmpty(): slot=");
    Serial.print(slot);
    Serial.print(" ratio=");
    Serial.print(ratio, 3);
    Serial.print(" -> ");
    Serial.println(ok ? "saved" : "FAILED");
  }

  // if this base has an RFID, persist immediately
  uint32_t tag = m_bases[slot].rfid;
  if (tag != 0) {
    Util::BaseMeta meta;
    App::PotCalibration dummy;
    // load ONLY meta, ignore old cal
    Util::loadBase(tag, meta, dummy);
    bool saved = ok && Util::saveBase(tag, meta, m_bases[slot].cal);
    if (SFC_DBG) Serial.println(saved ? "[SFC] captureBaseEmpty(): saved to NVS" :
                                        "[SFC] captureBaseEmpty(): FAILED to save to NVS");
  }

  return ok;
}
bool SyringeFillController::captureBaseFull(uint8_t slot) {
  if (slot >= Bases::kCount) {
    if (SFC_DBG) {
      Serial.print("[SFC] captureBaseFull(): slot OOR ");
      Serial.println(slot);
    }
    return false;
  }

  float ratio = readBaseRatio(slot);
  float currentMaxMl = (m_bases[slot].cal.pointCount > 0)
                         ? m_bases[slot].cal.points[m_bases[slot].cal.pointCount - 1].volume_ml
                         : 0.0f;
  bool ok = m_bases[slot].cal.addPoint(currentMaxMl, ratio);

  if (SFC_DBG) {
    Serial.print("[SFC] captureBaseFull(): slot=");
    Serial.print(slot);
    Serial.print(" ratio=");
    Serial.print(ratio, 3);
    Serial.print(" -> ");
    Serial.println(ok ? "saved" : "FAILED");
  }

  uint32_t tag = m_bases[slot].rfid;
  if (tag != 0) {
    Util::BaseMeta meta;
    App::PotCalibration dummy;
    Util::loadBase(tag, meta, dummy);               // keep meta
    bool saved = ok && Util::saveBase(tag, meta, m_bases[slot].cal);
    if (SFC_DBG) Serial.println(saved ? "[SFC] captureBaseFull(): saved to NVS" :
                                        "[SFC] captureBaseFull(): FAILED to save to NVS");
  }

  return ok;
}

// --------------------------------------------------
// save current base (whatever slot we are at) to NVS
// --------------------------------------------------
bool SyringeFillController::saveCurrentBaseToNVS() {
  if (m_currentSlot < 0 || m_currentSlot >= (int8_t)Bases::kCount) {
    if (SFC_DBG) Serial.println("[SFC] saveCurrentBaseToNVS(): no current slot");
    return false;
  }

  Syringe& sy = m_bases[m_currentSlot];
  if (sy.rfid == 0) {
    if (SFC_DBG) Serial.println("[SFC] saveCurrentBaseToNVS(): no RFID cached for this slot");
    return false;
  }

  Util::BaseMeta meta;
  PotCalibration cal;
  if (Util::loadBase(sy.rfid, meta, cal)) {
    // update cal
    cal = sy.cal;
    if (SFC_DBG) {
      Serial.print("[SFC] saveCurrentBaseToNVS(): updating existing base 0x");
      Serial.println(sy.rfid, HEX);
    }
    return Util::saveBase(sy.rfid, meta, cal);
  } else {
    // create new
    if (SFC_DBG) {
      Serial.print("[SFC] saveCurrentBaseToNVS(): creating new base 0x");
      Serial.println(sy.rfid, HEX);
    }
    memset(&meta, 0, sizeof(meta));
    return Util::saveBase(sy.rfid, meta, sy.cal);
  }
}

// --------------------------------------------------
// print helper
// --------------------------------------------------
void SyringeFillController::printBaseInfo(uint8_t slot, Stream& s) {
  if (slot >= Bases::kCount) {
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
  s.println();
}
// ------------------------------------------------------------
// blocking base-RFID read (with tick)
// ------------------------------------------------------------
uint32_t SyringeFillController::readBaseRFIDBlocking(uint32_t timeoutMs) {
  Serial.print("[SFC] readBaseRFIDBlocking(): start, timeout=");
  Serial.print(timeoutMs);
  Serial.println(" ms");

  BaseTagCapture cap;
  cap.got    = false;
  cap.packed = 0;

  const bool wasEnabled = BaseRFID::enabled();
  Serial.print("[SFC] readBaseRFIDBlocking(): BaseRFID was ");
  Serial.println(wasEnabled ? "ENABLED" : "DISABLED");

  BaseRFID::setListener(baseTagHandler, &cap);
  Serial.println("[SFC] readBaseRFIDBlocking(): listener registered");

  if (!wasEnabled) {
    BaseRFID::enable(true);
    Serial.println("[SFC] readBaseRFIDBlocking(): BaseRFID enabled temporarily");
  }

  const uint32_t start   = millis();
  uint32_t       lastLog = start;
  uint32_t       iter    = 0;

  while (millis() - start < timeoutMs) {
    Serial.print("[SFC] RBRFID iter=");
    Serial.print(iter++);
    Serial.println(" -> calling BaseRFID::tick()");
    BaseRFID::tick();
    Serial.println("[SFC] ...returned from BaseRFID::tick()");

    if (cap.got) {
      Serial.print("[SFC] readBaseRFIDBlocking(): tag captured, packed=0x");
      Serial.println(cap.packed, HEX);
      break;
    }

    uint32_t now = millis();
    if (now - lastLog > 200) {
      Serial.print("[SFC] readBaseRFIDBlocking(): waiting... ");
      Serial.print(now - start);
      Serial.println(" ms");
      lastLog = now;
    }

    delay(5);
  }

  BaseRFID::setListener(nullptr, nullptr);
  Serial.println("[SFC] readBaseRFIDBlocking(): listener cleared");

  if (!wasEnabled) {
    BaseRFID::enable(false);
    Serial.println("[SFC] readBaseRFIDBlocking(): BaseRFID returned to DISABLED");
  }

  if (!cap.got) {
    Serial.println("[SFC] readBaseRFIDBlocking(): TIMEOUT, no tag");
    return 0;
  }

  Serial.print("[SFC] readBaseRFIDBlocking(): success, returning 0x");
  Serial.println(cap.packed, HEX);
  return cap.packed;
}

// ------------------------------------------------------------
// toolhead side
// ------------------------------------------------------------

bool SyringeFillController::scanToolheadBlocking() {
  dbg("scanToolheadBlocking() start");

  uint32_t tag = readToolheadRFIDBlocking(2000);
  if (tag == 0) {
    dbg("no toolhead tag detected");
    return false;
  }

  m_toolhead.rfid = tag;
  m_toolhead.role = SyringeRole::Toolhead;

  PotCalibration cal;
  if (Util::loadCalibration(tag, cal)) {
    m_toolhead.cal = cal;
    Serial.print("[SFC] loaded toolhead cal for 0x");
    Serial.println(tag, HEX);
  } else {
    Serial.print("[SFC] no toolhead cal for 0x");
    Serial.println(tag, HEX);
  }

  m_toolhead.currentMl = readToolheadVolumeMl();
  Serial.print("[SFC] toolhead volume ~");
  Serial.println(m_toolhead.currentMl, 3);

  dbg("scanToolheadBlocking() done");
  return true;
}


// ------------------------------------------------------------
// blocking TOOLHEAD RFID read (I2C PN532 #1)
// ------------------------------------------------------------
uint32_t SyringeFillController::readToolheadRFIDBlocking(uint32_t timeoutMs) {
  Serial.print("[SFC] readToolheadRFIDBlocking(): timeout=");
  Serial.print(timeoutMs);
  Serial.println(" ms");

  struct Cap {
    bool got = false;
    uint32_t packed = 0;
  } cap;

  // listener that packs first up-to-4 bytes
  auto handler = [](const uint8_t* uid, uint8_t len, void* user) {
    Cap* c = static_cast<Cap*>(user);
    uint32_t v = 0;
    for (uint8_t i = 0; i < len && i < 4; ++i) {
      v = (v << 8) | uid[i];
    }
    c->packed = v;
    c->got = true;
  };

  const bool wasEnabled = RFID::enabled();
  if (!wasEnabled) {
    RFID::enable(true);
    Serial.println("[SFC] TH:block: enabled RFID temporarily");
  }

  RFID::setListener(handler, &cap);

  const uint32_t start = millis();
  uint32_t iter = 0;

  while (millis() - start < timeoutMs) {
    Serial.print("[SFC] TH:block iter=");
    Serial.println(iter++);
    RFID::tick();   // force tick for toolhead PN532

    if (cap.got) {
      Serial.print("[SFC] TH:block got tag 0x");
      Serial.println(cap.packed, HEX);
      break;
    }
    delay(5);
  }

  RFID::setListener(nullptr, nullptr);

  if (!wasEnabled) {
    RFID::enable(false);
    Serial.println("[SFC] TH:block: restored to DISABLED");
  }

  if (!cap.got) {
    Serial.println("[SFC] TH:block TIMEOUT");
    return 0;
  }

  Serial.print("[SFC] TH:block success: 0x");
  Serial.println(cap.packed, HEX);
  return cap.packed;
}

float App::SyringeFillController::mlFromRatio_(const App::PotCalibration& cal, float ratio) {
  return cal.ratioToMl(ratio);
}

void App::SyringeFillController::printToolheadInfo(Stream& out) {
  out.println(F("[SFC] Toolhead calibration:"));

  if (m_toolhead.rfid == 0) {
    out.println(F("  RFID: (none)  tip: run 'sfc.scanTool'"));
    return;
  }

  out.print(F("  RFID: 0x"));
  out.println(m_toolhead.rfid, HEX);

  // Refresh calibration from NVS each time (so it always matches Storage.cpp)
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

  // Live pot read (set this index correctly for the toolhead pot)
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


bool SyringeFillController::loadToolheadRecipeFromFS() {
  if (m_toolhead.rfid == 0) {
    dbg("loadToolheadRecipeFromFS(): no toolhead RFID");
    return false;
  }
  bool ok = Util::loadRecipe(m_toolhead.rfid, m_recipe);
  if (!ok && SFC_DBG) {
    Serial.print("[SFC] loadToolheadRecipeFromFS(): failed for tag 0x");
    Serial.println(m_toolhead.rfid, HEX);
  }
  return ok;
}

bool SyringeFillController::saveToolheadRecipeToFS() {
  if (m_toolhead.rfid == 0) {
    dbg("saveToolheadRecipeToFS(): no toolhead RFID");
    return false;
  }
  bool ok = Util::saveRecipe(m_toolhead.rfid, m_recipe);
  if (!ok && SFC_DBG) {
    Serial.print("[SFC] saveToolheadRecipeToFS(): failed for tag 0x");
    Serial.println(m_toolhead.rfid, HEX);
  }
  return ok;
}

// ------------------------------------------------------------
// run recipe
// ------------------------------------------------------------
void SyringeFillController::runRecipe() {
  dbg("runRecipe() start");
  for (uint8_t i = 0; i < m_recipe.count; ++i) {
    auto& step = m_recipe.steps[i];
    if (SFC_DBG) {
      Serial.print("[SFC] step ");
      Serial.print(i);
      Serial.print(": base=");
      Serial.print(step.baseSlot);
      Serial.print(" ml=");
      Serial.println(step.ml, 3);
    }
    if (!transferFromBase(step.baseSlot, step.ml)) {
      if (SFC_DBG) {
        Serial.print("[SFC] ERROR: transferFromBase failed at step ");
        Serial.println(i);
      }
    }
  }
  m_toolhead.currentMl = readToolheadVolumeMl();
  dbg("runRecipe() done");
}

// ------------------------------------------------------------
// positioning
// ------------------------------------------------------------
bool SyringeFillController::goToBase(uint8_t slot) {
  if (slot >= Bases::kCount) {
    if (SFC_DBG) {
      Serial.print("[SFC] goToBase(): slot out of range: ");
      Serial.println(slot);
    }
    return false;
  }
  long target = Bases::positionSteps(slot);
  if (target < 0) {
    if (SFC_DBG) {
      Serial.print("[SFC] goToBase(): no position for slot ");
      Serial.println(slot);
    }
    return false;
  }

  if (SFC_DBG) {
    Serial.print("[SFC] goToBase(");
    Serial.print(slot);
    Serial.print(") -> steps=");
    Serial.println(target);
  }

  Axis::moveTo(target);
  m_currentSlot = slot;
  if (SFC_DBG) {
    Serial.println("[SFC] goToBase: finished successfully");
  }
  return true;
}


// ------------------------------------------------------------
// toolhead reader (I2C PN532 #1)
// ------------------------------------------------------------
uint32_t SyringeFillController::readRFIDNow() {
  const uint32_t timeoutMs = 2000;
  const uint32_t startMs   = millis();

  const bool wasEnabled = RFID::enabled();
  if (!wasEnabled) {
    RFID::enable(true);
    if (SFC_DBG) Serial.println("[SFC] readRFIDNow(): RFID was disabled, enabling temporarily");
  }

  if (SFC_DBG) Serial.println("[SFC] readRFIDNow(): waiting for tag...");

  while (millis() - startMs < timeoutMs) {
    // note: App::loop() normally calls RFID::tick(), but we’re blocking here
    // if your other reader needs tick here too, you can call it
    if (RFID::available()) {
      const uint8_t* uid = RFID::uidBytes();
      uint8_t len        = RFID::uidLen();

      if (len == 0 || uid == nullptr) {
        if (SFC_DBG) Serial.println("[SFC] readRFIDNow(): available() but empty UID");
        if (!wasEnabled) RFID::enable(false);
        return 0;
      }

      uint32_t packed = 0;
      for (uint8_t i = 0; i < len && i < 4; ++i) {
        packed = (packed << 8) | uid[i];
      }

      if (SFC_DBG) {
        Serial.print("[SFC] readRFIDNow(): got UID len=");
        Serial.print(len);
        Serial.print(" packed=0x");
        Serial.println(packed, HEX);
      }

      if (!wasEnabled) RFID::enable(false);
      return packed;
    }
    delay(10);
  }

  if (SFC_DBG) Serial.println("[SFC] readRFIDNow(): timeout, no tag");
  if (!wasEnabled) RFID::enable(false);
  return 0;
}

// ------------------------------------------------------------
// volume helpers (still stubs)
// ------------------------------------------------------------
float SyringeFillController::readToolheadVolumeMl() {
  float ratio = readToolheadRatio();
  float ml = m_toolhead.cal.ratioToMl(ratio);
  if (SFC_DBG) {
    Serial.print("[SFC] readToolheadVolumeMl(): stub -> ");
    Serial.println(ml, 3);
  }
  return ml;
}

float SyringeFillController::readBaseVolumeMl(uint8_t slot) {
  if (slot >= Bases::kCount) {
    if (SFC_DBG) {
      Serial.print("[SFC] readBaseVolumeMl(): slot OOR ");
      Serial.println(slot);
    }
    return 0.0f;
  }
  float ratio = readBaseRatio(slot);
  float ml = m_bases[slot].cal.ratioToMl(ratio);
  if (SFC_DBG) {
    Serial.print("[SFC] readBaseVolumeMl(slot=");
    Serial.print(slot);
    Serial.print("): stub -> ");
    Serial.println(ml, 3);
  }
  return ml;
}

// ------------------------------------------------------------
// transfer (stub)
// ------------------------------------------------------------
bool SyringeFillController::transferFromBase(uint8_t slot, float ml) {
  if (slot >= Bases::kCount) {
    if (SFC_DBG) {
      Serial.print("[SFC] transferFromBase(): slot out of range: ");
      Serial.println(slot);
    }
    return false;
  }

  if (SFC_DBG) {
    Serial.print("[SFC] transferFromBase(slot=");
    Serial.print(slot);
    Serial.print(", ml=");
    Serial.print(ml, 3);
    Serial.println(") STUB");
  }

  // TODO: real synchronized transfer
  return true;
}

} // namespace App
