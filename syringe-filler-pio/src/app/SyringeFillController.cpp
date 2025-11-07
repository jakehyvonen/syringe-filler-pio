#include "app/SyringeFillController.hpp"
#include "hw/BaseRFID.hpp"

using namespace App;

namespace {
  // flip to false if you want it quiet
  constexpr bool SFC_DBG = true;

  struct BaseTagCapture {
    bool     got = false;
    uint32_t packed = 0;
  };

  void baseTagHandler(const uint8_t* uid, uint8_t len, void* user) {
    auto* cap = static_cast<BaseTagCapture*>(user);
    if (!cap) return;
    // pack first up-to-4 bytes
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


uint16_t SyringeFillController::readToolheadRawADC() {
  // TODO: hook to your real Pots API + channel
  // e.g. uint16_t raw = Pots::readScaled(POTS_TOOLHEAD_CH);
  // for now:
  uint16_t raw = 0;
  if (SFC_DBG) {
    Serial.print("[SFC] readToolheadRawADC(): stub -> ");
    Serial.println(raw);
  }
  return raw;
}

bool SyringeFillController::captureToolheadEmpty() {
  if (m_toolhead.rfid == 0) {
    if (SFC_DBG) Serial.println("[SFC] captureToolheadEmpty(): no toolhead RFID yet");
    return false;
  }
  uint16_t raw = readToolheadRawADC();
  m_toolhead.cal.adcEmpty = raw;
  if (SFC_DBG) {
    Serial.print("[SFC] toolhead empty ADC = ");
    Serial.println(raw);
  }
  return true;
}

bool SyringeFillController::captureToolheadFull(float mlFull) {
  if (m_toolhead.rfid == 0) {
    if (SFC_DBG) Serial.println("[SFC] captureToolheadFull(): no toolhead RFID yet");
    return false;
  }
  uint16_t raw = readToolheadRawADC();
  m_toolhead.cal.adcFull = raw;
  m_toolhead.cal.mlFull  = mlFull;
  if (SFC_DBG) {
    Serial.print("[SFC] toolhead full ADC = ");
    Serial.print(raw);
    Serial.print("  mlFull = ");
    Serial.println(mlFull, 3);
  }
  return true;
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

SyringeFillController::SyringeFillController() {
  if (SFC_DBG) {
    Serial.println("[SFC] ctor: init base slots");
  }
  for (uint8_t i = 0; i < Bases::kCount; ++i) {
    m_bases[i] = Syringe{};
    m_bases[i].slot = i;
  }
}

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


bool SyringeFillController::scanBaseSyringe(uint8_t slot) {
  if (slot >= Bases::kCount) {
    if (SFC_DBG) {
      Serial.print("[SFC] scanBaseSyringe(): slot out of range ");
      Serial.println(slot);
    }
    return false;
  }

  dbg("[SFC] scanBaseSyringe() begin move");
  if (!goToBase(slot)) {
    if (SFC_DBG) {
      Serial.print("[SFC] WARN: goToBase(");
      Serial.print(slot);
      Serial.println(") failed");
    }
    return false;
  }

  // --- read RFID tag from BASE reader (event/listener style) ---
  uint32_t tag = readBaseRFIDBlocking(2000);   // 2s timeout
  if (tag == 0) {
    if (SFC_DBG) {
      Serial.print("[SFC] INFO: no RFID detected at base slot ");
      Serial.println(slot);
    }
    return false;
  }

  if (SFC_DBG) {
    Serial.print("[SFC] base slot ");
    Serial.print(slot);
    Serial.print(" -> RFID 0x");
    Serial.println(tag, HEX);
  }

  // --- update local record ---
  Syringe& s = m_bases[slot];
  s.rfid = tag;
  s.role = SyringeRole::Base;
  s.slot = slot;

  // --- load calibration or metadata ---
  Util::BaseMeta meta;
  App::PotCalibration cal;
  if (Util::loadBase(tag, meta, cal)) {
    s.cal = cal;
    if (SFC_DBG) {
      Serial.print("[SFC]   loaded BASE meta+cal for tag 0x");
      Serial.println(tag, HEX);
    }
  } else if (Util::loadCalibration(tag, cal)) {
    s.cal = cal;
    if (SFC_DBG) {
      Serial.print("[SFC]   loaded CAL only for tag 0x");
      Serial.println(tag, HEX);
    }
  } else {
    if (SFC_DBG) {
      Serial.print("[SFC]   no calibration found for tag 0x");
      Serial.println(tag, HEX);
    }
  }

  // --- read volume ---
  s.currentMl = readBaseVolumeMl(slot);
  if (SFC_DBG) {
    Serial.print("[SFC]   measured volume ~ ");
    Serial.print(s.currentMl, 3);
    Serial.println(" mL");
  }

  dbg("[SFC] scanBaseSyringe() done");
  return true;
}

// NOW we’re outside scanBaseSyringe()
// so we can define the helper normally:

uint32_t SyringeFillController::readBaseRFIDBlocking(uint32_t timeoutMs) {
  BaseTagCapture cap;
  cap.got    = false;
  cap.packed = 0;

  // remember current state
  bool wasEnabled = BaseRFID::enabled();

  // subscribe
  BaseRFID::setListener(baseTagHandler, &cap);

  if (!wasEnabled) {
    BaseRFID::enable(true);
  }

  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    // drive the reader ourselves while we wait
    BaseRFID::tick();
    if (cap.got) break;
    delay(5);
  }

  // cleanup
  BaseRFID::setListener(nullptr, nullptr);
  if (!wasEnabled) {
    BaseRFID::enable(false);
  }

  return cap.got ? cap.packed : 0;
}




void SyringeFillController::scanToolheadSyringe() {
  dbg("scanToolheadSyringe() start");
  uint32_t tag = readRFIDNow();
  if (tag == 0) {
    dbg("no toolhead RFID detected; leaving old toolhead data");
    return;
  }

  m_toolhead.rfid = tag;
  m_toolhead.role = SyringeRole::Toolhead;

  App::PotCalibration cal;
  if (Util::loadCalibration(tag, cal)) {
    m_toolhead.cal = cal;
    if (SFC_DBG) {
      Serial.print("[SFC] toolhead cal loaded for tag 0x");
      Serial.println(tag, HEX);
    }
  } else {
    if (SFC_DBG) {
      Serial.print("[SFC] toolhead cal NOT found for tag 0x");
      Serial.println(tag, HEX);
    }
  }

  m_toolhead.currentMl = readToolheadVolumeMl();
  if (SFC_DBG) {
    Serial.print("[SFC] toolhead volume ~ ");
    Serial.print(m_toolhead.currentMl, 3);
    Serial.println(" mL");
  }

  if (!loadToolheadRecipeFromFS()) {
    if (SFC_DBG) {
      Serial.println("[SFC] no recipe file for this toolhead");
    }
  } else {
    if (SFC_DBG) {
      Serial.print("[SFC] recipe loaded with ");
      Serial.print(m_recipe.count);
      Serial.println(" steps");
    }
  }
  dbg("scanToolheadSyringe() done");
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

void SyringeFillController::runRecipe() {
  dbg("runRecipe() start");
  for (uint8_t i = 0; i < m_recipe.count; ++i) {
    auto &step = m_recipe.steps[i];
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
      // you could break here, but for now we just continue
    }
  }
  m_toolhead.currentMl = readToolheadVolumeMl();
  dbg("runRecipe() done");
}

// helpers
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

  Axis::moveTo(target);   // your project’s move (non-blocking or blocking, up to you)
  return true;
}
uint32_t SyringeFillController::readRFIDNow() {
  const uint32_t timeoutMs = 2000;   // how long we wait for a tag
  const uint32_t startMs   = millis();

  // remember current state
  const bool wasEnabled = RFID::enabled();
  if (!wasEnabled) {
    RFID::enable(true);
    if (SFC_DBG) Serial.println("[SFC] readRFIDNow(): RFID was disabled, enabling temporarily");
  }

  if (SFC_DBG) Serial.println("[SFC] readRFIDNow(): waiting for tag...");

  // poll the reader ourselves because we're in a blocking function
  while (millis() - startMs < timeoutMs) {
    //RFID::tick();              // this is normally called from App::loop()
    if (RFID::available()) {
      // got something
      const uint8_t* uid = RFID::uidBytes();
      uint8_t len        = RFID::uidLen();

      if (len == 0 || uid == nullptr) {
        if (SFC_DBG) Serial.println("[SFC] readRFIDNow(): RFID::available() but empty UID");
        // restore state and bail
        if (!wasEnabled) RFID::enable(false);
        return 0;
      }

      // pack first up-to-4 bytes into uint32_t (big-endian-ish)
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

      // restore previous enabled state
      if (!wasEnabled) RFID::enable(false);
      return packed;
    }

    // small sleep so we don't hog CPU
    delay(10);
  }

  if (SFC_DBG) Serial.println("[SFC] readRFIDNow(): timeout, no tag");

  // restore previous enabled state
  if (!wasEnabled) RFID::enable(false);
  return 0;
}


float SyringeFillController::readToolheadVolumeMl() {
  // still a stub; feed 0 through cal
  float ml = m_toolhead.cal.rawToMl(0);
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
  float ml = m_bases[slot].cal.rawToMl(0);
  if (SFC_DBG) {
    Serial.print("[SFC] readBaseVolumeMl(slot=");
    Serial.print(slot);
    Serial.print("): stub -> ");
    Serial.println(ml, 3);
  }
  return ml;
}

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

  // TODO: real sync transfer
  return true;
}
