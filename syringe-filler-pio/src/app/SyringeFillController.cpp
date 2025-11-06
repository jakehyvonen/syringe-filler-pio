#include "app/SyringeFillController.hpp"

using namespace App;

namespace {
  // flip to false if you want it quiet
  constexpr bool SFC_DBG = true;

  void dbg(const char* msg) {
    if (SFC_DBG) {
      Serial.print("[SFC] ");
      Serial.println(msg);
    }
  }
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
    if (!goToBase(i)) {
      if (SFC_DBG) {
        Serial.print("[SFC] WARN: goToBase(");
        Serial.print(i);
        Serial.println(") failed; skipping RFID read");
      }
      continue;
    }

    uint32_t tag = readRFIDNow();
    if (tag == 0) {
      if (SFC_DBG) {
        Serial.print("[SFC] INFO: no RFID at base slot ");
        Serial.println(i);
      }
      continue;
    }

    if (SFC_DBG) {
      Serial.print("[SFC] base slot ");
      Serial.print(i);
      Serial.print(" -> RFID 0x");
      Serial.println(tag, HEX);
    }

    m_bases[i].rfid = tag;
    m_bases[i].role = SyringeRole::Base;
    m_bases[i].slot = i;

    Util::BaseMeta meta;
    App::PotCalibration cal;
    if (Util::loadBase(tag, meta, cal)) {
      m_bases[i].cal = cal;
      if (SFC_DBG) {
        Serial.print("[SFC]   loaded BASE meta+cal from NVS for tag 0x");
        Serial.println(tag, HEX);
      }
    } else if (Util::loadCalibration(tag, cal)) {
      m_bases[i].cal = cal;
      if (SFC_DBG) {
        Serial.print("[SFC]   loaded CAL only from NVS for tag 0x");
        Serial.println(tag, HEX);
      }
    } else {
      if (SFC_DBG) {
        Serial.print("[SFC]   no calibration in NVS for tag 0x");
        Serial.println(tag, HEX);
      }
    }

    m_bases[i].currentMl = readBaseVolumeMl(i);
    if (SFC_DBG) {
      Serial.print("[SFC]   measured volume ~ ");
      Serial.print(m_bases[i].currentMl, 3);
      Serial.println(" mL");
    }
  }
  dbg("scanAllBaseSyringes() done");
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
    RFID::tick();              // this is normally called from App::loop()
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
