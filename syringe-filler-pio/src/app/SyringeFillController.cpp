#include "app/SyringeFillController.hpp"

#include "hw/Bases.hpp"
#include "motion/Axis.hpp"
#include "hw/RFID.hpp"
#include "hw/BaseRFID.hpp"
#include "util/Storage.hpp"   // Util::loadBase, Util::saveBase, Util::loadRecipe, ...
#include <Arduino.h>
#include <math.h>

namespace App {

namespace {
  constexpr bool SFC_DBG = true;

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
SyringeFillController::SyringeFillController()
  : m_calibration(m_toolhead, m_bases, Bases::kCount, m_baseToPot, m_currentSlot) {
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


bool SyringeFillController::saveToolheadCalibration() {
  return m_calibration.saveToolheadCalibration();
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

  m_calibration.initializeBaseFromTag(slot, tag);

  Syringe& s = m_bases[slot];
  s.currentMl = m_calibration.readBaseVolumeMl(slot);
  if (SFC_DBG) {
    Serial.print("[SFC] scanBaseSyringe(): measured ~");
    Serial.print(s.currentMl, 3);
    Serial.println(" mL");
  }

  return true;
}
bool SyringeFillController::captureToolheadCalibrationPoint(float ml, String& message) {
  return m_calibration.captureToolheadCalibrationPoint(ml, message);
}

bool SyringeFillController::clearCurrentBaseCalibrationPoints(String& message) {
  return m_calibration.clearCurrentBaseCalibrationPoints(message);
}

bool SyringeFillController::clearToolheadCalibrationPoints(String& message) {
  return m_calibration.clearToolheadCalibrationPoints(message);
}

bool SyringeFillController::forceCurrentBaseCalibrationZero(String& message) {
  return m_calibration.forceCurrentBaseCalibrationZero(message);
}

bool SyringeFillController::forceToolheadCalibrationZero(String& message) {
  return m_calibration.forceToolheadCalibrationZero(message);
}
bool SyringeFillController::captureBaseCalibrationPoint(uint8_t slot, float ml, String& message) {
  return m_calibration.captureBaseCalibrationPoint(slot, ml, message);
}

bool SyringeFillController::saveCurrentBaseToNVS() {
  return m_calibration.saveCurrentBaseToNVS();
}

void SyringeFillController::printBaseInfo(uint8_t slot, Stream& s) {
  m_calibration.printBaseInfo(slot, s);
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

  m_calibration.initializeToolheadFromTag(tag);

  m_toolhead.currentMl = m_calibration.readToolheadVolumeMl();
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

void App::SyringeFillController::printToolheadInfo(Stream& out) {
  m_calibration.printToolheadInfo(out);
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
  m_toolhead.currentMl = m_calibration.readToolheadVolumeMl();
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
