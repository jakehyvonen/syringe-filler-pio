/**
 * @file SyringeFillController.cpp
 * @brief Orchestrates base/toolhead scanning and recipe execution.
 */
#include "app/SyringeFillController.hpp"

#include "hw/Bases.hpp"
#include "motion/Axis.hpp"
#include "motion/AxisPair.hpp"
#include "hw/RFID.hpp"
#include "hw/BaseRFID.hpp"
#include "util/Storage.hpp"   // Util::loadBase, Util::saveBase, Util::loadRecipe, ...
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

namespace App {

namespace {
  constexpr bool DEBUG_FLAG = true;
  constexpr long kBaseRfidScanErrorThreshold = 2300;
  constexpr uint32_t kBaseRfidScanIntervalMs = 75;

  // used by BaseRFID listener
  struct BaseTagCapture {
    bool     got    = false;
    uint32_t packed = 0;
  };

  // used by Toolhead RFID listener
  struct ToolheadTagCapture {
    bool     got    = false;
    uint32_t packed = 0;
  };

  struct BaseRfidMoveScanContext {
    BaseTagCapture* capture = nullptr;
    uint32_t lastPollMs = 0;
  };

  // listener that BaseRFID will call
  // Pack a base RFID UID into a 32-bit value for capture.
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

  // listener that Toolhead RFID will call
  // Pack a toolhead RFID UID into a 32-bit value for capture.
  void toolheadTagHandler(const uint8_t* uid, uint8_t len, void* user) {
    auto* cap = static_cast<ToolheadTagCapture*>(user);
    if (!cap) return;

    uint32_t v = 0;
    for (uint8_t i = 0; i < len && i < 4; ++i) {
      v = (v << 8) | uid[i];
    }
    cap->packed = v;
    cap->got    = true;
  }

  void baseRfidMoveHook(long errSteps, void* user) {
    auto* ctx = static_cast<BaseRfidMoveScanContext*>(user);
    if (!ctx || !ctx->capture || ctx->capture->got) {
      return;
    }
    if (labs(errSteps) > kBaseRfidScanErrorThreshold) {
      return;
    }
    uint32_t now = millis();
    if (now - ctx->lastPollMs < kBaseRfidScanIntervalMs) {
      return;
    }
    ctx->lastPollMs = now;
    BaseRFID::tick();
  }

  // Print a debug message when DEBUG_FLAG is enabled.
  void dbg(const char* msg) {
    if (DEBUG_FLAG) {
      Serial.print("[SFC] ");
      Serial.println(msg);
    }
  }

}

// ------------------------------------------------------------
// ctor
// ------------------------------------------------------------
// Initialize controller state and base slot mappings.
SyringeFillController::SyringeFillController()
  : m_calibration(m_toolhead, m_bases, Bases::kCount, m_baseToPot, m_currentSlot) {
  if (DEBUG_FLAG) {
    Serial.println("[SFC] ctor: init base slots");
  }
  for (uint8_t i = 0; i < Bases::kCount; ++i) {
    m_bases[i] = Syringe{};
    m_bases[i].slot = i;
  }
  m_currentSlot = -1;
  // Physical mapping of pots.
  m_baseToPot[0] = 3;
  m_baseToPot[1] = 4;
  m_baseToPot[2] = 5;
  m_baseToPot[3] = 0;
  m_baseToPot[4] = 1;
}


// ------------------------------------------------------------
// scan all / scan one base
// ------------------------------------------------------------
// Scan all base slots and load any detected syringes.
void SyringeFillController::scanAllBaseSyringes() {
  dbg("scanAllBaseSyringes() start");
  for (uint8_t i = 0; i < Bases::kCount; ++i) {
    if (!scanBaseSyringe(i)) {
      if (DEBUG_FLAG) {
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
// Scan a single base slot and load its syringe data.
bool SyringeFillController::scanBaseSyringe(uint8_t slot) {
  if (slot >= Bases::kCount) {
    if (DEBUG_FLAG) {
      Serial.print("[SFC] scanBaseSyringe(): slot out of range ");
      Serial.println(slot);
    }
    return false;
  }

  BaseTagCapture cap;
  cap.got    = false;
  cap.packed = 0;

  const bool wasEnabled = BaseRFID::enabled();
  BaseRFID::setListener(baseTagHandler, &cap);
  if (!wasEnabled) {
    BaseRFID::enable(true);
  }

  BaseRfidMoveScanContext ctx;
  ctx.capture = &cap;
  ctx.lastPollMs = millis();

  if (!goToBase(slot, baseRfidMoveHook, &ctx)) {
    BaseRFID::setListener(nullptr, nullptr);
    if (!wasEnabled) {
      BaseRFID::enable(false);
    }
    if (DEBUG_FLAG) {
      Serial.print("[SFC] scanBaseSyringe(): goToBase(");
      Serial.print(slot);
      Serial.println(") failed");
    }
    return false;
  }

  BaseRFID::setListener(nullptr, nullptr);
  if (!wasEnabled) {
    BaseRFID::enable(false);
  }

  uint32_t tag = cap.got ? cap.packed : 0;
  if (tag == 0) {
    if (DEBUG_FLAG) Serial.println("[SFC] scanBaseSyringe(): calling readBaseRFIDBlocking()");
    tag = readBaseRFIDBlocking(2000); // 2s timeout
  }
  if (tag == 0) {
    if (DEBUG_FLAG) {
      Serial.print("[SFC] scanBaseSyringe(): no tag at slot ");
      Serial.println(slot);
    }
    return false;
  }

  if (DEBUG_FLAG) {
    Serial.print("[SFC] scanBaseSyringe(): slot ");
    Serial.print(slot);
    Serial.print(" -> tag 0x");
    Serial.println(tag, HEX);
  }

  m_calibration.initializeBaseFromTag(slot, tag);

  Syringe& s = m_bases[slot];
  s.currentMl = m_calibration.readBaseVolumeMl(slot);
  if (DEBUG_FLAG) {
    Serial.print("[SFC] scanBaseSyringe(): measured ~");
    Serial.print(s.currentMl, 3);
    Serial.println(" mL");
  }

  return true;
}
// Capture a toolhead calibration point via the calibration helper.
bool SyringeFillController::captureToolheadCalibrationPoint(float ml, String& message) {
  return m_calibration.captureToolheadCalibrationPoint(ml, message);
}

// Clear calibration points for the current base.
bool SyringeFillController::clearCurrentBaseCalibrationPoints(String& message) {
  return m_calibration.clearCurrentBaseCalibrationPoints(message);
}

// Clear calibration points for the toolhead.
bool SyringeFillController::clearToolheadCalibrationPoints(String& message) {
  return m_calibration.clearToolheadCalibrationPoints(message);
}

// Force the current base calibration through zero.
bool SyringeFillController::forceCurrentBaseCalibrationZero(String& message) {
  return m_calibration.forceCurrentBaseCalibrationZero(message);
}

// Force the toolhead calibration through zero.
bool SyringeFillController::forceToolheadCalibrationZero(String& message) {
  return m_calibration.forceToolheadCalibrationZero(message);
}
// Capture a base calibration point for a slot.
bool SyringeFillController::captureBaseCalibrationPoint(uint8_t slot, float ml, String& message) {
  return m_calibration.captureBaseCalibrationPoint(slot, ml, message);
}

// Print base calibration info for a slot.
void SyringeFillController::printBaseInfo(uint8_t slot, Stream& s) {
  m_calibration.printBaseInfo(slot, s);
}
// ------------------------------------------------------------
// blocking base-RFID read (with tick)
// ------------------------------------------------------------
// Block until a base RFID tag is read or timeout expires.
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

// Block until a toolhead RFID tag is read.
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
// Block until a toolhead RFID tag is read or timeout expires.
uint32_t SyringeFillController::readToolheadRFIDBlocking(uint32_t timeoutMs) {
  Serial.print("[SFC] readToolheadRFIDBlocking(): start, timeout=");
  Serial.print(timeoutMs);
  Serial.println(" ms");

  ToolheadTagCapture cap;
  cap.got    = false;
  cap.packed = 0;

  const bool wasEnabled = RFID::enabled();
  Serial.print("[SFC] readToolheadRFIDBlocking(): RFID was ");
  Serial.println(wasEnabled ? "ENABLED" : "DISABLED");

  RFID::setListener(toolheadTagHandler, &cap);
  Serial.println("[SFC] readToolheadRFIDBlocking(): listener registered");

  if (!wasEnabled) {
    RFID::enable(true);
    Serial.println("[SFC] readToolheadRFIDBlocking(): RFID enabled temporarily");
  }

  const uint32_t start   = millis();
  uint32_t       lastLog = start;
  uint32_t       iter    = 0;

  while (millis() - start < timeoutMs) {
    Serial.print("[SFC] RTHRFID iter=");
    Serial.print(iter++);
    Serial.println(" -> calling RFID::tick()");
    RFID::tick();   // force tick for toolhead PN532
    Serial.println("[SFC] ...returned from RFID::tick()");

    if (cap.got) {
      Serial.print("[SFC] readToolheadRFIDBlocking(): tag captured, packed=0x");
      Serial.println(cap.packed, HEX);
      break;
    }

    uint32_t now = millis();
    if (now - lastLog > 200) {
      Serial.print("[SFC] readToolheadRFIDBlocking(): waiting... ");
      Serial.print(now - start);
      Serial.println(" ms");
      lastLog = now;
    }
    delay(5);
  }

  RFID::setListener(nullptr, nullptr);
  Serial.println("[SFC] readToolheadRFIDBlocking(): listener cleared");

  if (!wasEnabled) {
    RFID::enable(false);
    Serial.println("[SFC] readToolheadRFIDBlocking(): RFID returned to DISABLED");
  }

  if (!cap.got) {
    Serial.println("[SFC] readToolheadRFIDBlocking(): TIMEOUT, no tag");
    return 0;
  }

  Serial.print("[SFC] readToolheadRFIDBlocking(): success, returning 0x");
  Serial.println(cap.packed, HEX);
  return cap.packed;
}

// Print toolhead calibration information.
void App::SyringeFillController::printToolheadInfo(Stream& out) {
  m_calibration.printToolheadInfo(out);
}

// Build a JSON volume report for toolhead and bases.
bool SyringeFillController::showVolumes(String& data, String& message) {
  return m_calibration.buildVolumesReport(data, message);
}


// Load a toolhead recipe from LittleFS.
bool SyringeFillController::loadToolheadRecipeFromFS() {
  if (m_toolhead.rfid == 0) {
    dbg("loadToolheadRecipeFromFS(): no toolhead RFID");
    return false;
  }
  bool ok = Util::loadRecipe(m_toolhead.rfid, m_recipe);
  if (!ok && DEBUG_FLAG) {
    Serial.print("[SFC] loadToolheadRecipeFromFS(): failed for tag 0x");
    Serial.println(m_toolhead.rfid, HEX);
  }
  return ok;
}

// Save the current toolhead recipe to LittleFS.
bool SyringeFillController::saveToolheadRecipeToFS() {
  if (m_toolhead.rfid == 0) {
    dbg("saveToolheadRecipeToFS(): no toolhead RFID");
    return false;
  }
  bool ok = Util::saveRecipe(m_toolhead.rfid, m_recipe);
  if (!ok && DEBUG_FLAG) {
    Serial.print("[SFC] saveToolheadRecipeToFS(): failed for tag 0x");
    Serial.println(m_toolhead.rfid, HEX);
  }
  return ok;
}

// ------------------------------------------------------------
// run recipe
// ------------------------------------------------------------
// Execute the current recipe by transferring from each base.
void SyringeFillController::runRecipe() {
  dbg("runRecipe() start");
  for (uint8_t i = 0; i < m_recipe.count; ++i) {
    auto& step = m_recipe.steps[i];
    if (DEBUG_FLAG) {
      Serial.print("[SFC] step ");
      Serial.print(i);
      Serial.print(": base=");
      Serial.print(step.baseSlot);
      Serial.print(" ml=");
      Serial.println(step.ml, 3);
    }
    if (!transferFromBase(step.baseSlot, step.ml)) {
      if (DEBUG_FLAG) {
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
// Move the gantry to a stored base position and update selection.
bool SyringeFillController::goToBase(uint8_t slot, Axis::MoveHook hook, void* context) {
  if (slot >= Bases::kCount) {
    if (DEBUG_FLAG) {
      Serial.print("[SFC] goToBase(): slot out of range: ");
      Serial.println(slot);
    }
    return false;
  }
  long target = Bases::positionSteps(slot);
  if (target < 0) {
    if (DEBUG_FLAG) {
      Serial.print("[SFC] goToBase(): no position for slot ");
      Serial.println(slot);
    }
    return false;
  }

  if (DEBUG_FLAG) {
    Serial.print("[SFC] goToBase(");
    Serial.print(slot);
    Serial.print(") -> steps=");
    Serial.println(target);
  }

  Axis::moveToWithHook(target, hook, context);
  m_currentSlot = slot;
  if (DEBUG_FLAG) {
    Serial.println("[SFC] goToBase: finished successfully");
  }
  return true;
}


// ------------------------------------------------------------
// toolhead reader (I2C PN532 #1)
// ------------------------------------------------------------
// Poll the toolhead RFID reader for a single tag.
uint32_t SyringeFillController::readRFIDNow() {
  const uint32_t timeoutMs = 2000;
  const uint32_t startMs   = millis();

  const bool wasEnabled = RFID::enabled();
  if (!wasEnabled) {
    RFID::enable(true);
    if (DEBUG_FLAG) Serial.println("[SFC] readRFIDNow(): RFID was disabled, enabling temporarily");
  }

  if (DEBUG_FLAG) Serial.println("[SFC] readRFIDNow(): waiting for tag...");

  while (millis() - startMs < timeoutMs) {
    // App::loop() normally calls RFID::tick(), but this is a blocking loop.
    if (RFID::available()) {
      const uint8_t* uid = RFID::uidBytes();
      uint8_t len        = RFID::uidLen();

      if (len == 0 || uid == nullptr) {
        if (DEBUG_FLAG) Serial.println("[SFC] readRFIDNow(): available() but empty UID");
        if (!wasEnabled) RFID::enable(false);
        return 0;
      }

      uint32_t packed = 0;
      for (uint8_t i = 0; i < len && i < 4; ++i) {
        packed = (packed << 8) | uid[i];
      }

      if (DEBUG_FLAG) {
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

  if (DEBUG_FLAG) Serial.println("[SFC] readRFIDNow(): timeout, no tag");
  if (!wasEnabled) RFID::enable(false);
  return 0;
}

// ------------------------------------------------------------
// transfer (stub)
// ------------------------------------------------------------
// Transfer volume from a base to the toolhead using step calculations.
bool SyringeFillController::transferFromBase(uint8_t slot, float ml) {
  if (slot >= Bases::kCount) {
    if (DEBUG_FLAG) {
      Serial.print("[SFC] transferFromBase(): slot out of range: ");
      Serial.println(slot);
    }
    return false;
  }

  if (!isfinite(ml) || ml <= 0.0f) {
    if (DEBUG_FLAG) {
      Serial.print("[SFC] transferFromBase(): invalid mL: ");
      Serial.println(ml, 3);
    }
    return false;
  }

  if (m_toolhead.rfid == 0) {
    if (DEBUG_FLAG) Serial.println("[SFC] transferFromBase(): no toolhead syringe loaded");
    return false;
  }

  if (m_bases[slot].rfid == 0) {
    if (DEBUG_FLAG) {
      Serial.print("[SFC] transferFromBase(): no base syringe loaded at slot ");
      Serial.println(slot);
    }
    return false;
  }

  if (m_currentSlot != static_cast<int8_t>(slot)) {
    if (!goToBase(slot)) {
      if (DEBUG_FLAG) {
        Serial.print("[SFC] transferFromBase(): failed to goToBase(");
        Serial.print(slot);
        Serial.println(")");
      }
      return false;
    }
  }

  if (Bases::selected() != static_cast<uint8_t>(slot + 1)) {
    if (!Bases::select(static_cast<uint8_t>(slot + 1))) {
      if (DEBUG_FLAG) {
        Serial.print("[SFC] transferFromBase(): failed to select base ");
        Serial.println(slot);
      }
      return false;
    }
  }

  // Steps per mL derivation:
  // Base syringe (M12, 1.75 mm pitch):
  //   steps/mm = 200 * microsteps / 1.75
  //   mm/mL = 15 mm / 10 mL = 1.5 mm/mL
  //   steps/mL = steps/mm * mm/mL
  // Toolhead syringe (M8, 1.25 mm pitch):
  //   steps/mm = 200 * microsteps / 1.25
  //   mm/mL = 69 mm / 20 mL = 3.45 mm/mL
  //   steps/mL = steps/mm * mm/mL
  constexpr float kMicrosteps = 2.0f; // half-step microstepping
  constexpr float kBaseStepsPerMm = (200.0f * kMicrosteps) / 1.75f;
  constexpr float kBaseMmPerMl = 1.5f;
  constexpr float kBaseStepsPerMl = kBaseStepsPerMm * kBaseMmPerMl; // ~343 steps/mL @ half-step
  constexpr float kToolStepsPerMm = (200.0f * kMicrosteps) / 1.25f;
  constexpr float kToolMmPerMl = 3.45f;
  constexpr float kToolStepsPerMl = kToolStepsPerMm * kToolMmPerMl; // ~1104 steps/mL @ half-step
  const long toolSteps = -lroundf(ml * kToolStepsPerMl);
  const long baseSteps = lroundf(ml * kBaseStepsPerMl);

  if (DEBUG_FLAG) {
    Serial.print("[SFC] transferFromBase(slot=");
    Serial.print(slot);
    Serial.print(", ml=");
    Serial.print(ml, 3);
    Serial.print(") steps tool=");
    Serial.print(toolSteps);
    Serial.print(" base=");
    Serial.println(baseSteps);
  }

  AxisPair::moveSync(toolSteps, baseSteps);
  return true;
}

} // namespace App
