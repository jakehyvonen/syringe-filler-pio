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
#include "servo/Toolhead.hpp"
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

namespace App {

namespace {
  // ------------------------------------------------------------
  // Tunables and derived motion constants
  // ------------------------------------------------------------
  constexpr bool DEBUG_FLAG = true;
  constexpr long kBaseRfidScanErrorThreshold = 1700;
  constexpr uint32_t kBaseRfidScanIntervalMs = 75;
  constexpr float kMicrosteps = 2.0f; // half-step microstepping
  constexpr float kBaseStepsPerMm = (200.0f * kMicrosteps) / 1.75f;
  constexpr float kBaseMmPerMl = 1.5f;
  constexpr float kBaseStepsPerMl = kBaseStepsPerMm * kBaseMmPerMl; // ~343 steps/mL @ half-step
  constexpr float kToolStepsPerMm = (200.0f * kMicrosteps) / 1.25f;
  constexpr float kToolMmPerMl = 3.45f;
  constexpr float kToolStepsPerMl = kToolStepsPerMm * kToolMmPerMl; // ~1104 steps/mL @ half-step
  constexpr float kRetractionMl = 0.17f;

  #pragma region RFID
  // ------------------------------------------------------------
  // RFID capture helpers
  // ------------------------------------------------------------
  // Captures the latest base RFID UID packed into a 32-bit value.
  struct BaseTagCapture {
    bool     got    = false;
    uint32_t packed = 0;
  };

  // Captures the latest toolhead RFID UID packed into a 32-bit value.
  struct ToolheadTagCapture {
    bool     got    = false;
    uint32_t packed = 0;
  };

  // Context passed while moving gantry so base RFID polling can be throttled.
  struct BaseRfidMoveScanContext {
    BaseTagCapture* capture = nullptr;
    uint32_t lastPollMs = 0;
  };

  // Base RFID listener callback.
  // Packs up to 4 UID bytes into a single uint32_t for easier downstream use.
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

  // Toolhead RFID listener callback.
  // Packs up to 4 UID bytes into a single uint32_t for easier downstream use.
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

  // Move hook executed during goToBase() to opportunistically poll the base reader.
  // Polling is skipped when position error is too high or polling interval is too short.
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

#pragma endregion
//do I need a comment here or something?

  #pragma region Helpers

  // ------------------------------------------------------------
  // Logging and conversion helpers
  // ------------------------------------------------------------

  // Print a debug message when DEBUG_FLAG is enabled.
  void dbg(const char* msg) {
    if (DEBUG_FLAG) {
      Serial.print("[SFC] ");
      Serial.println(msg);
    }
  }

  void printRecipeIdList(const char* context) {
    if (!DEBUG_FLAG) return;
    uint32_t ids[32];
    const size_t maxIds = sizeof(ids) / sizeof(ids[0]);
    size_t count = 0;
    if (!Util::listRecipeIds(ids, maxIds, count)) {
      Serial.print("[SFC] ");
      Serial.print(context);
      Serial.println(": unable to list /recipes");
      return;
    }
    if (count == 0) {
      Serial.print("[SFC] ");
      Serial.print(context);
      Serial.println(": no recipes found in /recipes");
      return;
    }
    Serial.print("[SFC] ");
    Serial.print(context);
    Serial.print(": available recipe IDs (");
    Serial.print(count);
    Serial.println("):");
    for (size_t i = 0; i < count; ++i) {
      Serial.print("  - 0x");
      Serial.print(ids[i], HEX);
      Serial.print(" (");
      Serial.print(ids[i]);
      Serial.println(")");
    }
  }

  // Convert mL transfer values into toolhead axis steps.
  long toolStepsForMl(float ml) { return lroundf(ml * kToolStepsPerMl); }
  // Convert mL transfer values into base axis steps.
  long baseStepsForMl(float ml) { return lroundf(ml * kBaseStepsPerMl); }

}

#pragma endregion

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

int8_t SyringeFillController::currentSlot() const {
  return Bases::selected();
}

#pragma region Base Syringes
// ------------------------------------------------------------
// Base scanning and initialization
// ------------------------------------------------------------
// Scan all base slots and load any detected syringes.
void SyringeFillController::scanAllBaseSyringes() {
  dbg("scanAllBaseSyringes() start");
  Axis::setSpeedSPS(2300);
  for (uint8_t i = 1; i < Bases::kCount; ++i) {
    if (!scanBaseSyringe(i)) {
      if (DEBUG_FLAG) {
        Serial.print("[SFC] WARN: scanBaseSyringe(");
        Serial.print(i);
        Serial.println(") failed");
      }
    }
  }
  //scan slot 0 last because of RFID scanning physics
  if (!scanBaseSyringe(0)) {
      if (DEBUG_FLAG) {
        Serial.print("[SFC] WARN: scanBaseSyringe(");
        Serial.print(0);
        Serial.println(") failed");
      }
    }

  dbg("scanAllBaseSyringes() done");
}

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


// ------------------------------------------------------------
// Calibration pass-through APIs
// ------------------------------------------------------------
// Capture a toolhead calibration point via the calibration helper.
bool SyringeFillController::captureToolheadCalibrationPoint(float ml, String& message) {
  return m_calibration.captureToolheadCalibrationPoint(ml, message);
}

// Run an automatic multi-point calibration sequence for the toolhead syringe.
bool SyringeFillController::autoCalibrateToolhead(float incrementMl, uint8_t points, String& message) {
  if (!isfinite(incrementMl) || incrementMl <= 0.0f) {
    message = "increment mL must be > 0";
    return false;
  }

  if (points < 2) {
    message = "number of points must be >= 2";
    return false;
  }

  if (m_toolhead.rfid == 0) {
    message = "no toolhead RFID; scan the toolhead first";
    return false;
  }

  if (!Toolhead::isCoupled()) {
    Toolhead::couple();
  }

  if (DEBUG_FLAG) {
    Serial.print("[SFC] autoCalibrateToolhead(incrementMl=");
    Serial.print(incrementMl, 3);
    Serial.print(", points=");
    Serial.print(points);
    Serial.println(") start");
  }

  for (uint8_t i = 0; i < points; ++i) {
    const float targetMl = incrementMl * static_cast<float>(i);
    if (!m_calibration.captureToolheadCalibrationPoint(targetMl, message)) {
      return false;
    }

    if (i + 1 >= points) {
      break;
    }

    const long withdrawSteps = -toolStepsForMl(incrementMl);
    if (withdrawSteps == 0) {
      message = "increment too small; computed 0 steps";
      return false;
    }

    if (DEBUG_FLAG) {
      Serial.print("[SFC] autoCalibrateToolhead(): withdraw ");
      Serial.print(incrementMl, 3);
      Serial.print(" mL -> ");
      Serial.print(withdrawSteps);
      Serial.println(" steps");
    }

    AxisPair::move2(withdrawSteps);
  }

  message = "toolhead auto calibration complete";
  return true;
}

// Clear calibration points for the current base.
bool SyringeFillController::clearCurrentBaseCalibrationPoints(String& message) {
  setCurrentSlot(currentSlot());
  return m_calibration.clearCurrentBaseCalibrationPoints(message);
}

// Clear calibration points for the toolhead.
bool SyringeFillController::clearToolheadCalibrationPoints(String& message) {
  return m_calibration.clearToolheadCalibrationPoints(message);
}

// Set steps-per-mL for a base calibration profile.
bool SyringeFillController::setBaseStepsPermL(uint8_t slot, float stepsPermL, String& message) {
  return m_calibration.setBaseStepsPermL(slot, stepsPermL, message);
}

// Capture a base calibration point for a slot.
bool SyringeFillController::captureBaseCalibrationPoint(uint8_t slot, float ml, String& message) {
  return m_calibration.captureBaseCalibrationPoint(slot, ml, message);
}

// Run an automatic multi-point calibration sequence for a base syringe.
bool SyringeFillController::autoCalibrateBase(float incrementMl, uint8_t points, int8_t slot, String& message) {
  if (!isfinite(incrementMl) || incrementMl <= 0.0f) {
    message = "increment mL must be > 0";
    return false;
  }

  if (points < 2) {
    message = "number of points must be >= 2";
    return false;
  }

  const int8_t activeSlot = (slot >= 0) ? slot : currentSlot();
  if (activeSlot < 0 || activeSlot >= Bases::kCount) {
    message = "no current base";
    return false;
  }

  const uint8_t baseSlot = (uint8_t)activeSlot;
  if (m_bases[baseSlot].rfid == 0) {
    message = "base has no RFID; scan the base first";
    return false;
  }

  if (currentSlot() != activeSlot && !goToBase(baseSlot)) {
    message = "failed to move to base";
    return false;
  }

  if (Bases::selected() != activeSlot && !Bases::select(baseSlot)) {
    message = "failed to select base";
    return false;
  }

  if (DEBUG_FLAG) {
    Serial.print("[SFC] autoCalibrateBase(slot=");
    Serial.print(baseSlot);
    Serial.print(", incrementMl=");
    Serial.print(incrementMl, 3);
    Serial.print(", points=");
    Serial.print(points);
    Serial.println(") start");
  }

  for (uint8_t i = 0; i < points; ++i) {
    const float targetMl = incrementMl * static_cast<float>(i);
    if (!m_calibration.captureBaseCalibrationPoint(baseSlot, targetMl, message)) {
      return false;
    }

    if (i + 1 >= points) {
      break;
    }

    const long withdrawSteps = baseStepsForMl(incrementMl);
    if (withdrawSteps == 0) {
      message = "increment too small; computed 0 steps";
      return false;
    }

    if (DEBUG_FLAG) {
      Serial.print("[SFC] autoCalibrateBase(): withdraw ");
      Serial.print(incrementMl, 3);
      Serial.print(" mL -> ");
      Serial.print(withdrawSteps);
      Serial.println(" steps");
    }

    AxisPair::move3(withdrawSteps);
  }

  message = "base auto calibration complete";
  return true;
}

// Print base calibration info for a slot.
void SyringeFillController::printBaseInfo(uint8_t slot, Stream& s) {
  m_calibration.printBaseInfo(slot, s);
}
// ------------------------------------------------------------
// RFID blocking scan helpers
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
#pragma endregion

// ------------------------------------------------------------
// Toolhead scanning and initialization
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

// ------------------------------------------------------------
// Status and recipe persistence helpers
// ------------------------------------------------------------
// Print toolhead calibration information.
void App::SyringeFillController::printToolheadInfo(Stream& out) {
  m_calibration.printToolheadInfo(out);
}

bool SyringeFillController::printScannedCalibrationInfo(Stream& out, String& message) {
  bool anyPrinted = false;

  if (m_toolhead.rfid != 0) {
    m_calibration.printToolheadInfo(out);
    anyPrinted = true;
  }

  for (uint8_t slot = 0; slot < Bases::kCount; ++slot) {
    if (m_bases[slot].rfid == 0) {
      continue;
    }
    m_calibration.printBaseInfo(slot, out);
    anyPrinted = true;
  }

  if (!anyPrinted) {
    message = "no scanned syringes";
    return false;
  }

  message = "calibration info printed";
  return true;
}

// Build a JSON volume report for toolhead and bases.
bool SyringeFillController::showVolumes(String& data, String& message) {
  return m_calibration.buildVolumesReport(data, message);
}


// Load a recipe from LittleFS by recipe ID.
bool SyringeFillController::loadRecipeFromFS(uint32_t recipeId) {
  if (recipeId == 0) {
    dbg("loadRecipeFromFS(): no recipe ID");
    printRecipeIdList("loadRecipeFromFS()");
    return false;
  }
  if (DEBUG_FLAG) {
    Serial.print("[SFC] loadRecipeFromFS(): requested 0x");
    Serial.print(recipeId, HEX);
    Serial.print(" (");
    Serial.print(recipeId);
    Serial.println(")");
  }
  bool ok = Util::loadRecipe(recipeId, m_recipe);
  if (!ok && DEBUG_FLAG) {
    Serial.print("[SFC] loadRecipeFromFS(): failed for recipe 0x");
    Serial.println(recipeId, HEX);
    printRecipeIdList("loadRecipeFromFS()");
    return false;
  }
  if (DEBUG_FLAG) {
    Serial.print("[SFC] loadRecipeFromFS(): loaded recipe with ");
    Serial.print(m_recipe.count);
    Serial.println(" step(s)");
    for (uint8_t i = 0; i < m_recipe.count; ++i) {
      Serial.print("[SFC] recipe step ");
      Serial.print(i);
      Serial.print(": base=");
      Serial.print(m_recipe.steps[i].baseSlot);
      Serial.print(" ml=");
      Serial.println(m_recipe.steps[i].ml, 3);
    }
    if (m_recipe.count == 0) {
      Serial.println("[SFC] loadRecipeFromFS(): recipe has zero steps");
    }
  }
  return ok;
}

// Save the current recipe to LittleFS by recipe ID.
bool SyringeFillController::saveRecipeToFS(uint32_t recipeId) {
  if (recipeId == 0) {
    dbg("saveRecipeToFS(): no recipe ID");
    return false;
  }
  if (DEBUG_FLAG) {
    Serial.print("[SFC] saveRecipeToFS(): saving recipe with ");
    Serial.print(m_recipe.count);
    Serial.println(" step(s)");
    for (uint8_t i = 0; i < m_recipe.count; ++i) {
      Serial.print("[SFC] recipe step ");
      Serial.print(i);
      Serial.print(": base=");
      Serial.print(m_recipe.steps[i].baseSlot);
      Serial.print(" ml=");
      Serial.println(m_recipe.steps[i].ml, 3);
    }
  }
  bool ok = Util::saveRecipe(recipeId, m_recipe);
  if (!ok && DEBUG_FLAG) {
    Serial.print("[SFC] saveRecipeToFS(): failed for recipe 0x");
    Serial.println(recipeId, HEX);
  }
  return ok;
}

// ------------------------------------------------------------
// Recipe execution and breakpoint control
// ------------------------------------------------------------
// Execute the current recipe by transferring from each base.
void SyringeFillController::setBreakpointsEnabled(bool enabled) {
  m_breakpointsEnabled = enabled;
  Serial.print("[SFC] breakpoints ");
  Serial.println(m_breakpointsEnabled ? "enabled" : "disabled");
}

bool SyringeFillController::breakpointsEnabled() const { return m_breakpointsEnabled; }

// Block at serial breakpoints between recipe steps when enabled.
bool SyringeFillController::serialBreakpoint(const String &label) {
  if (!m_breakpointsEnabled) {
    return true;
  }

  Serial.println("[SFC] --- BREAKPOINT ---");
  if (label.length()) {
    Serial.print("[SFC] ");
    Serial.println(label);
  }
  Serial.println("[SFC] Send 'c' to continue or 'breakPointsOff' to disable breakpoints.");

  String input;
  while (true) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        input.trim();
        if (input == "c") {
          Serial.println("[SFC] breakpoint continue");
          return true;
        }
        if (input == "breakPointsOff") {
          setBreakpointsEnabled(false);
          Serial.println("[SFC] breakpoint continue (breakpoints disabled)");
          return true;
        }
        if (input.length()) {
          Serial.print("[SFC] Unknown breakpoint command: ");
          Serial.println(input);
        }
        Serial.println("[SFC] Waiting... send 'c' or 'breakPointsOff'.");
        input = "";
      } else {
        input += c;
      }
    }
    delay(5);
  }
}

void SyringeFillController::runRecipe() {
  dbg("runRecipe() start");
  if (DEBUG_FLAG) {
    Serial.print("[SFC] runRecipe(): step count=");
    Serial.print(m_recipe.count);
    Serial.print(" toolheadRFID=0x");
    Serial.print(m_toolhead.rfid, HEX);
    Serial.print(" currentSlot=");
    Serial.println(currentSlot());
  }
  if (m_recipe.count == 0) {
    if (DEBUG_FLAG) {
      Serial.println("[SFC] runRecipe(): no steps to run, aborting");
      printRecipeIdList("runRecipe()");
    }
    return;
  }
  bool anyTransfer = false;
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
      continue;
    }
    anyTransfer = true;
    serialBreakpoint(String("runRecipe step ") + String(i) + " complete");
    if (i + 1 < m_recipe.count) {
      if (DEBUG_FLAG) {
        Serial.println("[SFC] runRecipe(): retracting between steps");
      }
      if (!retractToolhead(kRetractionMl) && DEBUG_FLAG) {
        Serial.println("[SFC] WARN: retraction failed between steps");
      }
    }
  }
  if (anyTransfer) {
    if (DEBUG_FLAG) {
      Serial.println("[SFC] runRecipe(): retracting after recipe");
    }
    if (!retractToolhead(kRetractionMl) && DEBUG_FLAG) {
      Serial.println("[SFC] WARN: retraction failed after recipe");
    }
  }
  m_toolhead.currentMl = m_calibration.readToolheadVolumeMl();
  if (DEBUG_FLAG) {
    Serial.print("[SFC] runRecipe(): toolhead current mL=");
    Serial.println(m_toolhead.currentMl, 3);
  }
  dbg("runRecipe() done");
}

// ------------------------------------------------------------
// Positioning and live RFID polling
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

  if (!Toolhead::isRaised()) {
    Toolhead::raise();
  }
  Axis::moveToWithHook(target, hook, context);
  setCurrentSlot(slot);
  if (DEBUG_FLAG) {
    Serial.println("[SFC] goToBase: finished successfully");
  }
  return true;
}


// ------------------------------------------------------------
// One-shot RFID reads
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
// Transfer and retraction motion
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

  if (currentSlot() != static_cast<int8_t>(slot)) {
    if (!goToBase(slot)) {
      if (DEBUG_FLAG) {
        Serial.print("[SFC] transferFromBase(): failed to goToBase(");
        Serial.print(slot);
        Serial.println(")");
      }
      return false;
    }
  }

  if (Bases::selected() != static_cast<int8_t>(slot)) {
    if (!Bases::select(slot)) {
      if (DEBUG_FLAG) {
        Serial.print("[SFC] transferFromBase(): failed to select base ");
        Serial.println(slot);
      }
      return false;
    }
  }

  if (!Toolhead::isCoupled()) {
    Toolhead::couple();
  }

  const long toolSteps = -toolStepsForMl(ml);
  const long baseSteps = baseStepsForMl(ml);

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

// Retract a small volume on the toolhead to reduce drips between actions.
bool SyringeFillController::retractToolhead(float ml) {
  if (!isfinite(ml) || ml <= 0.0f) {
    if (DEBUG_FLAG) {
      Serial.print("[SFC] retractToolhead(): invalid mL: ");
      Serial.println(ml, 3);
    }
    return false;
  }

  if (m_toolhead.rfid == 0) {
    if (DEBUG_FLAG) Serial.println("[SFC] retractToolhead(): no toolhead syringe loaded");
    return false;
  }

  if (!Toolhead::isCoupled()) {
    Toolhead::couple();
  }

  const long retractSteps = -toolStepsForMl(ml);
  if (retractSteps == 0) {
    return true;
  }

  if (DEBUG_FLAG) {
    Serial.print("[SFC] retractToolhead(ml=");
    Serial.print(ml, 3);
    Serial.print(") steps=");
    Serial.println(retractSteps);
  }

  AxisPair::move2(retractSteps);
  return true;
}

// Build a compact controller status snapshot for command responses.
bool SyringeFillController::buildStatusJson(String& data) const {
  String out = "{";

  out += "\"toolhead\":{";
  out += "\"rfid\":\"0x" + String(m_toolhead.rfid, HEX) + "\"";
  out += ",\"calPoints\":" + String(m_toolhead.cal.pointCount);
  out += "}";

  const int8_t slot = currentSlot();
  out += ",\"currentBase\":{";
  out += "\"slot\":" + String(slot);
  if (slot >= 0 && slot < Bases::kCount) {
    out += ",\"rfid\":\"0x" + String(m_bases[slot].rfid, HEX) + "\"";
  }
  out += "}";

  uint8_t scanned = 0;
  for (uint8_t i = 0; i < Bases::kCount; ++i) {
    if (m_bases[i].rfid != 0) {
      scanned++;
    }
  }
  out += ",\"bases\":{";
  out += "\"scanned\":" + String(scanned);
  out += ",\"total\":" + String(Bases::kCount);
  out += "}";

  out += ",\"recipe\":{";
  out += "\"steps\":" + String(m_recipe.count);
  out += "}";

  out += "}";
  data = out;
  return true;
}

} // namespace App
