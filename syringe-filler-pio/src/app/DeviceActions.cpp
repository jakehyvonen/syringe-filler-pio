/**
 * @file DeviceActions.cpp
 * @brief Implementations for high-level hardware actions.
 */
#include "app/DeviceActions.hpp"

#include <math.h>

#include "hw/Bases.hpp"
#include "hw/Drivers.hpp"
#include "hw/Pots.hpp"
#include "hw/RFID.hpp"
#include "hw/BaseRFID.hpp"
#include "hw/Pins.hpp"
#include "motion/Axis.hpp"
#include "motion/AxisPair.hpp"
#include "motion/Homing.hpp"
#include "servo/Toolhead.hpp"

namespace App {
namespace {
inline long mmToSteps(float mm) { return (long)lround(mm * Pins::STEPS_PER_MM); }
}

namespace DeviceActions {

// Gantry / axis 1
// Report the current gantry position in steps and millimeters.
PositionResult gantryPosition() {
  PositionResult res{};
  res.steps = Axis::current();
  res.mm = res.steps / Pins::STEPS_PER_MM;
  res.ok = true;
  res.message = "gantry position";
  return res;
}

// Set the gantry speed in steps per second.
ActionResult setGantrySpeed(long sps) {
  Axis::setSpeedSPS(sps);
  return {true, "gantry speed updated"};
}

// Home the gantry and reset its logical position.
ActionResult homeGantry() {
  Homing::home();
  return {true, "gantry homed"};
}

// Move the gantry to an absolute step position.
ActionResult moveGantryToSteps(long targetSteps) {
  if (targetSteps < Pins::MIN_POS_STEPS) targetSteps = Pins::MIN_POS_STEPS;
  if (targetSteps > Pins::MAX_POS_STEPS) targetSteps = Pins::MAX_POS_STEPS;
  Axis::enable(true);
  Axis::moveTo(targetSteps);
  Axis::enable(false);
  return {true, "gantry moved"};
}

// Move the gantry to an absolute position in millimeters.
ActionResult moveGantryToMm(float targetMm) { return moveGantryToSteps(mmToSteps(targetMm)); }

// Bases
// Select a base by 1-based index.
ActionResult selectBase(uint8_t idx) {
  if (!Bases::select(idx)) {
    return {false, "invalid base index"};
  }
  return {true, "base selected"};
}

// Return the currently selected base index.
uint8_t selectedBase() { return Bases::selected(); }

// Move the gantry to the stored position for a base index.
ActionResult moveToBase(uint8_t idx, long &targetSteps) {
  if (idx < 1 || idx > Pins::NUM_BASES) {
    return {false, "base index out of range"};
  }
  targetSteps = Bases::getPos(idx);
  if (targetSteps < 0) {
    return {false, "base position unknown"};
  }
  if (targetSteps < Pins::MIN_POS_STEPS || targetSteps > Pins::MAX_POS_STEPS) {
    return {false, "base target outside limits"};
  }
  Axis::enable(true);
  Axis::moveTo(targetSteps);
  Axis::enable(false);
  return {true, "moved to base"};
}

// Servos / toolhead
// Set a raw servo pulse width in microseconds.
ActionResult setServoPulseRaw(int channel, int us) {
  Toolhead::setPulseRaw(channel, us);
  return {true, "servo pulse set"};
}

// Set a servo channel to a target angle.
ActionResult setServoAngle(int channel, int angle) {
  if (channel < 0 || channel > 15) {
    return {false, "channel must be 0-15"};
  }
  Toolhead::setAngle(channel, angle);
  return {true, "servo angle set"};
}

// Sweep a servo channel to a target angle with delay.
ActionResult setServoAngleSlow(int channel, int angle, int delayMs) {
  if (channel < 0 || channel > 15) {
    return {false, "channel must be 0-15"};
  }
  Toolhead::setAngleSlow(channel, angle, delayMs);
  return {true, "servo angle set slowly"};
}

// Raise the toolhead to the safe position.
ActionResult raiseToolhead() {
  Toolhead::raise();
  return {true, "toolhead raised"};
}

// Execute the coupling sequence for the syringes.
ActionResult coupleSyringes() {
  Toolhead::couple();
  return {true, "syringes coupled"};
}

// Secondary axes
// Move axis 2 by a signed step count.
ActionResult moveAxis2(long steps) {
  AxisPair::move2(steps);
  return {true, "axis2 moved"};
}

// Move axis 3 by a signed step count.
ActionResult moveAxis3(long steps) {
  AxisPair::move3(steps);
  return {true, "axis3 moved"};
}

// Set the shared speed for axes 2 and 3.
ActionResult setAxis23Speed(long sps) {
  AxisPair::setSpeedSPS(sps);
  return {true, "axis2/3 speed set"};
}

// Move axes 2 and 3 synchronously with an optional base requirement.
ActionResult moveAxisSync(long steps2, long steps3, bool requireBaseSelected) {
  if (requireBaseSelected && steps3 != 0 && Bases::selected() == 0) {
    return {false, "base required for axis3"};
  }
  AxisPair::moveSync(steps2, steps3);
  return {true, "axes moved"};
}

// Report the current axis 2 position in steps and millimeters.
PositionResult axis2Position() {
  PositionResult res{};
  res.steps = AxisPair::pos2();
  res.mm = res.steps / Pins::STEPS_PER_MM;
  res.ok = true;
  res.message = "axis2 position";
  return res;
}

// Report the current axis 3 position in steps and millimeters.
PositionResult axis3Position() {
  PositionResult res{};
  res.steps = AxisPair::pos3();
  res.mm = res.steps / Pins::STEPS_PER_MM;
  res.ok = true;
  res.message = "axis3 position";
  return res;
}

// RFID
// Handle toolhead RFID commands.
ActionResult handleRfidCommand(const String &arg) {
  if (arg == "on") {
    RFID::enable(true);
    return {true, "rfid polling enabled"};
  }
  if (arg == "off") {
    RFID::enable(false);
    return {true, "rfid polling disabled"};
  }
  if (arg == "once") {
    RFID::detectOnce(30, 100);
    return {true, "rfid detect once"};
  }
  return {false, "usage: rfid on|off|once"};
}

// Handle base RFID commands.
ActionResult handleBaseRfidCommand(const String &arg) {
  if (arg == "on") {
    BaseRFID::enable(true);
    return {true, "base rfid polling enabled"};
  }
  if (arg == "off") {
    BaseRFID::enable(false);
    return {true, "base rfid polling disabled"};
  }
  if (arg.startsWith("once")) {
    int tries = 30;
    int spaceIndex = arg.indexOf(' ');
    if (spaceIndex > 0) {
      int val = arg.substring(spaceIndex + 1).toInt();
      if (val > 0 && val < 200) tries = val;
    }
    BaseRFID::detectOnce(tries, 100);
    return {true, "base rfid detect once"};
  }
  return {false, "usage: rfid2 on|off|once [tries]"};
}

// Syringe fill controller helpers
// Scan all base syringes and populate controller state.
ActionResult sfcScanBases(App::SyringeFillController &sfc) {
  sfc.scanAllBaseSyringes();
  return {true, "scanned all base syringes"};
}

// Execute the currently loaded recipe.
ActionResult sfcRunRecipe(App::SyringeFillController &sfc) {
  sfc.runRecipe();
  return {true, "recipe completed"};
}

// Load the toolhead recipe from storage.
ActionResult sfcLoadRecipe(App::SyringeFillController &sfc) {
  return {sfc.loadToolheadRecipeFromFS(), "recipe loaded"};
}

// Save the toolhead recipe to storage.
ActionResult sfcSaveRecipe(App::SyringeFillController &sfc) {
  return {sfc.saveToolheadRecipeToFS(), "recipe saved"};
}

// Report a placeholder status until a richer status API exists.
ActionResult sfcStatus() {
  // Not exposed via controller; stub for structured output.
  return {true, "status not exposed"};
}

// Scan a single base syringe slot.
ActionResult sfcScanBase(App::SyringeFillController &sfc, uint8_t slot) {
  sfc.scanBaseSyringe(slot);
  return {true, "base scanned"};
}

// Scan the toolhead syringe RFID tag.
ActionResult sfcScanTool(App::SyringeFillController &sfc) {
  bool ok = sfc.scanToolheadBlocking();
  return {ok, ok ? "toolhead scan ok" : "toolhead scan failed"};
}

// Capture a toolhead calibration point at the given volume.
ActionResult sfcCaptureToolCalPoint(App::SyringeFillController &sfc, float ml) {
  String message;
  bool ok = sfc.captureToolheadCalibrationPoint(ml, message);
  return {ok, message.length() ? message : (ok ? "toolhead syringe point saved" : "toolhead syringe point failed")};
}

// Print toolhead calibration info to serial.
ActionResult sfcShowTool(App::SyringeFillController &sfc) {
  sfc.printToolheadInfo(Serial);
  return {true, "toolhead info printed"};
}

// Build a JSON volume report for scanned syringes.
ActionResult showVolumes(App::SyringeFillController &sfc, String &data) {
  String message;
  bool ok = sfc.showVolumes(data, message);
  return {ok, message};
}

// Convenience wrapper for recipe save.
ActionResult sfcRecipeSave(App::SyringeFillController &sfc) { return sfcSaveRecipe(sfc); }

// Convenience wrapper for recipe load.
ActionResult sfcRecipeLoad(App::SyringeFillController &sfc) { return sfcLoadRecipe(sfc); }

// Print the current base info if one is selected.
ActionResult sfcShowCurrentBase(App::SyringeFillController &sfc) {
  int8_t slot = sfc.currentSlot();
  if (slot < 0) {
    return {false, "no current base"};
  }
  sfc.printBaseInfo((uint8_t)slot, Serial);
  return {true, "base info printed"};
}

// Capture a base calibration point for a slot or current base.
ActionResult sfcCaptureBaseCalPoint(App::SyringeFillController &sfc, float ml, int8_t slot) {
  String message;
  bool ok = false;
  if (slot < 0) {
    ok = sfc.captureCurrentBaseCalibrationPoint(ml, message);
    if (!ok && message.length() == 0) {
      message = "no current base";
    }
  } else {
    ok = sfc.captureBaseCalibrationPoint((uint8_t)slot, ml, message);
  }
  return {ok, message.length() ? message : (ok ? "calibration point saved" : "calibration point failed")};
}

// Clear calibration points for the current base.
ActionResult sfcClearBaseCalPoints(App::SyringeFillController &sfc) {
  String message;
  bool ok = sfc.clearCurrentBaseCalibrationPoints(message);
  return {ok, message.length() ? message : (ok ? "base calibration points cleared" : "clear failed")};
}

// Clear calibration points for the toolhead.
ActionResult sfcClearToolCalPoints(App::SyringeFillController &sfc) {
  String message;
  bool ok = sfc.clearToolheadCalibrationPoints(message);
  return {ok, message.length() ? message : (ok ? "toolhead calibration points cleared" : "clear failed")};
}

// Force the current base calibration through zero.
ActionResult sfcForceBaseCalZero(App::SyringeFillController &sfc) {
  String message;
  bool ok = sfc.forceCurrentBaseCalibrationZero(message);
  return {ok, message.length() ? message : (ok ? "base calibration forced to 0 mL" : "update failed")};
}

// Force the toolhead calibration through zero.
ActionResult sfcForceToolCalZero(App::SyringeFillController &sfc) {
  String message;
  bool ok = sfc.forceToolheadCalibrationZero(message);
  return {ok, message.length() ? message : (ok ? "toolhead calibration forced to 0 mL" : "update failed")};
}

ActionResult sfcTransferFromBase(App::SyringeFillController &sfc, uint8_t slot, float ml) {
  bool ok = sfc.transferFromBase(slot, ml);
  return {ok, ok ? "transfer completed" : "transfer failed"};
}

// Pots
// Read a pot channel in raw and scaled units.
ActionResult readPot(uint8_t idx, uint16_t &counts, uint16_t &scaled) {
  counts = Pots::readCounts(idx);
  scaled = Pots::readScaled(idx);
  return {true, "pot read"};
}

// Read the pot mapped to a base slot.
ActionResult readBasePot(uint8_t base, uint8_t &potIdx, uint16_t &counts, uint16_t &scaled) {
  potIdx = Pins::BASE_POT_IDX[base];
  counts = Pots::readCounts(potIdx);
  scaled = Pots::readScaled(potIdx);
  return {true, "base pot read"};
}

// Build a JSON report of all pot readings.
ActionResult readAllPots(String &data) {
  String out = "[";
  for (uint8_t i = 0; i < Pots::NUM_POTS; ++i) {
    if (i > 0) {
      out += ",";
    }
    uint16_t counts = Pots::readCounts(i);
    float percent = Pots::ratioFromCounts(counts);
    float ratio = percent / 100.0f;
    out += "{\"index\":" + String(i);
    out += ",\"counts\":" + String(counts);
    out += ",\"ratio\":" + String(ratio, 5);
    out += ",\"percent\":" + String(percent, 3);
    out += "}";
  }
  out += "]";
  data = "{\"pots\":" + out + "}";
  return {true, "pot readings reported"};
}

// Pot-driven motion
// Perform pot-driven motion on axis 2.
ActionResult potMove(uint16_t target, long sps) {
  bool ok = AxisPair::move2UntilPotSimple(target, sps);
  return {ok, ok ? "potmove ok" : "potmove failed"};
}

}  // namespace DeviceActions
}  // namespace App
