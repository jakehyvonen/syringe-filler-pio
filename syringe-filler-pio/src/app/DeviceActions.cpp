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
PositionResult gantryPosition() {
  PositionResult res{};
  res.steps = Axis::current();
  res.mm = res.steps / Pins::STEPS_PER_MM;
  res.ok = true;
  res.message = "gantry position";
  return res;
}

ActionResult setGantrySpeed(long sps) {
  Axis::setSpeedSPS(sps);
  return {true, "gantry speed updated"};
}

ActionResult homeGantry() {
  Homing::home();
  return {true, "gantry homed"};
}

ActionResult moveGantryToSteps(long targetSteps) {
  if (targetSteps < Pins::MIN_POS_STEPS) targetSteps = Pins::MIN_POS_STEPS;
  if (targetSteps > Pins::MAX_POS_STEPS) targetSteps = Pins::MAX_POS_STEPS;
  Axis::enable(true);
  Axis::moveTo(targetSteps);
  Axis::enable(false);
  return {true, "gantry moved"};
}

ActionResult moveGantryToMm(float targetMm) { return moveGantryToSteps(mmToSteps(targetMm)); }

// Bases
ActionResult selectBase(uint8_t idx) {
  if (!Bases::select(idx)) {
    return {false, "invalid base index"};
  }
  return {true, "base selected"};
}

uint8_t selectedBase() { return Bases::selected(); }

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
ActionResult setServoPulseRaw(int channel, int us) {
  Toolhead::setPulseRaw(channel, us);
  return {true, "servo pulse set"};
}

ActionResult setServoAngle(int channel, int angle) {
  if (channel < 0 || channel > 15) {
    return {false, "channel must be 0-15"};
  }
  Toolhead::setAngle(channel, angle);
  return {true, "servo angle set"};
}

ActionResult setServoAngleSlow(int channel, int angle, int delayMs) {
  if (channel < 0 || channel > 15) {
    return {false, "channel must be 0-15"};
  }
  Toolhead::setAngleSlow(channel, angle, delayMs);
  return {true, "servo angle set slowly"};
}

ActionResult raiseToolhead() {
  Toolhead::raise();
  return {true, "toolhead raised"};
}

ActionResult coupleSyringes() {
  Toolhead::couple();
  return {true, "syringes coupled"};
}

// Secondary axes
ActionResult moveAxis2(long steps) {
  AxisPair::move2(steps);
  return {true, "axis2 moved"};
}

ActionResult moveAxis3(long steps) {
  AxisPair::move3(steps);
  return {true, "axis3 moved"};
}

ActionResult setAxis23Speed(long sps) {
  AxisPair::setSpeedSPS(sps);
  return {true, "axis2/3 speed set"};
}

ActionResult moveAxisSync(long steps2, long steps3, bool requireBaseSelected) {
  if (requireBaseSelected && steps3 != 0 && Bases::selected() == 0) {
    return {false, "base required for axis3"};
  }
  AxisPair::moveSync(steps2, steps3);
  return {true, "axes moved"};
}

PositionResult axis2Position() {
  PositionResult res{};
  res.steps = AxisPair::pos2();
  res.mm = res.steps / Pins::STEPS_PER_MM;
  res.ok = true;
  res.message = "axis2 position";
  return res;
}

PositionResult axis3Position() {
  PositionResult res{};
  res.steps = AxisPair::pos3();
  res.mm = res.steps / Pins::STEPS_PER_MM;
  res.ok = true;
  res.message = "axis3 position";
  return res;
}

// RFID
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
ActionResult sfcScanBases(App::SyringeFillController &sfc) {
  sfc.scanAllBaseSyringes();
  return {true, "scanned all base syringes"};
}

ActionResult sfcRunRecipe(App::SyringeFillController &sfc) {
  sfc.runRecipe();
  return {true, "recipe completed"};
}

ActionResult sfcLoadRecipe(App::SyringeFillController &sfc) {
  return {sfc.loadToolheadRecipeFromFS(), "recipe loaded"};
}

ActionResult sfcSaveRecipe(App::SyringeFillController &sfc) {
  return {sfc.saveToolheadRecipeToFS(), "recipe saved"};
}

ActionResult sfcStatus() {
  // Not exposed via controller; stub for structured output.
  return {true, "status not exposed"};
}

ActionResult sfcScanBase(App::SyringeFillController &sfc, uint8_t slot) {
  sfc.scanBaseSyringe(slot);
  return {true, "base scanned"};
}

ActionResult sfcScanTool(App::SyringeFillController &sfc) {
  bool ok = sfc.scanToolheadBlocking();
  return {ok, ok ? "toolhead scan ok" : "toolhead scan failed"};
}

ActionResult sfcCaptureToolCalPoint(App::SyringeFillController &sfc, float ml) {
  String message;
  bool ok = sfc.captureToolheadCalibrationPoint(ml, message);
  return {ok, message.length() ? message : (ok ? "toolhead syringe point saved" : "toolhead syringe point failed")};
}

ActionResult sfcShowTool(App::SyringeFillController &sfc) {
  sfc.printToolheadInfo(Serial);
  return {true, "toolhead info printed"};
}

ActionResult showVolumes(App::SyringeFillController &sfc, String &data) {
  String message;
  bool ok = sfc.showVolumes(data, message);
  return {ok, message};
}

ActionResult sfcRecipeSave(App::SyringeFillController &sfc) { return sfcSaveRecipe(sfc); }

ActionResult sfcRecipeLoad(App::SyringeFillController &sfc) { return sfcLoadRecipe(sfc); }

ActionResult sfcShowCurrentBase(App::SyringeFillController &sfc) {
  int8_t slot = sfc.currentSlot();
  if (slot < 0) {
    return {false, "no current base"};
  }
  sfc.printBaseInfo((uint8_t)slot, Serial);
  return {true, "base info printed"};
}

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

ActionResult sfcClearBaseCalPoints(App::SyringeFillController &sfc) {
  String message;
  bool ok = sfc.clearCurrentBaseCalibrationPoints(message);
  return {ok, message.length() ? message : (ok ? "base calibration points cleared" : "clear failed")};
}

ActionResult sfcClearToolCalPoints(App::SyringeFillController &sfc) {
  String message;
  bool ok = sfc.clearToolheadCalibrationPoints(message);
  return {ok, message.length() ? message : (ok ? "toolhead calibration points cleared" : "clear failed")};
}

ActionResult sfcForceBaseCalZero(App::SyringeFillController &sfc) {
  String message;
  bool ok = sfc.forceCurrentBaseCalibrationZero(message);
  return {ok, message.length() ? message : (ok ? "base calibration forced to 0 mL" : "update failed")};
}

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
ActionResult readPot(uint8_t idx, uint16_t &counts, uint16_t &scaled) {
  counts = Pots::readCounts(idx);
  scaled = Pots::readScaled(idx);
  return {true, "pot read"};
}

ActionResult readBasePot(uint8_t base, uint8_t &potIdx, uint16_t &counts, uint16_t &scaled) {
  potIdx = Pins::BASE_POT_IDX[base];
  counts = Pots::readCounts(potIdx);
  scaled = Pots::readScaled(potIdx);
  return {true, "base pot read"};
}

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
ActionResult potMove(uint16_t target, long sps) {
  bool ok = AxisPair::move2UntilPotSimple(target, sps);
  return {ok, ok ? "potmove ok" : "potmove failed"};
}

}  // namespace DeviceActions
}  // namespace App
