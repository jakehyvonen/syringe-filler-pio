/**
 * @file DeviceActions.hpp
 * @brief High-level hardware actions used by the command router.
 */
#pragma once
#include <Arduino.h>

#include "app/SyringeFillController.hpp"

namespace App {

struct ActionResult {
  bool ok;
  String message;
};

struct PositionResult : public ActionResult {
  long steps;
  float mm;
};

namespace DeviceActions {

// Gantry / axis 1
PositionResult gantryPosition();
ActionResult setGantrySpeed(long sps);
ActionResult homeGantry();
ActionResult moveGantryToSteps(long targetSteps);
ActionResult moveGantryToMm(float targetMm);

// Bases
ActionResult selectBase(uint8_t idx0);
int8_t selectedBase();
ActionResult moveToBase(uint8_t idx0, long &targetSteps);

// Servos / toolhead
ActionResult setServoPulseRaw(int channel, int us);
ActionResult setServoAngle(int channel, int angle);
ActionResult setServoAngleSlow(int channel, int angle, int delayMs);
ActionResult raiseToolhead();
ActionResult coupleSyringes();

// Secondary axes
ActionResult moveAxis2(long steps);
ActionResult moveAxis3(long steps);
ActionResult setAxis23Speed(long sps);
ActionResult moveAxisSync(long steps2, long steps3, bool requireBaseSelected);
PositionResult axis2Position();
PositionResult axis3Position();

// RFID
ActionResult handleRfidCommand(const String &arg);
ActionResult handleBaseRfidCommand(const String &arg);

// Syringe fill controller helpers
ActionResult sfcScanBases(App::SyringeFillController &sfc);
ActionResult sfcRunRecipe(App::SyringeFillController &sfc);
ActionResult sfcLoadRecipe(App::SyringeFillController &sfc, uint32_t recipeId);
ActionResult sfcSaveRecipe(App::SyringeFillController &sfc, uint32_t recipeId);
ActionResult sfcStatus(App::SyringeFillController &sfc, String &data);
ActionResult sfcScanBase(App::SyringeFillController &sfc, uint8_t slot);
ActionResult sfcScanTool(App::SyringeFillController &sfc);
ActionResult sfcCaptureToolCalPoint(App::SyringeFillController &sfc, float ml);
ActionResult sfcShowTool(App::SyringeFillController &sfc);
ActionResult showVolumes(App::SyringeFillController &sfc, String &data);
ActionResult sfcRecipeSave(App::SyringeFillController &sfc, uint32_t recipeId);
ActionResult sfcRecipeLoad(App::SyringeFillController &sfc, uint32_t recipeId);
ActionResult sfcShowCurrentBase(App::SyringeFillController &sfc);
ActionResult sfcCaptureBaseCalPoint(App::SyringeFillController &sfc, float ml, int8_t slot);
ActionResult sfcClearBaseCalPoints(App::SyringeFillController &sfc);
ActionResult sfcClearToolCalPoints(App::SyringeFillController &sfc);
ActionResult sfcForceBaseCalZero(App::SyringeFillController &sfc);
ActionResult sfcForceToolCalZero(App::SyringeFillController &sfc);
ActionResult sfcTransferFromBase(App::SyringeFillController &sfc, uint8_t slot, float ml);
ActionResult setBreakpointsOff(App::SyringeFillController &sfc);
ActionResult setBreakpointsOn(App::SyringeFillController &sfc);

// Pots
ActionResult readPot(uint8_t idx, uint16_t &counts, uint16_t &scaled);
ActionResult readBasePot(uint8_t base, uint8_t &potIdx, uint16_t &counts, uint16_t &scaled);
ActionResult readAllPots(String &data);

// Pot-driven motion
ActionResult potMove(uint16_t target, long sps);

// Diagnostics
ActionResult i2cScanBoth();

// Encoder
ActionResult handleEncoderPollingCommand(const String &args);

}  // namespace DeviceActions
}  // namespace App
