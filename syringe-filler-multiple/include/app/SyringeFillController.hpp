/**
 * @file SyringeFillController.hpp
 * @brief High-level syringe fill orchestration and recipe handling.
 */
#pragma once
#include <Arduino.h>
#include "app/SyringeCalibration.hpp"
#include "app/Syringe.hpp"
#include "hw/Bases.hpp"
#include "motion/Axis.hpp"
#include "util/Storage.hpp"   // for loadBase/saveBase etc.

namespace App {

class SyringeFillController {
public:
  SyringeFillController();

  void scanAllBaseSyringes();
  bool scanBaseSyringe(uint8_t slot);

  // Base selection helpers.
  // Return the currently selected base slot (or -1 if none).
  int8_t currentSlot() const;
  void   setCurrentSlot(int8_t slot) { m_currentSlot = (slot >= 0 && slot < Bases::kCount) ? slot : -1; }
  void   printBaseInfo(uint8_t slot, Stream& s);
  bool captureBaseCalibrationPoint(uint8_t slot, float ml, String& message);
  bool autoCalibrateBase(float incrementMl, uint8_t points, int8_t slot, String& message);
  // Capture a calibration point for the current base slot.
  bool captureCurrentBaseCalibrationPoint(float ml, String& message) {
    const int8_t slot = currentSlot();
    return (slot >= 0) ? captureBaseCalibrationPoint((uint8_t)slot, ml, message) : false;
  }
  bool clearCurrentBaseCalibrationPoints(String& message);
  bool clearToolheadCalibrationPoints(String& message);
  bool setBaseStepsPermL(uint8_t slot, float stepsPermL, String& message);
  bool loadRecipeFromFS(uint32_t recipeId);
  bool saveRecipeToFS(uint32_t recipeId);
  void runRecipe();
  void setBreakpointsEnabled(bool enabled);
  bool breakpointsEnabled() const;
  bool serialBreakpoint(const String &label = "");

  // calibration for toolhead already here...
  bool captureToolheadCalibrationPoint(float ml, String& message);
  bool printCurrentBaseInfo(Stream& s = Serial);
  bool scanToolheadBlocking();
  void printToolheadInfo(Stream& out);
  // Return the current toolhead RFID value.
  uint32_t toolheadRfid() const { return m_toolhead.rfid; }
  bool showVolumes(String& data, String& message);
  bool buildStatusJson(String& data) const;
  bool transferFromBase(uint8_t slot, float ml);



private:
  bool     goToBase(uint8_t slot, Axis::MoveHook hook = nullptr, void* context = nullptr);
  bool     retractToolhead(float ml);
  uint32_t readRFIDNow();
  uint32_t readBaseRFIDBlocking(uint32_t timeoutMs);
  uint32_t readToolheadRFIDBlocking(uint32_t timeoutMS);
  uint16_t readToolheadRawADC();
  uint16_t readBaseRawADC(uint8_t slot);

  Syringe m_toolhead;
  Syringe m_bases[Bases::kCount];
  uint8_t  m_baseToPot[Bases::kCount];
  int8_t  m_currentSlot = -1;
  bool m_breakpointsEnabled = true;
  SyringeCalibration m_calibration;
  Util::RecipeDTO m_recipe;
};

} // namespace App
