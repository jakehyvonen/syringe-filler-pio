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
  int8_t currentSlot() const { return m_currentSlot; }
  void   printBaseInfo(uint8_t slot, Stream& s);
  bool captureBaseCalibrationPoint(uint8_t slot, float ml, String& message);
  // Capture a calibration point for the current base slot.
  bool captureCurrentBaseCalibrationPoint(float ml, String& message) {
    return (m_currentSlot >= 0) ? captureBaseCalibrationPoint((uint8_t)m_currentSlot, ml, message) : false;
  }
  bool clearCurrentBaseCalibrationPoints(String& message);
  bool clearToolheadCalibrationPoints(String& message);
  bool forceCurrentBaseCalibrationZero(String& message);
  bool forceToolheadCalibrationZero(String& message);
  bool loadToolheadRecipeFromFS();
  bool saveToolheadRecipeToFS();
  void runRecipe();

  // calibration for toolhead already here...
  bool captureToolheadCalibrationPoint(float ml, String& message);
  bool printCurrentBaseInfo(Stream& s = Serial);
  bool scanToolheadBlocking();
  void printToolheadInfo(Stream& out);
  // Return the current toolhead RFID value.
  uint32_t toolheadRfid() const { return m_toolhead.rfid; }
  bool showVolumes(String& data, String& message);
  bool transferFromBase(uint8_t slot, float ml);



private:
  bool     goToBase(uint8_t slot, Axis::MoveHook hook = nullptr, void* context = nullptr);
  uint32_t readRFIDNow();
  uint32_t readBaseRFIDBlocking(uint32_t timeoutMs);
  uint32_t readToolheadRFIDBlocking(uint32_t timeoutMS);
  uint16_t readToolheadRawADC();
  uint16_t readBaseRawADC(uint8_t slot);

  Syringe m_toolhead;
  Syringe m_bases[Bases::kCount];
  uint8_t  m_baseToPot[Bases::kCount];
  int8_t  m_currentSlot = -1;
  SyringeCalibration m_calibration;
  Util::RecipeDTO m_recipe;
};

} // namespace App
