#pragma once
#include <Arduino.h>
#include "app/SyringeCalibration.hpp"
#include "app/Syringe.hpp"
#include "hw/Bases.hpp"
#include "util/Storage.hpp"   // for loadBase/saveBase etc.

namespace App {

class SyringeFillController {
public:
  SyringeFillController();

  void scanAllBaseSyringes();
  bool scanBaseSyringe(uint8_t slot);

  // NEW
  int8_t currentSlot() const { return m_currentSlot; }
  void   printBaseInfo(uint8_t slot, Stream& s);
  bool captureBaseCalibrationPoint(uint8_t slot, float ml, String& message);
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
  uint32_t toolheadRfid() const { return m_toolhead.rfid; }   // optional but handy
  bool showVolumes(String& data, String& message);



private:
  bool     goToBase(uint8_t slot);
  uint32_t readRFIDNow();
  uint32_t readBaseRFIDBlocking(uint32_t timeoutMs);
  uint32_t readToolheadRFIDBlocking(uint32_t timeoutMS);
  bool     transferFromBase(uint8_t slot, float ml);
  uint16_t readToolheadRawADC();
  uint16_t readBaseRawADC(uint8_t slot);

  Syringe m_toolhead;
  Syringe m_bases[Bases::kCount];
  uint8_t  m_baseToPot[Bases::kCount];
  int8_t  m_currentSlot = -1;
  SyringeCalibration m_calibration;
  Util::RecipeDTO m_recipe;  // or Util::Recipe, whatever you called it
};

} // namespace App
