#pragma once
#include <Arduino.h>
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
  bool   saveCurrentBaseToNVS();   // create/update entry for the base we’re parked at
  bool captureBaseEmpty(uint8_t slot);
  bool captureCurrentBaseEmpty() { 
    return (m_currentSlot >= 0) ? captureBaseEmpty((uint8_t)m_currentSlot) : false;
  }
  bool captureBaseFull(uint8_t slot);
  bool captureCurrentBaseFull() { 
    return (m_currentSlot >= 0) ? captureBaseFull((uint8_t)m_currentSlot) : false;
  }
  bool setCurrentBaseMlFull(float ml);
  bool setToolheadMlFull(float ml);  
  bool loadToolheadRecipeFromFS();
  bool saveToolheadRecipeToFS();
  void runRecipe();

  // calibration for toolhead already here...
  bool captureToolheadEmpty();
  bool captureToolheadFull(float mlFull);
  bool saveToolheadCalibration();
  bool printCurrentBaseInfo(Stream& s = Serial);
  bool scanToolheadBlocking();
  void printToolheadInfo(Stream& out);
  uint32_t toolheadRfid() const { return m_toolheadRfid; }   // optional but handy



private:
  bool     goToBase(uint8_t slot);
  uint32_t readRFIDNow();
  uint32_t readBaseRFIDBlocking(uint32_t timeoutMs);
  uint32_t readToolheadRFIDBlocking(uint32_t timeoutMS);
  float    readToolheadVolumeMl();
  float    readBaseVolumeMl(uint8_t slot);
  bool     transferFromBase(uint8_t slot, float ml);
  uint16_t readToolheadRawADC();
  uint16_t readBaseRawADC(uint8_t slot);

  Syringe m_toolhead;
  Syringe m_bases[Bases::kCount];
  int8_t getBasePotIndex(uint8_t baseSlot) const;
  uint32_t m_toolheadRfid = 0;     // set when toolhead is scanned
  App::PotCalibration m_toolCal;   // loaded from NVS via Util::loadCalibration()
  bool m_toolCalValid = false;

  static float mlFromCounts_(const App::PotCalibration& cal, uint16_t counts);

  uint8_t  m_baseToPot[Bases::kCount];
  int8_t  m_currentSlot = -1;
  Util::RecipeDTO m_recipe;  // or Util::Recipe, whatever you called it
};

} // namespace App
