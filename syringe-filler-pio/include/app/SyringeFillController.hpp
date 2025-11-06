#pragma once

#include <Arduino.h>
#include "app/Syringe.hpp"
#include "util/Storage.hpp"
#include "hw/Bases.hpp"
#include "hw/RFID.hpp"
#include "hw/Pots.hpp"
#include "motion/Axis.hpp"
#include "servo/Toolhead.hpp"

namespace App {

class SyringeFillController {
public:
  SyringeFillController();

  void scanAllBaseSyringes();
  void scanToolheadSyringe();
  bool scanBaseSyringe(uint8_t slot);   // <-- add this

  bool loadToolheadRecipeFromFS();
  bool saveToolheadRecipeToFS();

  void setRecipe(const Util::RecipeDTO& r) { m_recipe = r; }
  void runRecipe();

   // calibration helpers for TOOLHEAD
  bool captureToolheadEmpty();
  bool captureToolheadFull(float mlFull);
  bool saveToolheadCalibration();

  // (optional) expose current toolhead RFID for debugging/printing
  uint32_t toolheadRfid() const { return m_toolhead.rfid; }

private:
  App::Syringe     m_toolhead;
  App::Syringe     m_bases[Bases::kCount];   // <-- now real count
  Util::RecipeDTO  m_recipe;

  bool     goToBase(uint8_t slot);
  uint32_t readRFIDNow();
  float    readToolheadVolumeMl();
  float    readBaseVolumeMl(uint8_t slot);
  bool     transferFromBase(uint8_t slot, float ml);
  uint16_t readToolheadRawADC();   // new helper

};

} // namespace App
