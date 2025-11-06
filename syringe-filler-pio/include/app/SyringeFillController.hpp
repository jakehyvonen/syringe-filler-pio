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

  bool loadToolheadRecipeFromFS();
  bool saveToolheadRecipeToFS();

  void setRecipe(const Util::RecipeDTO& r) { m_recipe = r; }
  void runRecipe();

private:
  App::Syringe     m_toolhead;
  App::Syringe     m_bases[Bases::kCount];   // <-- now real count
  Util::RecipeDTO  m_recipe;

  bool     goToBase(uint8_t slot);
  uint32_t readRFIDNow();
  float    readToolheadVolumeMl();
  float    readBaseVolumeMl(uint8_t slot);
  bool     transferFromBase(uint8_t slot, float ml);
};

} // namespace App
