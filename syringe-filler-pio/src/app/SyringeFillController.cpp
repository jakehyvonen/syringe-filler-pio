#include "app/SyringeFillController.hpp"

using namespace App;

SyringeFillController::SyringeFillController() {
  for (uint8_t i = 0; i < Bases::kCount; ++i) {
    m_bases[i] = Syringe{};
    m_bases[i].slot = i;
  }
}

void SyringeFillController::scanAllBaseSyringes() {
  for (uint8_t i = 0; i < Bases::kCount; ++i) {
    goToBase(i);

    uint32_t tag = readRFIDNow();
    if (tag == 0) continue;

    m_bases[i].rfid = tag;
    m_bases[i].role = SyringeRole::Base;
    m_bases[i].slot = i;

    Util::BaseMeta meta;
    App::PotCalibration cal;
    if (Util::loadBase(tag, meta, cal)) {
      m_bases[i].cal = cal;
    } else if (Util::loadCalibration(tag, cal)) {
      m_bases[i].cal = cal;
    }

    m_bases[i].currentMl = readBaseVolumeMl(i);
  }
}

void SyringeFillController::scanToolheadSyringe() {
  uint32_t tag = readRFIDNow();
  if (tag == 0) return;

  m_toolhead.rfid = tag;
  m_toolhead.role = SyringeRole::Toolhead;

  App::PotCalibration cal;
  if (Util::loadCalibration(tag, cal)) {
    m_toolhead.cal = cal;
  }

  m_toolhead.currentMl = readToolheadVolumeMl();
  loadToolheadRecipeFromFS();
}

bool SyringeFillController::loadToolheadRecipeFromFS() {
  if (m_toolhead.rfid == 0) return false;
  return Util::loadRecipe(m_toolhead.rfid, m_recipe);
}

bool SyringeFillController::saveToolheadRecipeToFS() {
  if (m_toolhead.rfid == 0) return false;
  return Util::saveRecipe(m_toolhead.rfid, m_recipe);
}

void SyringeFillController::runRecipe() {
  for (uint8_t i = 0; i < m_recipe.count; ++i) {
    auto &step = m_recipe.steps[i];
    transferFromBase(step.baseSlot, step.ml);
  }
  m_toolhead.currentMl = readToolheadVolumeMl();
}

// helpers
bool SyringeFillController::goToBase(uint8_t slot) {
  if (slot >= Bases::kCount) return false;
  long target = Bases::positionSteps(slot);
  if (target >= 0) {
    Axis::moveTo(target);   // your project’s move
  }
  return true;
}

uint32_t SyringeFillController::readRFIDNow() {
  // still a stub until we see your RFID API
  return 0;
}

float SyringeFillController::readToolheadVolumeMl() {
  // still a stub; feed 0 through cal
  return m_toolhead.cal.rawToMl(0);
}

float SyringeFillController::readBaseVolumeMl(uint8_t slot) {
  if (slot >= Bases::kCount) return 0.0f;
  return m_bases[slot].cal.rawToMl(0);
}

bool SyringeFillController::transferFromBase(uint8_t slot, float ml) {
  (void)slot;
  (void)ml;
  return true;
}
