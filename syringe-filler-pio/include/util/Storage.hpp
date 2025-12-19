#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "app/Syringe.hpp"

namespace Util {

struct BaseMeta {
  char paintName[24];
  BaseMeta() { paintName[0] = '\0'; }
};

struct RecipeStepDTO {
  uint8_t baseSlot;
  float   ml;
};

struct RecipeDTO {
  static constexpr uint8_t kMaxSteps = 16;
  RecipeStepDTO steps[kMaxSteps];
  uint8_t count = 0;
};

struct CalPoint {
  float ml = 0.0f;
  float ratio = 0.0f;
};

constexpr uint8_t kCalPointCount = 2;

bool initStorage();

// calibration + base metadata
bool loadCalibration(uint32_t rfid, App::PotCalibration& out);
bool saveCalibration(uint32_t rfid, const App::PotCalibration& cal);
bool loadBase(uint32_t rfid, BaseMeta& meta, App::PotCalibration& cal);
bool saveBase(uint32_t rfid, const BaseMeta& meta, const App::PotCalibration& cal);

// base axis positions (index = 0-based base slot)
bool loadBasePos(uint8_t idx0, long& steps);
bool saveBasePos(uint8_t idx0, long steps);

// recipes
bool loadRecipe(uint32_t toolheadRfid, RecipeDTO& out);
bool saveRecipe(uint32_t toolheadRfid, const RecipeDTO& in);

} // namespace Util
