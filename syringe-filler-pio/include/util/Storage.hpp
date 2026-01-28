/**
 * @file Storage.hpp
 * @brief NVS/LittleFS persistence for calibration, bases, and recipes.
 */
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "app/Syringe.hpp"
#include "util/Recipe.hpp"

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

constexpr uint8_t kCalPointCount = App::PotCalibration::kMaxPoints;

bool initStorage();

// calibration + base metadata
bool loadCalibration(uint32_t rfid, App::PotCalibration& out);
bool saveCalibration(uint32_t rfid, const App::PotCalibration& cal);
bool loadBase(uint32_t rfid, BaseMeta& meta, App::PotCalibration& cal, App::CalibrationPoints& points);
bool saveBase(uint32_t rfid, const BaseMeta& meta, const App::PotCalibration& cal, const App::CalibrationPoints& points);

// base axis positions (index = 0-based base slot)
bool loadBasePos(uint8_t idx0, long& steps);
bool saveBasePos(uint8_t idx0, long steps);

// recipes
bool loadRecipe(uint32_t toolheadRfid, RecipeDTO& out);
bool saveRecipe(uint32_t toolheadRfid, const RecipeDTO& in);
bool loadRecipe(uint32_t toolheadRfid, Util::Recipe& recipe);
bool saveRecipe(uint32_t toolheadRfid, const Util::Recipe& recipe);
bool deleteRecipe(uint32_t toolheadRfid);
bool listRecipeRfids(uint32_t* out, size_t max, size_t& count);
bool listRecipes(String& outJson);
bool readRecipeJson(uint32_t toolheadRfid, String& outJson);
bool deleteRecipe(uint32_t toolheadRfid);

} // namespace Util
