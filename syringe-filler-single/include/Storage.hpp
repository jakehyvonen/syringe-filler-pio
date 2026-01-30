/**
 * @file Storage.hpp
 * @brief LittleFS persistence for base syringe metadata.
 */
#pragma once

#include <Arduino.h>

namespace Storage {

struct BaseInfo {
  char paintName[32];
  char recipeName[32];
  char recipeId[24];
  char notes[96];

  BaseInfo() {
    paintName[0] = '\0';
    recipeName[0] = '\0';
    recipeId[0] = '\0';
    notes[0] = '\0';
  }
};

bool init();
bool loadBase(uint32_t rfid, BaseInfo& out);
bool saveBase(uint32_t rfid, const BaseInfo& info);
bool deleteBase(uint32_t rfid);
bool listBaseIds(uint32_t* out, size_t max, size_t& count);

}  // namespace Storage
