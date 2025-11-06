#pragma once
#include <Arduino.h>
#include "hw/Pins.hpp"

namespace Bases {

// how many bases we physically have (comes from your Pins.hpp)
constexpr uint8_t kCount = Pins::NUM_BASES;

void init();
void disableAll();

/**
 * Select a base by 1-based index:
 *   0 = none
 *   1..kCount = valid
 */
bool select(uint8_t idx1);
uint8_t selected();          // 0 if none

// enable/disable only currently selected base's driver
void hold(bool on);

// old 1-based accessors
long  getPos(uint8_t idx1);
bool  setPos(uint8_t idx1, long steps);
bool  setHere(uint8_t idx1, long currentSteps);

// NEW: 0-based helpers for higher-level code
inline long positionSteps(uint8_t idx0) {
  if (idx0 >= kCount) return -1;
  return getPos(idx0 + 1);       // map 0-based -> your old 1-based
}

inline bool setPositionSteps(uint8_t idx0, long steps) {
  if (idx0 >= kCount) return false;
  return setPos(idx0 + 1, steps);
}

} // namespace Bases
