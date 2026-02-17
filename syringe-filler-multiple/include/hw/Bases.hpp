/**
 * @file Bases.hpp
 * @brief Base selection and position bookkeeping for syringe bases.
 */
#pragma once
#include <Arduino.h>
#include "hw/Pins.hpp"

namespace Bases {

// Number of physical bases (from Pins.hpp).
constexpr uint8_t kCount = Pins::NUM_BASES;

void init();
void disableAll();

// Select a base by 0-based index. Returns false if index is out of range.
bool select(uint8_t idx0);

// Clear base selection (-1 means none selected).
void clearSelection();

// Currently selected base index (0-based), or -1 when none selected.
int8_t selected();

// enable/disable only currently selected base's driver
void hold(bool on);

// Base position accessors (0-based index).
long  getPos(uint8_t idx0);
bool  setPos(uint8_t idx0, long steps);
bool  setHere(uint8_t idx0, long currentSteps);

// Legacy aliases retained for naming consistency at call sites.
inline long positionSteps(uint8_t idx0) { return getPos(idx0); }
inline bool setPositionSteps(uint8_t idx0, long steps) { return setPos(idx0, steps); }

} // namespace Bases
