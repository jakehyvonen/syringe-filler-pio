/**
 * @file Bases.cpp
 * @brief Base selection and persisted base position handling.
 */
#include <Arduino.h>
#include "hw/Bases.hpp"
#include "hw/Pins.hpp"
#include "hw/Drivers.hpp"
#include "motion/Axis.hpp"
#include "util/Storage.hpp"

namespace Bases {

// default positions if nothing in NVS
static long basePos[Pins::NUM_BASES] = {
  600,
  5300,
  10010,
  14700,
  19400
};

// -1 = none selected; else 0..NUM_BASES-1
static int8_t s_selected = -1;

// Cached logical states for MCP pins (used for debug and fallback).
static uint8_t s_baseEnableState[Pins::NUM_BASES] = {
  Pins::DISABLE_LEVEL,
  Pins::DISABLE_LEVEL,
  Pins::DISABLE_LEVEL,
  Pins::DISABLE_LEVEL,
  Pins::DISABLE_LEVEL
};

// Return the MCP pin for the currently selected base.
static inline uint8_t selectedEnPin() {
  if (s_selected < 0 || s_selected >= Pins::NUM_BASES) return 255;
  return Pins::BASE_EN_MCP[static_cast<uint8_t>(s_selected)];
}

static inline void setEnablePinMode(uint8_t pin, uint8_t mode) {
  if (Drivers::hasBaseEnableExpander()) {
    Drivers::BASE_EN_EXPANDER.pinMode(pin, mode);
  }
}

static inline void writeEnablePin(uint8_t pin, uint8_t level) {
  if (Drivers::hasBaseEnableExpander()) {
    Drivers::BASE_EN_EXPANDER.digitalWrite(pin, level);
  }

  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    if (Pins::BASE_EN_MCP[i] == pin) {
      s_baseEnableState[i] = level;
      break;
    }
  }
}

static inline int readEnablePin(uint8_t pin) {
  if (Drivers::hasBaseEnableExpander()) {
    return Drivers::BASE_EN_EXPANDER.digitalRead(pin);
  }

  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    if (Pins::BASE_EN_MCP[i] == pin) {
      return s_baseEnableState[i];
    }
  }
  return Pins::DISABLE_LEVEL;
}

// Print current base enable pin states to serial.
static void debugDumpEN(const char* label) {
  Serial.print("[Bases] "); Serial.print(label);
  Serial.print("  sel0=");  Serial.print(s_selected);
  Serial.print("  EN MCP pins: ");
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    const uint8_t pin = Pins::BASE_EN_MCP[i];
    int lvl = readEnablePin(pin);
    Serial.print(pin);
    Serial.print('=');
    Serial.print(lvl);
    if (i + 1 < Pins::NUM_BASES) Serial.print(' ');
  }
  Serial.println();
}

// Initialize base enable pins and load saved positions.
void init() {
  // MCP base-enable pins (all disabled at boot)
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    const uint8_t pin = Pins::BASE_EN_MCP[i];
    setEnablePinMode(pin, OUTPUT);
    writeEnablePin(pin, Pins::DISABLE_LEVEL);
  }
  s_selected = -1;

  // try to load positions from NVS; if found, overwrite defaults
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    long loaded;
    if (Util::loadBasePos(i, loaded)) {
      basePos[i] = loaded;
    }
  }

  debugDumpEN("init");
}

// Disable all base enable outputs.
void disableAll() {
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    const uint8_t pin = Pins::BASE_EN_MCP[i];
    setEnablePinMode(pin, OUTPUT);
    writeEnablePin(pin, Pins::DISABLE_LEVEL);
  }
}

// Return the currently selected base (0-based, -1 for none).
int8_t selected() {
  return s_selected;
}

// Clear base selection and disable all base drivers.
void clearSelection() {
  disableAll();
  s_selected = -1;
  Serial.println("[Bases] Base: none");
  debugDumpEN("after clearSelection");
}

// Select a base by 0-based index.
bool select(uint8_t idx0) {
  if (idx0 >= Pins::NUM_BASES) {
    Serial.println("[Bases] ERROR: base index out of range");
    return false;
  }

  s_selected = static_cast<int8_t>(idx0);
  disableAll();

  // Base enable outputs are active-low. Selecting a base should immediately
  // assert its MCP pin LOW so exactly one base driver is enabled.
  const uint8_t enPin = selectedEnPin();
  if (enPin != 255) {
    setEnablePinMode(enPin, OUTPUT);
    writeEnablePin(enPin, Pins::ENABLE_LEVEL);
  }

  Serial.print("[Bases] Base selected: "); Serial.println(static_cast<int>(idx0));
  debugDumpEN("after select");
  return true;
}

// Enable or disable the currently selected base.
void hold(bool on) {
  if (s_selected < 0) {
    Serial.println("[Bases] hold() called with no selection; ignoring");
    return;
  }

  const uint8_t enPin = selectedEnPin();
  if (enPin == 255) return;

  writeEnablePin(enPin, on ? Pins::ENABLE_LEVEL : Pins::DISABLE_LEVEL);

  Serial.print("[Bases] hold(");
  Serial.print(on ? "ON" : "OFF");
  Serial.print(") -> sel0="); Serial.print(s_selected);
  Serial.print(" enPin="); Serial.println(enPin);

  debugDumpEN("after hold");
}

// Get the stored position for a base index.
long getPos(uint8_t idx0) {
  if (idx0 >= Pins::NUM_BASES) return -1;
  return basePos[idx0];
}

// Set and persist the stored position for a base index.
bool setPos(uint8_t idx0, long steps) {
  if (idx0 >= Pins::NUM_BASES) return false;
  basePos[idx0] = steps;
  // persist new value
  Util::saveBasePos(idx0, steps);
  return true;
}

// Store the current axis position as the base position.
bool setHere(uint8_t idx0, long current) {
  if (idx0 >= Pins::NUM_BASES) return false;
  basePos[idx0] = current;
  Util::saveBasePos(idx0, current);
  return true;
}

} // namespace Bases
