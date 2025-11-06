#include <Arduino.h>
#include "hw/Bases.hpp"
#include "hw/Pins.hpp"
#include "motion/Axis.hpp"
#include "util/Storage.hpp"     // <-- NEW

namespace Bases {

// default positions if nothing in NVS
static long basePos[Pins::NUM_BASES] = {
  600,
  5300,
  10010,
  14700,
  19400
};

// 0 = none selected; else 1..NUM_BASES
static uint8_t s_selected = 0;

static inline uint8_t selectedEnPin() {
  if (s_selected == 0) return 255;
  return Pins::BASE_EN[s_selected - 1];
}

static void debugDumpEN(const char* label) {
  Serial.print("[Bases] "); Serial.print(label);
  Serial.print("  sel=");  Serial.print(s_selected);
  Serial.print("  EN pins: ");
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    int lvl = digitalRead(Pins::BASE_EN[i]);
    Serial.print(Pins::BASE_EN[i]);
    Serial.print('=');
    Serial.print(lvl);
    if (i + 1 < Pins::NUM_BASES) Serial.print(' ');
  }
  Serial.println();
}

void init() {
  // pins
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    pinMode(Pins::BASE_EN[i], OUTPUT);
    digitalWrite(Pins::BASE_EN[i], Pins::DISABLE_LEVEL);
  }
  s_selected = 0;

  // try to load positions from NVS; if found, overwrite defaults
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    long loaded;
    if (Util::loadBasePos(i, loaded)) {
      basePos[i] = loaded;
    }
  }

  debugDumpEN("init");
}

void disableAll() {
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    digitalWrite(Pins::BASE_EN[i], Pins::DISABLE_LEVEL);
  }
}

uint8_t selected() {
  return s_selected;
}

bool select(uint8_t idx1) {
  // idx1: 0 = none, 1..NUM_BASES valid
  if (idx1 == 0) {
    disableAll();
    s_selected = 0;
    Serial.println("[Bases] Base: none");
    debugDumpEN("after select(0)");
    return true;
  }

  if (idx1 > Pins::NUM_BASES) {
    Serial.println("[Bases] ERROR: base index out of range");
    return false;
  }

  s_selected = idx1;
  disableAll();

  Serial.print("[Bases] Base selected: "); Serial.println(idx1);
  debugDumpEN("after select(n)");
  return true;
}

void hold(bool on) {
  if (s_selected == 0) {
    Serial.println("[Bases] hold() called with no selection; ignoring");
    return;
  }

  const uint8_t enPin = selectedEnPin();
  if (enPin == 255) return;

  digitalWrite(enPin, on ? Pins::ENABLE_LEVEL : Pins::DISABLE_LEVEL);

  Serial.print("[Bases] hold(");
  Serial.print(on ? "ON" : "OFF");
  Serial.print(") -> sel="); Serial.print(s_selected);
  Serial.print(" enPin="); Serial.println(enPin);

  debugDumpEN("after hold");
}

long getPos(uint8_t idx1) {
  if (idx1 == 0 || idx1 > Pins::NUM_BASES) return -1;
  return basePos[idx1 - 1];
}

bool setPos(uint8_t idx1, long steps) {
  if (idx1 == 0 || idx1 > Pins::NUM_BASES) return false;
  basePos[idx1 - 1] = steps;
  // persist new value
  Util::saveBasePos(idx1 - 1, steps);
  return true;
}

bool setHere(uint8_t idx1, long current) {
  if (idx1 == 0 || idx1 > Pins::NUM_BASES) return false;
  basePos[idx1 - 1] = current;
  Util::saveBasePos(idx1 - 1, current);
  return true;
}

} // namespace Bases
