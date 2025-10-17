#include <Arduino.h>
#include "hw/Bases.hpp"
#include "hw/Pins.hpp"
#include "motion/Axis.hpp"

namespace Bases {

static long basePos[Pins::NUM_BASES] = { 3385, 5480, 3330, 3330, 3330 };

void init() {
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    pinMode(Pins::BASE_EN[i], OUTPUT);
    digitalWrite(Pins::BASE_EN[i], Pins::DISABLE_LEVEL); // default disabled (HIGH)
  }
}

void disableAll() {
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    digitalWrite(Pins::BASE_EN[i], Pins::DISABLE_LEVEL);
  }
}

bool select(uint8_t idx1) {
  if (idx1 == 0) {
    disableAll();
    Serial.println("Base: none");
    return true;
  }
  if (idx1 > Pins::NUM_BASES) {
    Serial.println("ERROR: base index out of range");
    return false;
  }
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    digitalWrite(Pins::BASE_EN[i], (i == (idx1 - 1)) ? Pins::ENABLE_LEVEL : Pins::DISABLE_LEVEL);
  }
  Serial.print("Base selected: "); Serial.println(idx1);
  return true;
}

long getPos(uint8_t idx1) {
  if (idx1 == 0 || idx1 > Pins::NUM_BASES) return -1;
  return basePos[idx1 - 1];
}

bool setPos(uint8_t idx1, long steps) {
  if (idx1 == 0 || idx1 > Pins::NUM_BASES) return false;
  basePos[idx1 - 1] = steps;
  return true;
}

bool setHere(uint8_t idx1, long current) {
  if (idx1 == 0 || idx1 > Pins::NUM_BASES) return false;
  basePos[idx1 - 1] = current;
  return true;
}

} // namespace Bases
