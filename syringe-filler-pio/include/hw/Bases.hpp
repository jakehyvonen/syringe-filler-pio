#pragma once
#include <Arduino.h>

namespace Bases {
  void init();
  void disableAll();
  bool select(uint8_t idx1);       // 1..NUM_BASES, 0 = none
  uint8_t selected();               // returns 0 if none selected
  void hold(bool on);               // enable/disable only the selected base
  long  getPos(uint8_t idx1);
  bool  setPos(uint8_t idx1, long steps);
  bool  setHere(uint8_t idx1, long current);
}
