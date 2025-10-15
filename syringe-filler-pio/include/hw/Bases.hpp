#pragma once
#include <stdint.h>

namespace Bases {
  void init();
  void disableAll();
  bool select(uint8_t idx1); // 0..NUM_BASES, 0=none
  long  getPos(uint8_t idx1);
  bool  setPos(uint8_t idx1, long steps);
  bool  setHere(uint8_t idx1, long current);
}
