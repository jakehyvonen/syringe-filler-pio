#pragma once
#include <stdint.h>

namespace Pots {
  constexpr uint8_t NUM_POTS = 1;
  void init();
  void poll();
  uint16_t raw(uint8_t i);
  uint16_t filt(uint8_t i);
  float percent(uint8_t i);
  uint16_t readScaled(uint8_t i); // one-shot 0..1023
}
