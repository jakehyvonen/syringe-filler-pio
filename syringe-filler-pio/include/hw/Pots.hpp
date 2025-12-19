#pragma once
#include <stdint.h>

namespace Pots {
  // With VREF on ADS#1 A0 we have only 7 single-ended channels remaining.
  constexpr uint8_t NUM_POTS = 7;

  void init();
  void poll();

  // ADS native counts (0..32767 at GAIN_ONE)
  uint16_t raw(uint8_t i);
  uint16_t filt(uint8_t i);

  // 0..100% (ratiometric vs live VREF on ADS0:A0)
  float percent(uint8_t i);
  float ratioFromCounts(uint16_t potCounts);
  uint16_t countsFromRatio(float ratio);

  // One-shot ADS native counts
  uint16_t readCounts(uint8_t i);

  // ---- Legacy (for AxisPair, etc.) -----------------------------------------
  // DEPRECATED: returns ratiometric 0..1023 for backward compatibility.
  // Internally uses the same ratio as `percent()`, so it’s stable even if 3.3 V moves.
  uint16_t readScaled(uint8_t i);
}
