/**
 * @file Toolhead.hpp
 * @brief Servo control for toolhead lift and coupling mechanism.
 */
#pragma once
#include <stdint.h>

namespace Toolhead {
  void init(); // optional
  bool isRaised(); // reads Pins::RAISED
  void setAngle(uint8_t ch, int angle);
  void setAngleSlow(uint8_t ch, int target, int stepDelay = 23);
  void setPulseRaw(uint8_t ch, int pulse);
  void raise();
  void couple();
  bool ensureRaised(uint16_t timeout_ms = 1200);
}
