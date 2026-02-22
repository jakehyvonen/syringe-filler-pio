/**
 * @file Toolhead.hpp
 * @brief Toolhead lift (stepper) and coupler (servo) control.
 */
#pragma once
#include <stdint.h>

namespace Toolhead {
  void init();
  bool isReady();
  bool isRaised(); // reads Pins::RAISED

  // Legacy-compatible servo APIs (coupler only).
  void setAngle(uint8_t ch, int angle);
  void setAngleSlow(uint8_t ch, int target, int stepDelay = 23);
  void setPulseRaw(uint8_t ch, int pulse);
  void setSlowRampMs(int delayMs);
  int getSlowRampMs();

  // Toolhead stepper APIs.
  void setSpeedSPS(long sps);
  void moveSteps(long steps);
  long current();
  void setCurrent(long steps);
  bool homeRaised(uint16_t timeout_ms = 11000);

  void raise();
  void couple();
  bool isCoupled();
  bool ensureRaised(uint16_t timeout_ms = 11000);
}
