/**
 * @file Toolhead.hpp
 * @brief Toolhead lift and coupling control using stepper axis 4.
 */
#pragma once
#include <stdint.h>

namespace Toolhead {
  void init();
  bool isReady();
  bool isRaised(); // reads Pins::RAISED

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
