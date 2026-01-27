/**
 * @file Axis.hpp
 * @brief Primary gantry axis control (timer-based stepping).
 */
#pragma once
#include <stdint.h>

namespace Axis {
  using MoveHook = void (*)(long errSteps, void* context);

  void init();
  void setSpeedSPS(long sps);  // affects step interval
  void enable(bool on);
  void dir(bool high);
  void stepBlocking();         // uses Pins::STEP1 timing
  void moveSteps(long steps);  // ensures Toolhead::ensureRaised() internally
  void moveTo(long targetSteps);
  void moveToWithHook(long targetSteps, MoveHook hook, void* context);
  long current();
  void setCurrent(long s);
}
