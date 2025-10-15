#pragma once
#include <stdint.h>

namespace Axis {
  void init();
  void setSpeedSPS(long sps);  // affects step interval
  void enable(bool on);
  void dir(bool high);
  void stepBlocking();         // uses Pins::STEP1 timing
  void moveSteps(long steps);  // ensures Toolhead::ensureRaised() internally
  void moveTo(long targetSteps);
  long current();
  void setCurrent(long s);
}
