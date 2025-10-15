#include "motion/Axis.hpp"
#include "servo/Toolhead.hpp"
#include "hw/Pins.hpp"
#include <Arduino.h>

namespace Axis {

static volatile long s_currentSteps = 0;
static unsigned long s_stepIntervalUs = 1000;     // µs between toggles
static long          s_sps            = 500;      // nominal
static bool          s_enabled        = false;

static inline void stepOnceTimed() {
  static bool state = false;
  static unsigned long last = micros();

  unsigned long now;
  do { now = micros(); } while ((unsigned long)(now - last) < s_stepIntervalUs);
  last = now;

  state = !state;
  digitalWrite(Pins::STEP1, state ? HIGH : LOW);
}

void init() {
  pinMode(Pins::STEP1, OUTPUT);
  pinMode(Pins::DIR1,  OUTPUT);
  pinMode(Pins::EN1,   OUTPUT);
  digitalWrite(Pins::EN1, Pins::DISABLE_LEVEL);
  digitalWrite(Pins::DIR1, HIGH);
}

void setSpeedSPS(long sps) {
  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;
  s_sps = sps;
  // Each "full" step toggles high+low. We gate on half-period -> divide by 2.
  s_stepIntervalUs = (unsigned long)(1000000.0 / (2.0 * (double)sps));
  Serial.print("Axis1 speed set "); Serial.print(sps);
  Serial.print(" sps (interval=");  Serial.print(s_stepIntervalUs);
  Serial.println(" us)");
}

void enable(bool on) {
  s_enabled = on;
  digitalWrite(Pins::EN1, on ? Pins::ENABLE_LEVEL : Pins::DISABLE_LEVEL);
}

void dir(bool high) {
  digitalWrite(Pins::DIR1, high ? HIGH : LOW);
}

void stepBlocking() {
  // One full pulse
  digitalWrite(Pins::STEP1, HIGH);
  delayMicroseconds(Pins::STEP_PULSE_US);
  digitalWrite(Pins::STEP1, LOW);
}

void moveSteps(long steps) {
  if (steps == 0) return;
  if (!Toolhead::ensureRaised()) {
    Serial.println("Axis1: movement blocked (toolhead not raised).");
    return;
  }

  bool dirHigh = (steps > 0);
  long todo = labs(steps);

  enable(true);
  dir(dirHigh);

  for (long i = 0; i < todo; ++i) {
    stepOnceTimed();
    s_currentSteps += dirHigh ? +1 : -1;
    stepOnceTimed();
  }

  enable(false);
}

void moveTo(long targetSteps) {
  long tgt = targetSteps;
  // Soft limits
  if (tgt < Pins::MIN_POS_STEPS) tgt = Pins::MIN_POS_STEPS;
  if (tgt > Pins::MAX_POS_STEPS) tgt = Pins::MAX_POS_STEPS;

  long delta = tgt - s_currentSteps;
  moveSteps(delta);
}

long current() { return s_currentSteps; }
void setCurrent(long s) { s_currentSteps = s; }

} // namespace Axis
