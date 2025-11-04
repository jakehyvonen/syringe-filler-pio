#include "motion/Homing.hpp"
#include "motion/Axis.hpp"
#include "servo/Toolhead.hpp"
#include "hw/Pins.hpp"
#include "hw/Encoder.hpp"    // <-- NEW
#include <Arduino.h>

namespace Homing {

// Local helper: time-gated toggle for Axis1, using Axis' current interval
static unsigned long s_fastIntervalUs = 1000; // default; will be derived from Axis speed
static inline void stepOnceTimed() {
  static bool state = false;
  static unsigned long last = micros();

  unsigned long now;
  do { now = micros(); } while ((unsigned long)(now - last) < s_fastIntervalUs);
  last = now;

  state = !state;
  digitalWrite(Pins::STEP12, state ? HIGH : LOW);
}

void home() {
  if (!Toolhead::ensureRaised()) {
    Serial.println("HOMING ABORTED: toolhead not raised.");
    return;
  }

  // Use Axis speed as baseline for timing if you expose it; otherwise default.
  // Here we keep a conservative default.
  s_fastIntervalUs = 1000; // ~500 sps full-steps (two toggles per step)

  Axis::enable(true);

  // If already on switch, back off first
  if (digitalRead(Pins::LIMIT) == LOW) {
    Axis::dir(!Pins::HOME_DIR_HIGH);
    for (int i = 0; i < 800; ++i) stepOnceTimed();
    Axis::setCurrent(Axis::current() - 800 * (Pins::HOME_DIR_HIGH ? +1 : -1));
  }

  // Fast approach
  Axis::dir(Pins::HOME_DIR_HIGH);
  unsigned long start = millis();
  const unsigned long timeoutMs = 15000;
  while (digitalRead(Pins::LIMIT) == HIGH) {
    stepOnceTimed();
    Axis::setCurrent(Axis::current() + (Pins::HOME_DIR_HIGH ? +1 : -1));
    if ((millis() - start) > timeoutMs) {
      Serial.println("HOMING ERROR: timeout (fast approach).");
      Axis::enable(false);
      return;
    }
  }
  Serial.println("Switch hit (fast).");

  // Back off
  {
    const int backOff = 600;
    Axis::dir(!Pins::HOME_DIR_HIGH);
    for (int i = 0; i < backOff; ++i) {
      stepOnceTimed();
      Axis::setCurrent(Axis::current() + (Pins::HOME_DIR_HIGH ? -1 : +1));
    }
  }
  delay(10);

  // Slow approach (3x slower)
  unsigned long saved = s_fastIntervalUs;
  s_fastIntervalUs = saved * 3;

  Axis::dir(Pins::HOME_DIR_HIGH);
  start = millis();
  while (digitalRead(Pins::LIMIT) == HIGH) {
    stepOnceTimed();
    Axis::setCurrent(Axis::current() + (Pins::HOME_DIR_HIGH ? +1 : -1));
    if ((millis() - start) > timeoutMs) {
      Serial.println("HOMING ERROR: timeout (slow approach).");
      s_fastIntervalUs = saved;
      Axis::enable(false);
      return;
    }
  }

  // Zero position (stepper logical)
  Axis::setCurrent(0);

  // Release a few steps
  {
    const int releaseSteps = 5;
    Axis::dir(!Pins::HOME_DIR_HIGH);
    for (int i = 0; i < releaseSteps; ++i) {
      stepOnceTimed();
      Axis::setCurrent(Axis::current() + (Pins::HOME_DIR_HIGH ? -1 : +1));
    }
  }

  // ---- NEW: also zero the encoder here ----
  EncoderHW::reset();

  // ---- OPTIONAL: try to find the encoder index just after homing ----
  // This assumes Pins::ENC_Z is wired through the level shifter and is not floating.
  // We move away from the switch (same dir as the little release above) for a short range
  // and if we see the index high, we re-zero the encoder right there.
  {
    const int maxIndexSearchSteps = 2000;  // adjust to your mechanics
    bool foundIndex = false;

    // move in the "away from home" direction
    Axis::dir(!Pins::HOME_DIR_HIGH);
    for (int i = 0; i < maxIndexSearchSteps; ++i) {
      stepOnceTimed();

      // poll index line
      if (digitalRead(Pins::ENC_Z) == HIGH) {
        EncoderHW::reset();
        foundIndex = true;
        Serial.print("HOMING: encoder index captured at step ");
        Serial.println(i);
        break;
      }
    }

    if (!foundIndex) {
      Serial.println("HOMING: encoder index not seen in search window; using limit-based zero.");
    }
  }

  s_fastIntervalUs = saved;
  Serial.println("HOMING COMPLETE. pos=0 (stepper + encoder)");

  Axis::enable(false);
}

} // namespace Homing
