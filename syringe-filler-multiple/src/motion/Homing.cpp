/**
 * @file Homing.cpp
 * @brief Homing routine for the primary axis and encoder alignment.
 */
#include "motion/Homing.hpp"
#include "motion/Axis.hpp"
#include "servo/Toolhead.hpp"
#include "hw/Pins.hpp"
#include "hw/Encoder.hpp"
#include <Arduino.h>

namespace Homing {

// Local helper: time-gated toggle for Axis1, using Axis' current interval
static unsigned long s_fastIntervalUs = 1000; // default; will be derived from Axis speed
// Toggle the step pin at the configured fast interval.
static inline void stepOnceTimed() {
  static bool state = false;
  static unsigned long last = micros();

  unsigned long now;
  do { now = micros(); } while ((unsigned long)(now - last) < s_fastIntervalUs);
  last = now;

  state = !state;
  digitalWrite(Pins::STEP1, state ? HIGH : LOW);
}

// Conditionally emit encoder readings while homing when debug polling is enabled.
static inline void maybePrintEncoderReading(const char *phase, unsigned long &lastPrintMs) {
  if (!EncoderHW::polling()) return;

  const unsigned long now = millis();
  if ((now - lastPrintMs) < 100) return;  // throttle output during step loops
  lastPrintMs = now;

  Serial.print("[HOMING ENC] phase=");
  Serial.print(phase);
  Serial.print(" count=");
  Serial.print(EncoderHW::count());
  Serial.print(" mm=");
  Serial.println(EncoderHW::mm(), 3);
}

// Home the primary axis and zero the encoder.
void home() {
  if (!Toolhead::ensureRaised()) {
    Serial.println("HOMING ABORTED: toolhead not raised.");
    return;
  }

  s_fastIntervalUs = 1000; // ~500 sps full-steps (two toggles per step)
  unsigned long lastEncoderPrintMs = 0;

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
    maybePrintEncoderReading("fast_approach", lastEncoderPrintMs);
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
      maybePrintEncoderReading("backoff", lastEncoderPrintMs);
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
    maybePrintEncoderReading("slow_approach", lastEncoderPrintMs);
    if ((millis() - start) > timeoutMs) {
      Serial.println("HOMING ERROR: timeout (slow approach).");
      s_fastIntervalUs = saved;
      Axis::enable(false);
      return;
    }
  }

  // Stepper logical zero at the switch
  Axis::setCurrent(0);

  // Release a few steps away from the switch
  {
    const int releaseSteps = 5;
    Axis::dir(!Pins::HOME_DIR_HIGH);
    for (int i = 0; i < releaseSteps; ++i) {
      stepOnceTimed();
      Axis::setCurrent(Axis::current() + (Pins::HOME_DIR_HIGH ? -1 : +1));
      maybePrintEncoderReading("release", lastEncoderPrintMs);
    }
  }

  // ---- encoder zero at the final home position ----
  EncoderHW::zeroHere();

  // ---- OPTIONAL index search that returns to 0 ----
  {
    const int maxIndexSearchSteps = 5000;  // tune for the mechanics
    bool foundIndex = false;
    long stepsMoved = 0;

    // remember where we are (should be 0)
    long homePosSteps = Axis::current();

    // move away from home to look for index
    Axis::dir(!Pins::HOME_DIR_HIGH);
    for (int i = 0; i < maxIndexSearchSteps; ++i) {
      stepOnceTimed();
      stepsMoved++;
      // keep stepper position in sync while searching
      Axis::setCurrent(Axis::current() + (Pins::HOME_DIR_HIGH ? -1 : +1));
      maybePrintEncoderReading("index_search", lastEncoderPrintMs);

      if (digitalRead(Pins::ENC_Z) == HIGH) {
        // index found right here → make THIS the encoder zero
        EncoderHW::zeroHere();
        foundIndex = true;
        Serial.print("HOMING: encoder index captured at search step ");
        Serial.println(i);
        break;
      }
    }

    // go back to the home position we saved so we always end at 0
    Axis::dir(Pins::HOME_DIR_HIGH); // reverse direction
    for (long i = 0; i < stepsMoved; ++i) {
      stepOnceTimed();
      Axis::setCurrent(Axis::current() + (Pins::HOME_DIR_HIGH ? +1 : -1));
      maybePrintEncoderReading("return_to_home", lastEncoderPrintMs);
    }
    // ensure exact logical 0
    Axis::setCurrent(homePosSteps);

    if (!foundIndex) {
      Serial.println("HOMING: encoder index not seen in search window; using limit-based zero.");
    }
  }

  s_fastIntervalUs = saved;
  Serial.println("HOMING COMPLETE. pos=0 (stepper + encoder)");

  Axis::enable(false);
}

} // namespace Homing
