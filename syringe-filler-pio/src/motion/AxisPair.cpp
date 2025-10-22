#include "motion/AxisPair.hpp"
#include "hw/Drivers.hpp"
#include "hw/Pots.hpp"
#include "hw/Pins.hpp"
#include "hw/Bases.hpp"
#include <Arduino.h>

namespace AxisPair {

// Position counters for each logical axis
static volatile long s_pos2 = 0;  // toolhead syringe (uses STEP2/DIR2/EN2)
static volatile long s_pos3 = 0;  // selected base syringe (uses STEP3/DIR3 and Bases::hold())

// Speed state
static long          s_speedSPS    = 800;
static unsigned long s_interval_us = 1000000UL / (2UL * 800UL);

/**
 * Step a given STEP pin at a bounded rate.
 * Keeps separate cadence for STEP2 vs STEP3 via static state.
 */
static inline void stepOnceTimedOnPin(uint8_t pin, unsigned long interval_us) {
  static unsigned long last2 = 0, last3 = 0;
  static bool state2 = false, state3 = false;

  unsigned long &last  = (pin == Pins::STEP2) ? last2 : last3;
  bool         &state = (pin == Pins::STEP2) ? state2 : state3;

  unsigned long now;
  do { now = micros(); } while ((unsigned long)(now - last) < interval_us);
  last = now;

  state = !state;
  digitalWrite(pin, state ? HIGH : LOW);
}

void init() {
  // TOOLHEAD (Axis 2)
  pinMode(Pins::STEP2, OUTPUT);
  pinMode(Pins::DIR2,  OUTPUT);
  pinMode(Pins::EN2,   OUTPUT);
  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);

  // BASES (Axis 3) share STEP3/DIR3; enable is handled by Bases::hold()
  pinMode(Pins::STEP3, OUTPUT);
  pinMode(Pins::DIR3,  OUTPUT);

  Serial.println("[AxisPair] init done (EN2 disabled, bases disabled via Bases.init())");
}

void setSpeedSPS(long sps) {
  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;
  s_speedSPS    = sps;
  s_interval_us = (unsigned long)(1000000.0 / (2.0 * (double)sps));
  Serial.print("[AxisPair] speed set to "); Serial.print(s_speedSPS);
  Serial.print(" sps (interval=");  Serial.print(s_interval_us);
  Serial.println(" us)");
}

/** ============================
 *  AXIS 2: TOOLHEAD SYRINGE
 *  Uses STEP2 / DIR2 / EN2
 *  ============================ */
void move2(long steps) {
  if (steps == 0) return;

  const bool dirHigh = (steps > 0);
  long todo = labs(steps);

  Serial.print("[AxisPair] move2 steps="); Serial.println(steps);

  // Enable ONLY the toolhead driver (EN2). Bases remain disabled.
  digitalWrite(Pins::EN2, Pins::ENABLE_LEVEL);
  digitalWrite(Pins::DIR2, dirHigh ? HIGH : LOW);

  for (long i = 0; i < todo; ++i) {
    stepOnceTimedOnPin(Pins::STEP2, s_interval_us);
    s_pos2 += dirHigh ? +1 : -1;
    stepOnceTimedOnPin(Pins::STEP2, s_interval_us);
  }

  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
}

/** =================================
 *  AXIS 3: SELECTED BASE SYRINGE ONLY
 *  Uses STEP3 / DIR3; enable via Bases
 *  ================================= */
void move3(long steps) {
  if (steps == 0) return;

  const bool dirHigh = (steps > 0);
  long todo = labs(steps);

  Serial.print("[AxisPair] move3 steps="); Serial.println(steps);

  // Enable ONLY the currently selected base via Bases::hold(true)
  Bases::hold(true);
  digitalWrite(Pins::DIR3, dirHigh ? HIGH : LOW);

  for (long i = 0; i < todo; ++i) {
    stepOnceTimedOnPin(Pins::STEP3, s_interval_us);
    s_pos3 += dirHigh ? +1 : -1;
    stepOnceTimedOnPin(Pins::STEP3, s_interval_us);
  }

  Bases::hold(false);
}

/** ==============================================================
 *  SYNCHRONOUS MOVE: TOOLHEAD (AXIS 2) + SELECTED BASE (AXIS 3)
 *  Enables BOTH: EN2 for toolhead AND Bases::hold(true) for base.
 *  ============================================================== */
void moveSync(long steps2, long steps3) {
  const long a = labs(steps2), b = labs(steps3);
  if (a == 0 && b == 0) return;

  const bool dir2High = (steps2 >= 0);
  const bool dir3High = (steps3 >= 0);

  Serial.print("[AxisPair] moveSync s2="); Serial.print(steps2);
  Serial.print(" s3="); Serial.println(steps3);

  // Enable both participants
  digitalWrite(Pins::EN2, Pins::ENABLE_LEVEL);  // toolhead
  Bases::hold(true);                            // selected base

  digitalWrite(Pins::DIR2, dir2High ? HIGH : LOW);
  digitalWrite(Pins::DIR3, dir3High ? HIGH : LOW);

  const long n = (a > b) ? a : b;
  long acc2 = 0, acc3 = 0;

  unsigned long last = micros();
  for (long i = 0; i < n; ++i) {
    unsigned long now;
    do { now = micros(); } while ((unsigned long)(now - last) < s_interval_us);
    last = now;

    bool do2 = false, do3 = false;
    acc2 += a; if (acc2 >= n) { acc2 -= n; do2 = true; }
    acc3 += b; if (acc3 >= n) { acc3 -= n; do3 = true; }

    if (do2) digitalWrite(Pins::STEP2, HIGH);
    if (do3) digitalWrite(Pins::STEP3, HIGH);
    delayMicroseconds(Pins::STEP_PULSE_US);
    if (do2) { digitalWrite(Pins::STEP2, LOW); s_pos2 += dir2High ? +1 : -1; }
    if (do3) { digitalWrite(Pins::STEP3, LOW); s_pos3 += dir3High ? +1 : -1; }
  }

  // Disable both
  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
  Bases::hold(false);
}

void link(long steps) {
  // Positive steps push toolhead forward while pulling base back (or vice versa)
  moveSync(steps, -steps);
}

long pos2() { return s_pos2; }
long pos3() { return s_pos3; }

/**
 * AXIS 2 closed-loop: move toolhead until pot[0] reaches target
 */
bool move2UntilPotSimple(uint16_t target_adc, long sps) {
  if (!Drivers::hasADS()) {
    Serial.println("[AxisPair] move2UntilPot: ADS not present.");
    return false;
  }

  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;

  const unsigned long interval_us = (unsigned long)(1000000.0 / (2.0 * (double)sps));
  const uint8_t hysteresis = 3;
  const unsigned long timeout_ms = 20000UL;

  Pots::init();   // safe to call repeatedly

  uint16_t pot = Pots::readScaled(0);
  bool dirHigh = (target_adc > pot);

  // Enable toolhead only (we're moving Axis 2)
  digitalWrite(Pins::EN2, Pins::ENABLE_LEVEL);
  digitalWrite(Pins::DIR2, dirHigh ? HIGH : LOW);

  unsigned long last = micros();
  unsigned long startMs = millis();

  while (true) {
    if (timeout_ms && (millis() - startMs) > timeout_ms) {
      digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
      Serial.println("[AxisPair] move2UntilPot: TIMEOUT");
      return false;
    }

    unsigned long now;
    do { now = micros(); } while ((unsigned long)(now - last) < interval_us);
    last = now;

    digitalWrite(Pins::STEP2, HIGH);
    delayMicroseconds(Pins::STEP_PULSE_US);
    digitalWrite(Pins::STEP2, LOW);

    s_pos2 += dirHigh ? +1 : -1;

    pot = Pots::readScaled(0);
    Serial.print("[AxisPair] pot[0]="); Serial.println(pot);

    if (dirHigh) {
      if (pot >= (uint16_t)(target_adc - hysteresis)) break;
    } else {
      if (pot <= (uint16_t)(target_adc + hysteresis)) break;
    }
  }

  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
  Serial.print("[AxisPair] final pot[0]="); Serial.println(pot);
  return true;
}

} // namespace AxisPair
