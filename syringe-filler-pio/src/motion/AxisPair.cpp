#include "motion/AxisPair.hpp"
#include "hw/Drivers.hpp"
#include "hw/Pots.hpp"
#include "hw/Pins.hpp"
#include <Arduino.h>

namespace AxisPair {

static volatile long s_pos2 = 0;
static volatile long s_pos3 = 0;

static long          s_speedSPS     = 800;
static unsigned long s_interval_us  = 1000000UL / (2UL * 800UL);

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
  pinMode(Pins::STEP2, OUTPUT);
  pinMode(Pins::DIR2,  OUTPUT);
  pinMode(Pins::EN2,   OUTPUT);

  pinMode(Pins::STEP3, OUTPUT);
  pinMode(Pins::DIR3,  OUTPUT);
  pinMode(Pins::EN3,   OUTPUT);

  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
  digitalWrite(Pins::EN3, Pins::DISABLE_LEVEL);
}

void setSpeedSPS(long sps) {
  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;
  s_speedSPS    = sps;
  s_interval_us = (unsigned long)(1000000.0 / (2.0 * (double)sps));
  Serial.print("speed23 set to "); Serial.print(s_speedSPS);
  Serial.print(" sps (interval=");  Serial.print(s_interval_us);
  Serial.println(" us)");
}

void move2(long steps) {
  if (steps == 0) return;

  bool dirHigh = (steps > 0);
  long todo = labs(steps);

  digitalWrite(Pins::EN2, Pins::ENABLE_LEVEL);
  digitalWrite(Pins::DIR2, dirHigh ? HIGH : LOW);

  for (long i = 0; i < todo; i++) {
    stepOnceTimedOnPin(Pins::STEP2, s_interval_us);
    s_pos2 += dirHigh ? +1 : -1;
    stepOnceTimedOnPin(Pins::STEP2, s_interval_us);
  }

  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
}

void move3(long steps) {
  if (steps == 0) return;

  bool dirHigh = (steps > 0);
  long todo = labs(steps);

  digitalWrite(Pins::EN3, Pins::ENABLE_LEVEL);
  digitalWrite(Pins::DIR3, dirHigh ? HIGH : LOW);

  for (long i = 0; i < todo; i++) {
    stepOnceTimedOnPin(Pins::STEP3, s_interval_us);
    s_pos3 += dirHigh ? +1 : -1;
    stepOnceTimedOnPin(Pins::STEP3, s_interval_us);
  }

  digitalWrite(Pins::EN3, Pins::DISABLE_LEVEL);
}

void moveSync(long steps2, long steps3) {
  long a = labs(steps2), b = labs(steps3);
  if (a == 0 && b == 0) return;

  bool dir2High = (steps2 >= 0);
  bool dir3High = (steps3 >= 0);

  digitalWrite(Pins::EN2, Pins::ENABLE_LEVEL);
  digitalWrite(Pins::EN3, Pins::ENABLE_LEVEL);
  digitalWrite(Pins::DIR2, dir2High ? HIGH : LOW);
  digitalWrite(Pins::DIR3, dir3High ? HIGH : LOW);

  long n = (a > b) ? a : b;
  long acc2 = 0, acc3 = 0;

  unsigned long last = micros();
  for (long i = 0; i < n; i++) {
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

  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
  digitalWrite(Pins::EN3, Pins::DISABLE_LEVEL);
}

void link(long steps) {
  moveSync(steps, -steps);
}

long pos2() { return s_pos2; }
long pos3() { return s_pos3; }


// Move axis #2 until pot[0] reaches target_adc (0..1023) at given speed (sps)
bool move2UntilPotSimple(uint16_t target_adc, long sps) {
  if (!Drivers::hasADS()) {
    Serial.println("move2UntilPot: ADS not present.");
    return false;
  }

  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;

  const unsigned long interval_us = (unsigned long)(1000000.0 / (2.0 * (double)sps));
  const uint8_t hysteresis = 3;
  const unsigned long timeout_ms = 20000UL;

  // Ensure pots module is ready
  Pots::init();   // safe to call repeatedly

  // Seed + choose direction
  uint16_t pot = Pots::readScaled(0);
  bool dirHigh = (target_adc > pot);

  digitalWrite(Pins::EN2, Pins::ENABLE_LEVEL);
  digitalWrite(Pins::DIR2, dirHigh ? HIGH : LOW);

  unsigned long last = micros();
  unsigned long startMs = millis();

  while (true) {
    if (timeout_ms && (millis() - startMs) > timeout_ms) {
      digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
      Serial.println("move2UntilPot: TIMEOUT");
      return false;
    }

    // time-gated toggle for STEP2
    unsigned long now;
    do { now = micros(); } while ((unsigned long)(now - last) < interval_us);
    last = now;

    digitalWrite(Pins::STEP2, HIGH);
    delayMicroseconds(Pins::STEP_PULSE_US);
    digitalWrite(Pins::STEP2, LOW);

    // track position 2
    s_pos2 += dirHigh ? +1 : -1;

    // read pot and decide whether to stop
    pot = Pots::readScaled(0);
    Serial.print("pot[0]="); Serial.println(pot);

    if (dirHigh) {
      if (pot >= (uint16_t)(target_adc - hysteresis)) break;
    } else {
      if (pot <= (uint16_t)(target_adc + hysteresis)) break;
    }
  }

  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
  Serial.print("pot[0]="); Serial.println(pot);
  return true;
}


} // namespace AxisPair
