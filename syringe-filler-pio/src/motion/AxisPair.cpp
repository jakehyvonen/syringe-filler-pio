/**
 * @file AxisPair.cpp
 * @brief Timer-driven motion control for axes 2 and 3.
 */
#include "motion/AxisPair.hpp"
#include "hw/Drivers.hpp"
#include "hw/Pots.hpp"
#include "hw/Pins.hpp"
#include "hw/Bases.hpp"

#include <Arduino.h>
#include "soc/gpio_struct.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us()

namespace AxisPair {

// ============================================================
// Public position readback
// ============================================================
static volatile long s_pos2 = 0;  // toolhead syringe (STEP2/DIR2/EN2)
static volatile long s_pos3 = 0;  // selected base syringe (STEP3/DIR3 + Bases::hold())

// Return the current position of axis 2 in steps.
long pos2() { return s_pos2; }
// Return the current position of axis 3 in steps.
long pos3() { return s_pos3; }

// ============================================================
// Speed (FULL PERIOD model)
// ============================================================
// Keep consistent: s_speedSPS <-> s_period_us derived together.
static volatile long          s_speedSPS  = 100;                 // steps/sec (dominant axis tick rate)
static volatile unsigned long s_period_us = 1000000UL / 100UL;   // 1e6 / sps

// ============================================================
// Timer + motion state
// ============================================================
static hw_timer_t* s_timer = nullptr;

enum class Mode : uint8_t { IDLE=0, MOVE2, MOVE3, SYNC };
static volatile Mode s_mode = Mode::IDLE;

// Axis 2 state
static volatile long s_rem2 = 0;
static volatile bool s_dir2High = true;

// Axis 3 state
static volatile long s_rem3 = 0;
static volatile bool s_dir3High = true;

// SYNC accumulators (Bresenham)
static volatile long s_syncA = 0;  // |steps2|
static volatile long s_syncB = 0;  // |steps3|
static volatile long s_syncN = 0;  // max(a,b)
static volatile long s_acc2  = 0;
static volatile long s_acc3  = 0;

// ============================================================
// Pulse width and timing clamps
// ============================================================
static inline uint32_t pulseWidthUs() {
  // Minimum 8us is a decent safety floor for real-world wiring.
  uint32_t pw = (Pins::STEP_PULSE_US < 8) ? 8u : (uint32_t)Pins::STEP_PULSE_US;
  return pw;
}

static inline unsigned long clampPeriodUs(unsigned long p) {
  // Ensure the step period is longer than the high-pulse time.
  // Give it a few extra microseconds so we're not right on the edge.
  const uint32_t pw = pulseWidthUs();
  const unsigned long minp = (unsigned long)(pw + 8u);
  if (p < minp) p = minp;
  return p;
}

// ============================================================
// Fast GPIO for STEP pins
// ============================================================
static inline void IRAM_ATTR step2High() { GPIO.out_w1ts = (1UL << Pins::STEP2); }
static inline void IRAM_ATTR step2Low()  { GPIO.out_w1tc = (1UL << Pins::STEP2); }
static inline void IRAM_ATTR step3High() { GPIO.out_w1ts = (1UL << Pins::STEP3); }
static inline void IRAM_ATTR step3Low()  { GPIO.out_w1tc = (1UL << Pins::STEP3); }

// ============================================================
// TIMER ISR (single interrupt per step period; IRAM-safe)
// IMPORTANT: no timerAlarmWrite/Enable calls here.
// ============================================================
// Timer ISR that emits pulses for axes 2/3 and updates positions.
static void IRAM_ATTR onStepTimer() {
  const Mode mode = s_mode;
  if (mode == Mode::IDLE) return;

  bool do2 = false, do3 = false;

  switch (mode) {
    case Mode::MOVE2:
      do2 = (s_rem2 > 0);
      break;

    case Mode::MOVE3:
      do3 = (s_rem3 > 0);
      break;

    case Mode::SYNC:
      if (s_rem2 > 0 || s_rem3 > 0) {
        // Bresenham-style: each ISR tick is one "time slice" for the dominant axis.
        s_acc2 += s_syncA;
        if (s_acc2 >= s_syncN) { s_acc2 -= s_syncN; do2 = (s_rem2 > 0); }

        s_acc3 += s_syncB;
        if (s_acc3 >= s_syncN) { s_acc3 -= s_syncN; do3 = (s_rem3 > 0); }
      }
      break;

    default:
      break;
  }

  // Emit pulses (HIGH then LOW) using IRAM-safe delay.
  // This is the stable version (no timer re-arming inside ISR).
  const uint32_t pw = pulseWidthUs();
  if (do2) step2High();
  if (do3) step3High();
  esp_rom_delay_us(pw);
  if (do2) step2Low();
  if (do3) step3Low();

  // Update counters/positions
  if (do2 && s_rem2 > 0) { s_rem2--; s_pos2 += s_dir2High ? +1 : -1; }
  if (do3 && s_rem3 > 0) { s_rem3--; s_pos3 += s_dir3High ? +1 : -1; }

  // Done?
  bool done = false;
  switch (mode) {
    case Mode::MOVE2: done = (s_rem2 <= 0); break;
    case Mode::MOVE3: done = (s_rem3 <= 0); break;
    case Mode::SYNC:  done = (s_rem2 <= 0 && s_rem3 <= 0); break;
    default: break;
  }
  if (done) s_mode = Mode::IDLE;
}

// ============================================================
// Init
// ============================================================
// Initialize GPIO and timer for axes 2 and 3.
void init() {
  // TOOLHEAD (Axis 2)
  pinMode(Pins::STEP2, OUTPUT);
  pinMode(Pins::DIR2,  OUTPUT);
  pinMode(Pins::EN2,   OUTPUT);
  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
  digitalWrite(Pins::STEP2, LOW);

  // BASES (Axis 3)
  pinMode(Pins::STEP3, OUTPUT);
  pinMode(Pins::DIR3,  OUTPUT);
  digitalWrite(Pins::STEP3, LOW);

  // Hardware timer 0, prescaler 80 => 1 tick = 1 us
  s_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(s_timer, &onStepTimer, true);

  // Auto-reload at constant period. We do NOT change this inside ISR.
  s_period_us = clampPeriodUs(1000000UL / (unsigned long)s_speedSPS);
  timerAlarmWrite(s_timer, s_period_us, true);
  timerAlarmDisable(s_timer);

  s_mode = Mode::IDLE;
  Serial.print("[AxisPair] init (timer ready). period_us=");
  Serial.print((unsigned long)s_period_us);
  Serial.print(" pw_us=");
  Serial.println((uint32_t)pulseWidthUs());
}

// ============================================================
// Speed setter
// ============================================================
// Update the shared step period for axes 2 and 3.
void setSpeedSPS(long sps) {
  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;

  const unsigned long newPeriod = clampPeriodUs((unsigned long)(1000000.0 / (double)sps));

  noInterrupts();
  s_speedSPS  = sps;
  s_period_us = newPeriod;
  // Safe to update timer outside ISR
  timerAlarmDisable(s_timer);
  timerAlarmWrite(s_timer, s_period_us, true);
  timerAlarmEnable(s_timer);
  interrupts();

  Serial.print("[AxisPair] speed set to ");
  Serial.print((long)s_speedSPS);
  Serial.print(" sps (period=");
  Serial.print((unsigned long)s_period_us);
  Serial.println(" us)");
}

// ============================================================
// Blocking helper: wait until ISR marks IDLE
// ============================================================
// Block until the ISR completes the current motion.
static void waitForIdle() {
  while (s_mode != Mode::IDLE) {
    delay(1);
  }
  timerAlarmDisable(s_timer);
}

// ============================================================
// AXIS 2: Toolhead syringe (EN2)
// ============================================================
// Move axis 2 by a signed step count.
void move2(long steps) {
  if (steps == 0) return;

  const bool dirHigh = (steps > 0);
  const long todo    = labs(steps);

  Serial.print("[AxisPair] move2 steps="); Serial.println(steps);

  digitalWrite(Pins::EN1, Pins::DISABLE_LEVEL); // keep gantry disabled during axis 2 moves

  digitalWrite(Pins::EN2, Pins::ENABLE_LEVEL);
  delayMicroseconds(500);
  digitalWrite(Pins::DIR2, dirHigh ? HIGH : LOW);
  delayMicroseconds(10);

  noInterrupts();
  s_dir2High = dirHigh;
  s_rem2     = todo;
  s_mode     = Mode::MOVE2;
  timerAlarmEnable(s_timer);
  interrupts();

  waitForIdle();
  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
}

// ============================================================
// AXIS 3: Selected base syringe (Bases::hold)
// ============================================================
// Move axis 3 by a signed step count.
void move3(long steps) {
  if (steps == 0) return;

  const bool dirHigh = (steps > 0);
  const long todo    = labs(steps);

  Serial.print("[AxisPair] move3 steps="); Serial.println(steps);

  Bases::hold(true);
  delayMicroseconds(500);
  digitalWrite(Pins::DIR3, dirHigh ? HIGH : LOW);
  delayMicroseconds(10);

  noInterrupts();
  s_dir3High = dirHigh;
  s_rem3     = todo;
  s_mode     = Mode::MOVE3;
  timerAlarmEnable(s_timer);
  interrupts();

  waitForIdle();
  Bases::hold(false);
}

// ============================================================
// Synchronous move (toolhead + base)
// ============================================================
// Move axes 2 and 3 synchronously.
void moveSync(long steps2, long steps3) {
  const long a = labs(steps2), b = labs(steps3);
  if (a == 0 && b == 0) return;

  const bool dir2High = (steps2 >= 0);
  const bool dir3High = (steps3 >= 0);

  Serial.print("[AxisPair] moveSync s2="); Serial.print(steps2);
  Serial.print(" s3="); Serial.println(steps3);

  digitalWrite(Pins::EN2, Pins::ENABLE_LEVEL);
  Bases::hold(true);
  delayMicroseconds(500);

  digitalWrite(Pins::DIR2, dir2High ? HIGH : LOW);
  digitalWrite(Pins::DIR3, dir3High ? HIGH : LOW);
  delayMicroseconds(10);

  noInterrupts();
  s_dir2High = dir2High;
  s_dir3High = dir3High;

  s_rem2  = a;
  s_rem3  = b;

  s_syncA = a;
  s_syncB = b;
  s_syncN = (a > b) ? a : b;

  s_acc2  = 0;
  s_acc3  = 0;

  s_mode  = Mode::SYNC;
  timerAlarmEnable(s_timer);
  interrupts();

  waitForIdle();

  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
  Bases::hold(false);
}

// Convenience move that links axes in opposite directions.
void link(long steps) { moveSync(steps, -steps); }

// ============================================================
// Axis 2 closed-loop helper (blocking)
// ============================================================
// Move axis 2 until the pot reaches the target ADC count.
bool move2UntilPotSimple(uint16_t target_adc, long sps) {
  if (!Drivers::hasADS()) {
    Serial.println("[AxisPair] move2UntilPot: ADS not present.");
    return false;
  }

  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;

  const unsigned long period_us = clampPeriodUs((unsigned long)(1000000.0 / (double)sps));
  const uint8_t hysteresis = 3;
  const unsigned long timeout_ms = 20000UL;

  Pots::init();

  uint16_t pot = Pots::readScaled(0);
  bool dirHigh = (target_adc > pot);

  digitalWrite(Pins::EN2, Pins::ENABLE_LEVEL);
  delayMicroseconds(500);
  digitalWrite(Pins::DIR2, dirHigh ? HIGH : LOW);
  delayMicroseconds(10);

  unsigned long nextEdge = micros();
  unsigned long startMs = millis();

  while (true) {
    if (timeout_ms && (millis() - startMs) > timeout_ms) {
      digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
      Serial.println("[AxisPair] move2UntilPot: TIMEOUT");
      return false;
    }

    while ((long)(micros() - nextEdge) < 0) { /* spin */ }
    nextEdge += period_us;

    digitalWrite(Pins::STEP2, HIGH);
    esp_rom_delay_us(pulseWidthUs());
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
