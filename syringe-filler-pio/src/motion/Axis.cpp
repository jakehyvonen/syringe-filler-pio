#include "motion/Axis.hpp"
#include "servo/Toolhead.hpp"
#include "hw/Pins.hpp"
#include <Arduino.h>



namespace Axis {

static volatile long          s_currentSteps = 0;
static volatile long          s_speedSPS     = 500;
static volatile unsigned long s_period_us    = 1000000UL / 500UL;
static volatile bool          s_enabled      = false;

static hw_timer_t *s_timer = nullptr;

enum class Mode : uint8_t { IDLE = 0, MOVE };
static volatile Mode s_mode = Mode::IDLE;

static volatile long s_remaining = 0;
static volatile bool s_dirHigh   = true;

// --- NEW: backlash handling ---
static constexpr long BACKLASH_STEPS = 40;  // tune this for your mechanics (start 20–80)
static volatile bool s_lastDirHigh   = true;
static volatile long s_compToIgnore  = 0;   // during a reversal, ignore this many steps in position count
// --------------------------------

static inline uint32_t pulseWidthUs() {
  uint32_t pw = (Pins::STEP_PULSE_US < 8) ? 8u : (uint32_t)Pins::STEP_PULSE_US;
  return pw;
}

static void IRAM_ATTR onStepTimer() {
  if (s_mode == Mode::IDLE) return;
  if (s_remaining <= 0) {
    s_mode = Mode::IDLE;
    return;
  }

  // STEP pulse
  digitalWrite(Pins::STEP1, HIGH);
  delayMicroseconds(pulseWidthUs());
  digitalWrite(Pins::STEP1, LOW);

  // Bookkeeping for remaining & position
  s_remaining--;

  // If we're taking up backlash after a reversal, consume steps
  if (s_compToIgnore > 0) {
    s_compToIgnore--;
    // Do NOT change s_currentSteps for these “lash take-up” steps
  } else {
    s_currentSteps += s_dirHigh ? +1 : -1;
  }

  if (s_remaining <= 0) {
    s_mode = Mode::IDLE;
  }
}

static void waitForIdle() {
  while (s_mode != Mode::IDLE) {
    delay(1);
  }
  if (s_timer) {
    timerAlarmDisable(s_timer);
  }
}

void init() {
  pinMode(Pins::STEP1, OUTPUT);
  pinMode(Pins::DIR1,  OUTPUT);
  pinMode(Pins::EN1,   OUTPUT);
  digitalWrite(Pins::EN1, Pins::DISABLE_LEVEL);
  digitalWrite(Pins::DIR1, HIGH);

  s_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(s_timer, &onStepTimer, true);
  timerAlarmWrite(s_timer, s_period_us, true);
  timerAlarmDisable(s_timer);

  s_mode = Mode::IDLE;
  s_lastDirHigh = true;
  s_compToIgnore = 0;

  Serial.println("[Axis] init (timer ready).");
}

void setSpeedSPS(long sps) {
  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;
  noInterrupts();
  s_speedSPS  = sps;
  s_period_us = (unsigned long)(1000000.0 / (double)sps);

  timerAlarmDisable(s_timer);
  timerAlarmWrite(s_timer, s_period_us, true);
  if (s_mode != Mode::IDLE) {
    timerAlarmEnable(s_timer);
  }
  interrupts();

  Serial.print("Axis1 speed set "); Serial.print(sps);
  Serial.print(" sps (period=");  Serial.print(s_period_us);
  Serial.println(" us)");
}

void enable(bool on) {
  s_enabled = on;
  digitalWrite(Pins::EN1, on ? Pins::ENABLE_LEVEL : Pins::DISABLE_LEVEL);
}

static inline void setDir(bool high) {
  digitalWrite(Pins::DIR1, high ? HIGH : LOW);
  // A4988 DIR setup/hold min is sub-µs; 10 µs is generous and safe:
  delayMicroseconds(10);
}

void stepBlocking() {
  digitalWrite(Pins::STEP1, HIGH);
  delayMicroseconds(pulseWidthUs());
  digitalWrite(Pins::STEP1, LOW);
}

void moveSteps(long steps) {
  if (steps == 0) return;
  if (!Toolhead::ensureRaised()) {
    Serial.println("Axis1: movement blocked (toolhead not raised).");
    return;
  }

  const bool dirHigh = (steps > 0);
  long todo          = labs(steps);

  enable(true);
  delayMicroseconds(500);

  // --- detect direction change for backlash compensation ---
  noInterrupts();
  const bool reversing = (dirHigh != s_lastDirHigh);
  interrupts();

  setDir(dirHigh);

  noInterrupts();
  s_dirHigh = dirHigh;

  if (reversing) {
    // Add physical steps to take up lash, but don’t advance logical position
    s_compToIgnore = BACKLASH_STEPS;
    todo += BACKLASH_STEPS;
    s_lastDirHigh = dirHigh;
  }

  s_remaining = todo;
  s_mode      = Mode::MOVE;

  // ensure current period
  timerAlarmWrite(s_timer, s_period_us, true);
  timerAlarmEnable(s_timer);
  interrupts();

  waitForIdle();
  enable(false);
}

void moveTo(long targetSteps) {
  long tgt = targetSteps;
  if (tgt < Pins::MIN_POS_STEPS) tgt = Pins::MIN_POS_STEPS;
  if (tgt > Pins::MAX_POS_STEPS) tgt = Pins::MAX_POS_STEPS;

  long cur;
  noInterrupts();
  cur = s_currentSteps;     // atomic read of shared counter
  interrupts();

  long delta = tgt - cur;
  moveSteps(delta);
}


void dir(bool high) {
  digitalWrite(Pins::DIR1, high ? HIGH : LOW);
}

long current() {
  long v;
  noInterrupts();
  v = s_currentSteps;
  interrupts();
  return v;
}

void setCurrent(long s) {
  noInterrupts();
  s_currentSteps = s;
  interrupts();
}

} // namespace Axis
