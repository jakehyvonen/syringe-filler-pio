#include "motion/Axis.hpp"
#include "servo/Toolhead.hpp"
#include "hw/Pins.hpp"
#include <Arduino.h>

namespace Axis {

// -----------------------------
// Public position readback
// -----------------------------
static volatile long s_pos = 0;  // logical position in steps
long current() { long v; noInterrupts(); v = s_pos; interrupts(); return v; }
void setCurrent(long s) { noInterrupts(); s_pos = s; interrupts(); }

// -----------------------------
// Speed (FULL PERIOD model)
// -----------------------------
static volatile long          s_speedSPS  = 800;
static volatile unsigned long s_period_us = 1000000UL / 800UL;

// -----------------------------
// Timer + motion state
// -----------------------------
static hw_timer_t *s_timer = nullptr;

enum class Mode : uint8_t { IDLE=0, MOVE };
static volatile Mode s_mode = Mode::IDLE;

// Axis 1 state
static volatile long s_rem     = 0;
static volatile bool s_dirHigh = true;

// Remember if we pulsed this tick (parity with AxisPair style)
static volatile bool s_do1 = false;

// Cached safe pulse width (us)
static inline uint32_t pulseWidthUs() {
  uint32_t pw = (Pins::STEP_PULSE_US < 12) ? 12u : (uint32_t)Pins::STEP_PULSE_US;
  // Long wiring? Bump to 12–20 us if needed.
  return pw;
}

// -----------------------------
// TIMER ISR (two-phase in one visit — parity with AxisPair)
//   1) decide if we step this tick
//   2) drive STEP HIGH, delay 'pw', drive LOW
// -----------------------------
static volatile bool s_phaseHigh = false;

static void IRAM_ATTR onStepTimer() {
  const Mode mode = s_mode;
  if (mode == Mode::IDLE) return;

  bool do1 = (mode == Mode::MOVE) && (s_rem > 0);

  // Two-phase step without busy-wait
  if (do1 && !s_phaseHigh) {
    // phase A: STEP HIGH
    GPIO.out_w1ts = (1UL << Pins::STEP12);  // fast write (optional)
    s_phaseHigh = true;
  } else if (s_phaseHigh) {
    // phase B: hold time has elapsed via timer period; STEP LOW
    GPIO.out_w1tc = (1UL << Pins::STEP12);
    s_phaseHigh = false;

    if (do1 && s_rem > 0) { s_rem--; s_pos += s_dirHigh ? +1 : -1; }
    if (mode == Mode::MOVE && s_rem <= 0) s_mode = Mode::IDLE;
  }
}

// -----------------------------
// Blocking helper: wait until ISR marks IDLE
// -----------------------------
static void waitForIdle() {
  while (s_mode != Mode::IDLE) {
    delay(1);
  }
  // After motion completes, stop timer to avoid free-running ISR
  if (s_timer) timerAlarmDisable(s_timer);
}

// -----------------------------
// Init
// -----------------------------
void init() {
  pinMode(Pins::STEP12, OUTPUT);
  pinMode(Pins::DIR12,  OUTPUT);
  pinMode(Pins::EN1,   OUTPUT);

  digitalWrite(Pins::EN1, Pins::DISABLE_LEVEL);
  digitalWrite(Pins::DIR12, HIGH);

  // Use hardware timer 1 so we don't collide with AxisPair's timer 0
  // Prescaler 80 => 1 tick = 1 us
  s_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(s_timer, &onStepTimer, true);
  timerAlarmWrite(s_timer, s_period_us, true); // auto-reload each period
  timerAlarmDisable(s_timer);

  s_mode = Mode::IDLE;
  Serial.println("[Axis] init (timer ready).");
}

// -----------------------------
// Speed setter (identical update pattern to AxisPair)
// -----------------------------
void setSpeedSPS(long sps) {
  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;
  s_speedSPS  = sps;
  s_period_us = (unsigned long)(1000000.0 / (double)sps);

  // Update timer period immediately
  noInterrupts();
  timerAlarmDisable(s_timer);
  timerAlarmWrite(s_timer, s_period_us, true);
  timerAlarmEnable(s_timer);  // keep parity with AxisPair (enabled even if IDLE)
  interrupts();

  Serial.print("[Axis] speed set to "); Serial.print(s_speedSPS);
  Serial.print(" sps (period=");  Serial.print(s_period_us);
  Serial.println(" us)");
}

// -----------------------------
// Enable/disable
// -----------------------------
void enable(bool on) {
  digitalWrite(Pins::EN1, on ? Pins::ENABLE_LEVEL : Pins::DISABLE_LEVEL);
}

// Optional helpers kept for API parity
void dir(bool high) {
  digitalWrite(Pins::DIR12, high ? HIGH : LOW);
  delayMicroseconds(3);
}
void stepBlocking() {
  digitalWrite(Pins::STEP12, HIGH);
  delayMicroseconds(pulseWidthUs());
  digitalWrite(Pins::STEP12, LOW);
}
// -----------------------------
// Motion
// -----------------------------
void moveSteps(long steps) {
  if (steps == 0) return;

  if (!Toolhead::ensureRaised()) {
    Serial.println("[Axis] movement blocked (toolhead not raised).");
    return;
  }

  const bool dirHigh = (steps > 0);
  const long todo    = labs(steps);

  enable(true);                      // keep enabled
  delayMicroseconds(500);
  digitalWrite(Pins::DIR12, dirHigh ? HIGH : LOW);
  delayMicroseconds(10);

  noInterrupts();
  s_dirHigh = dirHigh;
  s_rem     = todo;
  s_mode    = Mode::MOVE;
  timerAlarmWrite(s_timer, s_period_us, true);
  timerAlarmEnable(s_timer);
  interrupts();

  waitForIdle();

  // DO NOT disable here; keep holding torque for repeatability
  // enable(false);  // <-- removed
}


void moveTo(long targetSteps) {
  long tgt = targetSteps;
  if (tgt < Pins::MIN_POS_STEPS) tgt = Pins::MIN_POS_STEPS;
  if (tgt > Pins::MAX_POS_STEPS) tgt = Pins::MAX_POS_STEPS;

  long cur; noInterrupts(); cur = s_pos; interrupts();
  moveSteps(tgt - cur);
}

} // namespace Axis
