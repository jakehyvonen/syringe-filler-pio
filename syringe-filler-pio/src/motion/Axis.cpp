#include "motion/Axis.hpp"
#include "servo/Toolhead.hpp"
#include "hw/Pins.hpp"
#include "hw/Encoder.hpp"    // <-- NEW
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

// Cached safe pulse width (us)
static inline uint32_t pulseWidthUs() {
  uint32_t pw = (Pins::STEP_PULSE_US < 12) ? 12u : (uint32_t)Pins::STEP_PULSE_US;
  return pw;
}

static volatile bool s_phaseHigh = false;

// ISR
static void IRAM_ATTR onStepTimer() {
  const Mode mode = s_mode;
  if (mode == Mode::IDLE) return;

  bool do1 = (mode == Mode::MOVE) && (s_rem > 0);

  if (do1 && !s_phaseHigh) {
    GPIO.out_w1ts = (1UL << Pins::STEP12);
    s_phaseHigh = true;
  } else if (s_phaseHigh) {
    GPIO.out_w1tc = (1UL << Pins::STEP12);
    s_phaseHigh = false;

    if (do1 && s_rem > 0) {
      s_rem--;
      s_pos += s_dirHigh ? +1 : -1;
    }
    if (mode == Mode::MOVE && s_rem <= 0) {
      s_mode = Mode::IDLE;
    }
  }
}

static void waitForIdle() {
  while (s_mode != Mode::IDLE) {
    delay(1);
  }
  if (s_timer) timerAlarmDisable(s_timer);
}

// -----------------------------
// Init
// -----------------------------
void init() {
  pinMode(Pins::STEP12, OUTPUT);
  pinMode(Pins::DIR12,  OUTPUT);
  pinMode(Pins::EN1,    OUTPUT);

  digitalWrite(Pins::EN1, Pins::DISABLE_LEVEL);
  digitalWrite(Pins::DIR12, HIGH);

  s_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(s_timer, &onStepTimer, true);
  timerAlarmWrite(s_timer, s_period_us, true);
  timerAlarmDisable(s_timer);

  s_mode = Mode::IDLE;
  Serial.println("[Axis] init (timer ready).");
}

// -----------------------------
// Speed setter
// -----------------------------
void setSpeedSPS(long sps) {
  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;
  s_speedSPS  = sps;
  s_period_us = (unsigned long)(1000000.0 / (double)sps);

  noInterrupts();
  timerAlarmDisable(s_timer);
  timerAlarmWrite(s_timer, s_period_us, true);
  timerAlarmEnable(s_timer);
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

// Optional helpers
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
// Low-level motion (no encoder)
// -----------------------------
void moveSteps(long steps) {
  if (steps == 0) return;

  if (!Toolhead::ensureRaised()) {
    Serial.println("[Axis] movement blocked (toolhead not raised).");
    return;
  }

  const bool dirHigh = (steps > 0);
  const long todo    = labs(steps);

  enable(true);
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
}

// -----------------------------
// Helper: read encoder position in *steps*
// -----------------------------
static long encPosSteps() {
  // encoder.mm() is already (encoderCount / COUNTS_PER_MM)
  float mm = EncoderHW::mm();
  long steps = (long)lround(mm * Pins::STEPS_PER_MM);
  return steps;
}

// -----------------------------
// High-level moveTo using encoder
// -----------------------------
void moveTo(long targetSteps) {
  // Clamp to soft limits
  long tgt = targetSteps;
  if (tgt < Pins::MIN_POS_STEPS) tgt = Pins::MIN_POS_STEPS;
  if (tgt > Pins::MAX_POS_STEPS) tgt = Pins::MAX_POS_STEPS;

  Serial.printf("[Axis] moveTo(target=%ld)\n", tgt);

  // tuning parameters
  const long tolSteps  = 4;     // ~0.05 mm tolerance
  const long maxChunk  = 400;   // max move per iteration (~5 mm)
  const int  maxIters  = 53;    // safety cap
  const int  debugEvery = 1;    // print every iteration (you can raise to 2–3 to reduce spam)

  for (int i = 0; i < maxIters; ++i) {
    long curEnc = encPosSteps();
    long curAxis = current();
    long err = tgt - curEnc;

    if (i % debugEvery == 0) {
      Serial.printf("[Axis.moveTo] iter=%02d  tgt=%ld  enc=%ld  axis=%ld  err=%+ld\n",
                    i, tgt, curEnc, curAxis, err);
    }

    if (labs(err) <= tolSteps) {
      Serial.printf("[Axis.moveTo] done: |err|=%ld <= tol=%ld (enc=%ld, axis=%ld)\n",
                    labs(err), tolSteps, curEnc, curAxis);
      setCurrent(curEnc);
      return;
    }

    // Limit correction to avoid overshoot
    long stepCmd = err;
    if (stepCmd >  maxChunk) stepCmd =  maxChunk;
    if (stepCmd < -maxChunk) stepCmd = -maxChunk;

    Serial.printf("[Axis.moveTo] -> moveSteps(%ld)\n", stepCmd);

    moveSteps(stepCmd);

    // Give serial some breathing room
    delay(5);
  }

  // If we get here, we didn’t converge within maxIters
  long finalEnc = encPosSteps();
  long finalAxis = current();
  long finalErr = tgt - finalEnc;
  Serial.printf("[Axis.moveTo] MAX_ITERS reached: tgt=%ld enc=%ld axis=%ld err=%ld\n",
                tgt, finalEnc, finalAxis, finalErr);

  // Sync logical step counter to encoder
  setCurrent(finalEnc);
  Serial.println("[Axis.moveTo] logical axis synced to encoder position.");
}

void moveToMM(float mmTarget) {
  // convert mm to steps using your existing scale
  long targetSteps = (long)lround(mmTarget * Pins::STEPS_PER_MM);
  moveTo(targetSteps);
}


} // namespace Axis
