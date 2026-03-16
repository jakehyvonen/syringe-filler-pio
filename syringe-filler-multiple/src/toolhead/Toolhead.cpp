/**
 * @file Toolhead.cpp
 * @brief Toolhead lift and coupling control using stepper axis 4.
 */
#include "toolhead/Toolhead.hpp"

#include "hw/Drivers.hpp"
#include "hw/Pins.hpp"

#include <Arduino.h>

namespace Toolhead {
namespace {
static constexpr long COUPLE_STAGE1_STEPS = 1661;
static constexpr long COUPLE_STAGE2_MAX_TOTAL_STEPS = 2032;
static constexpr uint16_t COUPLE_STAGE1_SETTLE_MS = 300;
static constexpr uint16_t COUPLE_STAGE2_SETTLE_MS = 1221;

bool s_ready = false;
bool s_isCoupled = false;

volatile long s_pos4 = 0;
long s_speedSPS = 400;
unsigned long s_stepPeriodUs = 2500;

void writeEnableMcp(uint8_t pin, uint8_t level) {
  if (!Drivers::hasBaseEnableExpander()) return;
  Drivers::BASE_EN_EXPANDER.pinMode(pin, OUTPUT);
  Drivers::BASE_EN_EXPANDER.digitalWrite(pin, level);
}

void enable4(bool on) {
  writeEnableMcp(Pins::EN4_MCP, on ? Pins::ENABLE_LEVEL : Pins::DISABLE_LEVEL);
}

void step4Blocking() {
  digitalWrite(Pins::STEP4, HIGH);
  delayMicroseconds(Pins::STEP_PULSE_US);
  digitalWrite(Pins::STEP4, LOW);
}

bool isCoupledLimitHit() {
  return (digitalRead(Pins::COUPLED) == LOW);
}
} // namespace

void init() {
  pinMode(Pins::STEP4, OUTPUT);
  pinMode(Pins::DIR4, OUTPUT);
  digitalWrite(Pins::STEP4, LOW);
  enable4(false);
  setSpeedSPS(s_speedSPS);
  s_ready = true;
  Serial.printf("Toolhead ready (stepper axis4 via EN4 MCP pin %u)\n", Pins::EN4_MCP);
}

bool isReady() {
  return s_ready;
}

bool isRaised() {
  return (digitalRead(Pins::RAISED) == LOW);
}

void setSpeedSPS(long sps) {
  if (sps < 1) sps = 1;
  if (sps > 4000) sps = 4000;
  s_speedSPS = sps;
  s_stepPeriodUs = (unsigned long)(1000000.0 / (double)s_speedSPS);
}

void moveSteps(long steps) {
  if (steps == 0) return;

  const bool dirHigh = (steps > 0);
  const long todo = labs(steps);

  enable4(true);
  delayMicroseconds(500);

  digitalWrite(Pins::DIR4, dirHigh ? HIGH : LOW);
  delayMicroseconds(10);

  for (long i = 0; i < todo; ++i) {
    step4Blocking();
    if (s_stepPeriodUs > Pins::STEP_PULSE_US) {
      delayMicroseconds(s_stepPeriodUs - Pins::STEP_PULSE_US);
    }
    s_pos4 += dirHigh ? 1 : -1;
  }

  enable4(false);
}

long current() { return s_pos4; }
void setCurrent(long steps) { s_pos4 = steps; }

bool homeRaised(uint16_t timeout_ms) {
  if (isRaised()) {
    setCurrent(0);
    return true;
  }

  const unsigned long start = millis();
  enable4(true);
  delayMicroseconds(300);
  digitalWrite(Pins::DIR4, LOW);
  delayMicroseconds(10);

  uint16_t n = 0;
  while ((millis() - start) < timeout_ms) {
    step4Blocking();
    if (s_stepPeriodUs > Pins::STEP_PULSE_US) delayMicroseconds(s_stepPeriodUs - Pins::STEP_PULSE_US);

    if ((++n & 0x0F) == 0) {
      if (isRaised()) break;
    }
  }

  enable4(false);

  if (!isRaised()) {
    Serial.println("ERROR: toolhead home4 timeout waiting for RAISED switch.");
    return false;
  }

  setCurrent(0);
  return true;
}

void raise() {
  (void)homeRaised();
  s_isCoupled = false;
}

void couple() {
  if (!s_ready) {
    Serial.println("ERROR: couple requested before toolhead init.");
    return;
  }

  if (!ensureRaised()) {
    Serial.println("ERROR: couple aborted; toolhead not raised.");
    return;
  }

  s_isCoupled = false;

  moveSteps(COUPLE_STAGE1_STEPS);
  delay(COUPLE_STAGE1_SETTLE_MS);

  const long stage2MaxSteps = COUPLE_STAGE2_MAX_TOTAL_STEPS - COUPLE_STAGE1_STEPS;
  if (stage2MaxSteps <= 0) {
    Serial.println("ERROR: invalid coupling stage limits.");
    return;
  }

  enable4(true);
  delayMicroseconds(500);
  digitalWrite(Pins::DIR4, HIGH);
  delayMicroseconds(10);

  bool coupledLimitReached = false;
  for (long i = 0; i < stage2MaxSteps; ++i) {
    step4Blocking();
    if (s_stepPeriodUs > Pins::STEP_PULSE_US) {
      delayMicroseconds(s_stepPeriodUs - Pins::STEP_PULSE_US);
    }
    s_pos4 += 1;

    if (isCoupledLimitHit()) {
      coupledLimitReached = true;
      break;
    }
  }

  enable4(false);

  if (!coupledLimitReached) {
    Serial.println("ERROR: couple timeout; COUPLED switch not reached before stage-2 limit.");
    return;
  }

  delay(COUPLE_STAGE2_SETTLE_MS);
  s_isCoupled = true;
  Serial.println("s_coupled = true");
}

bool isCoupled() {
  return s_isCoupled;
}

bool ensureRaised(uint16_t timeout_ms) {
  if (isRaised()) return true;
  return homeRaised(timeout_ms);
}

} // namespace Toolhead
