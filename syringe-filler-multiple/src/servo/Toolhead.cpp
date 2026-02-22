/**
 * @file Toolhead.cpp
 * @brief Toolhead lift stepper and coupler servo control.
 */
#include "servo/Toolhead.hpp"

#include "hw/Drivers.hpp"
#include "hw/Pins.hpp"

#include <Arduino.h>
#include <ESP32Servo.h>

namespace Toolhead {
namespace {
static constexpr uint8_t COUPLING_SERVO = 1;
static constexpr int COUPLING_SERVO_COUPLED_POS   = 31;
static constexpr int COUPLING_SERVO_DECOUPLED_POS = 147;
static constexpr int RAMP_MS_SLOW_DEFAULT = 17;

Servo s_couplerServo;
bool  s_servoReady = false;
bool  s_couplerAttached = false;
int   s_couplerAngle  = 90;
bool  s_isCoupled = false;
int   s_rampMsSlow = RAMP_MS_SLOW_DEFAULT;

volatile long s_pos4 = 0;
long s_speedSPS = 400;
unsigned long s_stepPeriodUs = 2500;

bool ensureCouplerAttached() {
  if (s_couplerAttached) return true;

  s_couplerServo.setPeriodHertz(Pins::SERVO_HZ);
  const int ret = s_couplerServo.attach(Pins::SERVO_PIN_COUPLER, Pins::SERVO_MIN_US, Pins::SERVO_MAX_US);
  if (ret < 0) {
    Serial.printf("ERROR: coupler servo attach failed (pin=%u ret=%d)\n", Pins::SERVO_PIN_COUPLER, ret);
    s_couplerAttached = false;
    return false;
  }

  s_couplerAttached = true;
  if (s_couplerAngle >= 0 && s_couplerAngle <= 180) {
    s_couplerServo.write(s_couplerAngle);
    delay(20);
  }
  return true;
}

void ensureInit() {
  if (s_servoReady) return;

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  s_servoReady = ensureCouplerAttached();
  if (s_servoReady) {
    Serial.printf("Toolhead ready (coupler servo GPIO %u, toolhead stepper via EN4 MCP pin %u)\n",
                  Pins::SERVO_PIN_COUPLER,
                  Pins::EN4_MCP);
  }
}

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
} // namespace

void init() {
  pinMode(Pins::STEP4, OUTPUT);
  pinMode(Pins::DIR4, OUTPUT);
  digitalWrite(Pins::STEP4, LOW);
  enable4(false);
  ensureInit();
  setSpeedSPS(s_speedSPS);
}

bool isReady() {
  return s_servoReady;
}

bool isRaised() {
  return (digitalRead(Pins::RAISED) == LOW);
}

void setPulseRaw(uint8_t ch, int pulse) {
  ensureInit();
  if (!s_servoReady) return;
  if (ch != COUPLING_SERVO) {
    Serial.println("WARN: toolhead servo removed; only coupler channel is supported.");
    return;
  }

  if (pulse <= 0) {
    s_couplerServo.detach();
    s_couplerAttached = false;
    s_couplerAngle = -1;
    return;
  }

  if (!ensureCouplerAttached()) return;
  s_couplerServo.writeMicroseconds(pulse);
}

void setAngle(uint8_t ch, int angle) {
  ensureInit();
  if (!s_servoReady) return;
  if (ch != COUPLING_SERVO) {
    Serial.println("WARN: toolhead servo removed; use move4/home4 for toolhead motion.");
    return;
  }

  const int bounded = constrain(angle, 0, 180);
  if (s_couplerAngle == bounded) return;

  if (!ensureCouplerAttached()) return;

  s_couplerServo.write(bounded);
  s_couplerAngle = bounded;
}

void setAngleSlow(uint8_t ch, int target, int stepDelay) {
  ensureInit();
  if (!s_servoReady) return;
  if (ch != COUPLING_SERVO) {
    Serial.println("WARN: toolhead servo removed; only coupler channel supports servoslow.");
    return;
  }

  const int boundedTarget = constrain(target, 0, 180);
  const int start = s_couplerAngle;
  if (start == boundedTarget) return;

  const int step = (boundedTarget > start) ? 1 : -1;
  for (int a = start; a != boundedTarget; a += step) {
    setAngle(ch, a);
    delay(stepDelay <= 0 ? 1 : stepDelay);
  }
  setAngle(ch, boundedTarget);
}

void setSlowRampMs(int delayMs) {
  s_rampMsSlow = delayMs <= 0 ? 1 : delayMs;
}

int getSlowRampMs() {
  return s_rampMsSlow;
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

  // Axis 4 has dedicated STEP/DIR lines; only its own driver is toggled here.
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

  setAngle(COUPLING_SERVO, COUPLING_SERVO_DECOUPLED_POS);

  const unsigned long start = millis();
  enable4(true);
  delayMicroseconds(300);
  digitalWrite(Pins::DIR4, LOW);
  delayMicroseconds(10);

  uint16_t n = 0;
  while ((millis() - start) < timeout_ms) {
    step4Blocking();
    if (s_stepPeriodUs > Pins::STEP_PULSE_US) delayMicroseconds(s_stepPeriodUs - Pins::STEP_PULSE_US);

    if ((++n & 0x0F) == 0) { // every 16 steps
      if (isRaised()) break;
    } else {
      // fast path: skip digitalRead this iteration
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
  ensureInit();
  if (!s_servoReady) {
    Serial.println("ERROR: couple requested before toolhead init.");
    return;
  }

  if (!ensureRaised()) {
    Serial.println("ERROR: couple aborted; toolhead not raised.");
    return;
  }

  setAngleSlow(COUPLING_SERVO, COUPLING_SERVO_DECOUPLED_POS, s_rampMsSlow);
  delay(100);
  setAngleSlow(COUPLING_SERVO, COUPLING_SERVO_COUPLED_POS, s_rampMsSlow);
  delay(100);

  s_isCoupled = true;
}

bool isCoupled() {
  return s_isCoupled;
}

bool ensureRaised(uint16_t timeout_ms) {
  if (isRaised()) return true;
  return homeRaised(timeout_ms);
}

} // namespace Toolhead
