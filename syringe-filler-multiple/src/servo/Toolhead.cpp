/**
 * @file Toolhead.cpp
 * @brief Direct ESP32 servo control for toolhead lift and coupling.
 */
#include "servo/Toolhead.hpp"

#include "hw/Pins.hpp"

#include <Arduino.h>
#include <ESP32Servo.h>

namespace Toolhead {
namespace {
// Keep legacy caller mapping support from older PCA9685 channels.
static constexpr uint8_t TOOLHEAD_SERVO = 3;
static constexpr uint8_t COUPLING_SERVO = 5;

static constexpr int TOOLHEAD_SERVO_RAISED_POS    = 111;
static constexpr int TOOLHEAD_SERVO_COUPLING_POS1 = 140;
static constexpr int COUPLING_SERVO_COUPLED_POS   = 31;
static constexpr int COUPLING_SERVO_DECOUPLED_POS = 147;

static constexpr int RAMP_MS_FAST = 11;
static constexpr int RAMP_MS_SLOW = 17;

Servo s_toolheadServo;
Servo s_couplerServo;
bool  s_servoReady = false;
bool  s_toolheadAttached = false;
bool  s_couplerAttached = false;

int   s_toolheadAngle = 90;
int   s_couplerAngle  = 90;
bool  s_isCoupled = false;

Servo* resolveServo(uint8_t ch) {
  if (ch == TOOLHEAD_SERVO || ch == 0) return &s_toolheadServo;
  if (ch == COUPLING_SERVO || ch == 1) return &s_couplerServo;
  return nullptr;
}

int* resolveAngleSlot(uint8_t ch) {
  if (ch == TOOLHEAD_SERVO || ch == 0) return &s_toolheadAngle;
  if (ch == COUPLING_SERVO || ch == 1) return &s_couplerAngle;
  return nullptr;
}

bool* resolveAttachedSlot(uint8_t ch) {
  if (ch == TOOLHEAD_SERVO || ch == 0) return &s_toolheadAttached;
  if (ch == COUPLING_SERVO || ch == 1) return &s_couplerAttached;
  return nullptr;
}

int resolvePin(uint8_t ch) {
  if (ch == TOOLHEAD_SERVO || ch == 0) return Pins::SERVO_PIN_TOOLHEAD;
  if (ch == COUPLING_SERVO || ch == 1) return Pins::SERVO_PIN_COUPLER;
  return -1;
}

bool ensureAttached(uint8_t ch) {
  Servo* servo = resolveServo(ch);
  bool* attached = resolveAttachedSlot(ch);
  const int pin = resolvePin(ch);
  if (!servo || !attached || pin < 0) return false;

  if (*attached) return true;

  servo->setPeriodHertz(Pins::SERVO_HZ);
  const int attachPin = servo->attach(pin, Pins::SERVO_MIN_US, Pins::SERVO_MAX_US);
  if (attachPin != pin) {
    Serial.printf("ERROR: servo attach failed (ch=%u pin=%d ret=%d)\n", ch, pin, attachPin);
    *attached = false;
    return false;
  }

  *attached = true;
  return true;
}

void ensureServoInit() {
  if (s_servoReady) return;

  // ESP32Servo requires timer allocation before attach.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  s_toolheadServo.setPeriodHertz(Pins::SERVO_HZ);
  s_couplerServo.setPeriodHertz(Pins::SERVO_HZ);

  const bool toolheadOk = ensureAttached(TOOLHEAD_SERVO);
  const bool couplerOk = ensureAttached(COUPLING_SERVO);

  s_servoReady = toolheadOk && couplerOk && s_toolheadAttached && s_couplerAttached;
  if (s_servoReady) {
    Serial.printf("Toolhead servos ready on GPIO %u/%u\n",
                  Pins::SERVO_PIN_TOOLHEAD,
                  Pins::SERVO_PIN_COUPLER);
  }
}
} // namespace

void init() {
  ensureServoInit();
}

bool isReady() {
  return s_servoReady;
}

bool isRaised() {
  return (digitalRead(Pins::RAISED) == LOW);
}

void setPulseRaw(uint8_t ch, int pulse) {
  ensureServoInit();
  if (!s_servoReady) return;

  Servo* servo = resolveServo(ch);
  bool* attached = resolveAttachedSlot(ch);
  if (!servo || !attached) return;

  if (pulse <= 0) {
    servo->detach();
    *attached = false;
    return;
  }

  if (!ensureAttached(ch)) {
    Serial.printf("ERROR: setPulseRaw failed; servo ch %u unavailable.\n", ch);
    return;
  }

  servo->writeMicroseconds(pulse);
}

void setAngle(uint8_t ch, int angle) {
  ensureServoInit();
  if (!s_servoReady) return;

  Servo* servo = resolveServo(ch);
  int* angleSlot = resolveAngleSlot(ch);
  if (!servo || !angleSlot) return;

  const int bounded = constrain(angle, 0, 180);
  if (*angleSlot == bounded) return;

  if (!ensureAttached(ch)) {
    Serial.printf("ERROR: setAngle failed; servo ch %u unavailable.\n", ch);
    return;
  }

  servo->write(bounded);
  *angleSlot = bounded;

  Serial.print("Servo ");
  Serial.print(ch);
  Serial.print(" -> ");
  Serial.print(bounded);
  Serial.println(" deg");
}

void setAngleSlow(uint8_t ch, int target, int stepDelay) {
  ensureServoInit();
  if (!s_servoReady) return;

  int* angleSlot = resolveAngleSlot(ch);
  if (!angleSlot) return;

  const int boundedTarget = constrain(target, 0, 180);
  const int start = *angleSlot;
  if (start == boundedTarget) return;

  const int step = (boundedTarget > start) ? 1 : -1;
  for (int a = start; a != boundedTarget; a += step) {
    setAngle(ch, a);
    delay(stepDelay <= 0 ? 1 : stepDelay);
  }
  setAngle(ch, boundedTarget);

  Serial.print("Servo ");
  Serial.print(ch);
  Serial.print(" slowly moved to ");
  Serial.println(boundedTarget);
}

void raise() {
  if (!isReady()) {
    Serial.println("ERROR: raise requested before toolhead init.");
    return;
  }

  setAngleSlow(COUPLING_SERVO, COUPLING_SERVO_DECOUPLED_POS, RAMP_MS_SLOW);
  delay(50);
  setAngleSlow(TOOLHEAD_SERVO, TOOLHEAD_SERVO_RAISED_POS, RAMP_MS_FAST);
  s_isCoupled = false;
}

void couple() {
  if (!isReady()) {
    Serial.println("ERROR: couple requested before toolhead init.");
    return;
  }

  s_isCoupled = false;
  setAngleSlow(TOOLHEAD_SERVO, TOOLHEAD_SERVO_COUPLING_POS1, RAMP_MS_FAST);
  delay(100);

  setAngleSlow(COUPLING_SERVO, COUPLING_SERVO_DECOUPLED_POS, RAMP_MS_SLOW);
  delay(100);

  setPulseRaw(TOOLHEAD_SERVO, 0);
  delay(100);

  setAngleSlow(COUPLING_SERVO, COUPLING_SERVO_COUPLED_POS, RAMP_MS_SLOW);
  delay(100);

  setPulseRaw(COUPLING_SERVO, 0);
  s_isCoupled = isRaised();
}

bool isCoupled() {
  return s_isCoupled;
}

bool ensureRaised(uint16_t timeout_ms) {
  ensureServoInit();
  if (!s_servoReady) {
    Serial.println("ERROR: ensureRaised aborted; toolhead servos are uninitialized.");
    return false;
  }
  if (isRaised()) return true;

  setAngle(COUPLING_SERVO, COUPLING_SERVO_DECOUPLED_POS);
  delay(100);
  setAngle(TOOLHEAD_SERVO, TOOLHEAD_SERVO_RAISED_POS);

  const unsigned long start = millis();
  while (!isRaised() && (millis() - start) < timeout_ms) {
    setAngle(COUPLING_SERVO, COUPLING_SERVO_DECOUPLED_POS);
    delay(100);
    setAngle(TOOLHEAD_SERVO, TOOLHEAD_SERVO_RAISED_POS);
    delay(100);
  }

  if (!isRaised()) {
    Serial.println("ERROR: Toolhead not raised (timeout).");
    return false;
  }
  return true;
}

} // namespace Toolhead
