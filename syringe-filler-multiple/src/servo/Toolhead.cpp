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
static constexpr uint8_t TOOLHEAD_SERVO = 3; // keep legacy caller channel mapping
static constexpr uint8_t COUPLING_SERVO = 5;

static constexpr int TOOLHEAD_SERVO_RAISED_POS    = 111;
static constexpr int TOOLHEAD_SERVO_COUPLING_POS1 = 140;
static constexpr int COUPLING_SERVO_COUPLED_POS   = 31;
static constexpr int COUPLING_SERVO_DECOUPLED_POS = 147;

static constexpr int RAMP_MS_FAST = 11;  // per-degree delay for quick but smooth moves
static constexpr int RAMP_MS_SLOW = 17;  // increase if brownouts occur

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

void ensureAttached(uint8_t ch) {
  Servo* servo = resolveServo(ch);
  bool* attached = resolveAttachedSlot(ch);
  const int pin = resolvePin(ch);
  if (!servo || !attached || pin < 0) return;

  if (*attached) return;
  servo->setPeriodHertz(Pins::SERVO_HZ);
  const int attachPin = servo->attach(pin, Pins::SERVO_MIN_US, Pins::SERVO_MAX_US);
  if (attachPin != pin) {
    Serial.printf("FATAL: servo attach failed (ch=%u pin=%d ret=%d)\n", ch, pin, attachPin);
    while (true) delay(1000);
  }
  *attached = true;
}
} // namespace

void init() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  s_toolheadServo.setPeriodHertz(Pins::SERVO_HZ);
  s_couplerServo.setPeriodHertz(Pins::SERVO_HZ);

  const int attachedToolhead = s_toolheadServo.attach(Pins::SERVO_PIN_TOOLHEAD,
                                                       Pins::SERVO_MIN_US,
                                                       Pins::SERVO_MAX_US);
  if (attachedToolhead != Pins::SERVO_PIN_TOOLHEAD) {
    Serial.printf("FATAL: toolhead servo attach failed (pin=%d ret=%d)\n",
                  Pins::SERVO_PIN_TOOLHEAD,
                  attachedToolhead);
    while (true) delay(1000);
  }

  const int attachedCoupler = s_couplerServo.attach(Pins::SERVO_PIN_COUPLER,
                                                     Pins::SERVO_MIN_US,
                                                     Pins::SERVO_MAX_US);
  if (attachedCoupler != Pins::SERVO_PIN_COUPLER) {
    Serial.printf("FATAL: coupler servo attach failed (pin=%d ret=%d)\n",
                  Pins::SERVO_PIN_COUPLER,
                  attachedCoupler);
    while (true) delay(1000);
  }

  s_toolheadAttached = true;
  s_couplerAttached = true;
  s_servoReady = true;
}

bool isRaised() {
  return (digitalRead(Pins::RAISED) == LOW);
}

void setPulseRaw(uint8_t ch, int pulse) {
  if (!s_servoReady) return;

  Servo* servo = resolveServo(ch);
  bool* attached = resolveAttachedSlot(ch);
  if (!servo || !attached) return;

  if (pulse <= 0) {
    servo->detach();
    *attached = false;
    return;
  }

  ensureAttached(ch);
  servo->writeMicroseconds(pulse);
}

void setAngle(uint8_t ch, int angle) {
  if (!s_servoReady) return;

  Servo* servo = resolveServo(ch);
  int* angleSlot = resolveAngleSlot(ch);
  if (!servo || !angleSlot) return;

  const int bounded = constrain(angle, 0, 180);
  if (*angleSlot == bounded) return;

  ensureAttached(ch);
  servo->write(bounded);
  *angleSlot = bounded;

  Serial.print("Servo ");
  Serial.print(ch);
  Serial.print(" -> ");
  Serial.print(bounded);
  Serial.println(" deg");
}

void setAngleSlow(uint8_t ch, int target, int stepDelay) {
  if (!s_servoReady) return;

  int* angleSlot = resolveAngleSlot(ch);
  if (!angleSlot) return;

  const int boundedTarget = constrain(target, 0, 180);
  int start = *angleSlot;
  if (start == boundedTarget) return;

  int step = (boundedTarget > start) ? 1 : -1;
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
  setAngleSlow(COUPLING_SERVO, COUPLING_SERVO_DECOUPLED_POS, RAMP_MS_SLOW);
  delay(50);
  setAngleSlow(TOOLHEAD_SERVO, TOOLHEAD_SERVO_RAISED_POS, RAMP_MS_FAST);
  s_isCoupled = false;
}

void couple() {
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
  if (isRaised()) return true;

  setAngle(COUPLING_SERVO, COUPLING_SERVO_DECOUPLED_POS);
  delay(100);
  setAngle(TOOLHEAD_SERVO, TOOLHEAD_SERVO_RAISED_POS);

  unsigned long start = millis();
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
