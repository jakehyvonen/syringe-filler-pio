#include "servo/Toolhead.hpp"
#include "hw/Drivers.hpp"
#include "hw/Pins.hpp"
#include <Arduino.h>

namespace Toolhead {

static constexpr int SERVO_MIN     = 150;
static constexpr int SERVO_MAX     = 600;

static constexpr int TOOLHEAD_SERVO  = 3;
static constexpr int COUPLING_SERVO  = 5;

static constexpr int RAISED_POS      = 99;
static constexpr int COUPLING_POS1   = 132;
static constexpr int COUPLED_POS     = 11;
static constexpr int DECOUPLED_POS   = 151;

static int  s_angles[16];
static bool s_anglesInit = false;

static inline void ensureAnglesInit() {
  if (!s_anglesInit) {
    for (int i = 0; i < 16; ++i) s_angles[i] = 90;
    s_anglesInit = true;
  }
}

void init() {
  // nothing special; PCA is initialized in Drivers
}

bool isRaised() {
  // GPIO34/35 require external pull-ups to 3.3V
  return (digitalRead(Pins::RAISED) == LOW);
}

void setPulseRaw(uint8_t ch, int pulse) {
  if (!Drivers::hasPCA()) return;
  Drivers::PCA.writeMicroseconds(ch, pulse);
}

void setAngle(uint8_t ch, int angle) {
  if (!Drivers::hasPCA()) return;
  ensureAnglesInit();

  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  if (s_angles[ch] == angle) return; // avoid redundant I2C writes

  uint16_t p = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  Drivers::PCA.setPWM(ch, 0, p);

  s_angles[ch] = angle;
  Serial.print("Servo "); Serial.print(ch);
  Serial.print(" -> ");   Serial.print(angle);
  Serial.println(" deg");
}

void setAngleSlow(uint8_t ch, int target, int stepDelay) {
  if (!Drivers::hasPCA()) return;
  ensureAnglesInit();

  if (target < 0) target = 0;
  if (target > 180) target = 180;

  int start = s_angles[ch];
  if (start == target) return;

  int step = (target > start) ? 1 : -1;
  for (int a = start; a != target; a += step) {
    uint16_t p = map(a, 0, 180, SERVO_MIN, SERVO_MAX);
    Drivers::PCA.setPWM(ch, 0, p);
    delay(stepDelay <= 0 ? 1 : stepDelay);
  }
  uint16_t p = map(target, 0, 180, SERVO_MIN, SERVO_MAX);
  Drivers::PCA.setPWM(ch, 0, p);
  s_angles[ch] = target;

  Serial.print("Servo "); Serial.print(ch);
  Serial.print(" slowly moved to "); Serial.println(target);
}

void raise() {
  setAngle(TOOLHEAD_SERVO, RAISED_POS);
  delay(100);
  setAngle(COUPLING_SERVO, DECOUPLED_POS);
}

void couple() {
  setAngle(TOOLHEAD_SERVO, COUPLING_POS1);
  delay(71);
  setAngle(COUPLING_SERVO, 151);
  delay(71);
  setPulseRaw(TOOLHEAD_SERVO, 0); // stop pulses briefly
  delay(71);
  setAngle(COUPLING_SERVO, COUPLED_POS);
}

bool ensureRaised(uint16_t timeout_ms) {
  if (!Drivers::hasPCA()) {
    Serial.println("ERROR: No PCA9685; cannot raise toolhead.");
    return false;
  }
  if (isRaised()) return true;

  setAngle(COUPLING_SERVO, DECOUPLED_POS);
  delay(100);
  setAngle(TOOLHEAD_SERVO, RAISED_POS);

  unsigned long start = millis();
  while (!isRaised() && (millis() - start) < timeout_ms) {
    setAngle(COUPLING_SERVO, DECOUPLED_POS);
    delay(100);
    setAngle(TOOLHEAD_SERVO, RAISED_POS);
    delay(100);
  }

  if (!isRaised()) {
    Serial.println("ERROR: Toolhead not raised (timeout).");
    return false;
  }
  return true;
}

} // namespace Toolhead
