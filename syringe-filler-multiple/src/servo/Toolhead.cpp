/**
 * @file Toolhead.cpp
 * @brief PCA9685 servo control for toolhead lift and coupling.
 */
#include "servo/Toolhead.hpp"
#include "hw/Drivers.hpp"
#include "hw/Pins.hpp"
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

namespace Toolhead {

static constexpr int SERVO_MIN     = 150;
static constexpr int SERVO_MAX     = 600;

static constexpr int TOOLHEAD_SERVO  = 3;
static constexpr int COUPLING_SERVO  = 5;

static constexpr int TOOLHEAD_SERVO_RAISED_POS      = 111;
static constexpr int TOOLHEAD_SERVO_COUPLING_POS1   = 140;
static constexpr int COUPLING_SERVO_COUPLED_POS     = 31;
static constexpr int COUPLING_SERVO_DECOUPLED_POS   = 147;

static constexpr int RAMP_MS_FAST = 11;   // per-degree delay for quick but smooth moves
static constexpr int RAMP_MS_SLOW = 17;  // increase if brownouts occur


static int  s_angles[16];
static bool s_anglesInit = false;
static bool s_isCoupled = false;


static Adafruit_PWMServoDriver s_pca; // local servo controller ownership
static bool s_servoInitTried = false;
static bool s_servoReady = false;

static void ensureServoInit() {
  if (s_servoInitTried) return;
  s_servoInitTried = true;
  if (s_pca.begin()) {
    s_pca.setPWMFreq(60);
    s_servoReady = true;
    Serial.println("Toolhead servo controller ready @0x40");
  } else {
    s_servoReady = false;
    Serial.println("WARN: Toolhead servo controller not found @0x40.");
  }
}


static inline void ensureAnglesInit() {
  if (!s_anglesInit) {
    for (int i = 0; i < 16; ++i) s_angles[i] = 90;
    s_anglesInit = true;
  }
}

// Initialize toolhead state.
void init() {
  ensureServoInit();
}

// Return true when the toolhead raised switch is active.
bool isRaised() {
  // GPIO34/35 require external pull-ups to 3.3V
  return (digitalRead(Pins::RAISED) == LOW);
}

// Set a raw microsecond pulse on a PCA9685 channel.
void setPulseRaw(uint8_t ch, int pulse) {
  ensureServoInit();
  if (!s_servoReady) return;
  s_pca.writeMicroseconds(ch, pulse);
}

// Set a servo channel to an absolute angle.
void setAngle(uint8_t ch, int angle) {
  ensureServoInit();
  if (!s_servoReady) return;
  ensureAnglesInit();

  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  if (s_angles[ch] == angle) return; // avoid redundant I2C writes

  uint16_t p = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  s_pca.setPWM(ch, 0, p);

  s_angles[ch] = angle;
  Serial.print("Servo "); Serial.print(ch);
  Serial.print(" -> ");   Serial.print(angle);
  Serial.println(" deg");
}

// Sweep a servo channel to a target angle with delay.
void setAngleSlow(uint8_t ch, int target, int stepDelay) {
  ensureServoInit();
  if (!s_servoReady) return;
  ensureAnglesInit();

  if (target < 0) target = 0;
  if (target > 180) target = 180;

  int start = s_angles[ch];
  if (start == target) return;

  int step = (target > start) ? 1 : -1;
  for (int a = start; a != target; a += step) {
    uint16_t p = map(a, 0, 180, SERVO_MIN, SERVO_MAX);
    s_pca.setPWM(ch, 0, p);
    delay(stepDelay <= 0 ? 1 : stepDelay);
  }
  uint16_t p = map(target, 0, 180, SERVO_MIN, SERVO_MAX);
  s_pca.setPWM(ch, 0, p);
  s_angles[ch] = target;

  Serial.print("Servo "); Serial.print(ch);
  Serial.print(" slowly moved to "); Serial.println(target);
}

// Raise the toolhead and decouple the coupler.
void raise() {
  setAngleSlow(COUPLING_SERVO, COUPLING_SERVO_DECOUPLED_POS, RAMP_MS_SLOW);
  delay(50); // small settle time
  setAngleSlow(TOOLHEAD_SERVO, TOOLHEAD_SERVO_RAISED_POS, RAMP_MS_FAST);
  s_isCoupled = false;
}

// Run the coupling motion sequence.
void couple() {
  s_isCoupled = false;
  // Approach coupling position slowly, then close the coupler smoothly
  setAngleSlow(TOOLHEAD_SERVO, TOOLHEAD_SERVO_COUPLING_POS1, RAMP_MS_FAST);
  delay(100);

  // Optional: brief pause gives the mechanics time to settle before clamping
  setAngleSlow(COUPLING_SERVO, COUPLING_SERVO_DECOUPLED_POS, RAMP_MS_SLOW);
  delay(100);

  // Optional brief stop pulse can help some mechanisms settle.
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


// Raise the toolhead if needed and enforce a timeout.
bool ensureRaised(uint16_t timeout_ms) {
  ensureServoInit();
  if (!s_servoReady) {
    Serial.println("ERROR: Toolhead controller unavailable; cannot raise toolhead.");
    return false;
  }
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
