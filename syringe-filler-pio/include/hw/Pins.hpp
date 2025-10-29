#pragma once
#include <Arduino.h>

namespace Pins {
// ---------- I2C ----------
constexpr int      I2C_SDA  = 21;
constexpr int      I2C_SCL  = 22;
constexpr uint32_t I2C_FREQ = 100000; // Hz

// ---------- Stepper #1 (gantry axis) ----------
constexpr uint8_t STEP1 = 4;
constexpr uint8_t DIR1  = 16;
constexpr uint8_t EN1   = 17;

// ---------- Stepper #2 and #3 (plunger pair) ----------mov
constexpr uint8_t STEP2 = 18;
constexpr uint8_t DIR2  = 19;
constexpr uint8_t EN2   = 5;

constexpr uint8_t STEP3 = 12;
constexpr uint8_t DIR3  = 14;
//constexpr uint8_t EN3   = 27;

// ---------- Bases enable lines (active LOW) ----------
constexpr uint8_t NUM_BASES = 5;
constexpr uint8_t BASE_EN[NUM_BASES] = {27, 26, 25, 33, 32};

// ---------- Limit switches ----------
constexpr uint8_t LIMIT   = 34;
constexpr uint8_t RAISED  = 35;

// ---------- Motion config ----------
constexpr uint8_t  ENABLE_LEVEL  = LOW;
constexpr uint8_t  DISABLE_LEVEL = HIGH;
constexpr bool     HOME_DIR_HIGH = false;
constexpr float    STEPS_PER_MM  = 80.0f;
constexpr long     MIN_POS_STEPS = 0;
constexpr long     MAX_POS_STEPS = (long)(250.0f * STEPS_PER_MM);

// ---------- Step pulse timing ----------
constexpr uint8_t STEP_PULSE_US = 11;
}
