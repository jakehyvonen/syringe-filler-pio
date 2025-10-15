#pragma once
#include <Arduino.h>

namespace Pins {
// Stepper #1 (gantry axis)
constexpr uint8_t STEP1 = 4;
constexpr uint8_t DIR1  = 16;
constexpr uint8_t EN1   = 17;

// Stepper #2 and #3 (plunger pair)
constexpr uint8_t STEP2 = 18;
constexpr uint8_t DIR2  = 19;
constexpr uint8_t EN2   = 5;

constexpr uint8_t STEP3 = 12;
constexpr uint8_t DIR3  = 14;
constexpr uint8_t EN3   = 27;

// Bases enable lines (active LOW)
constexpr uint8_t NUM_BASES = 5;
constexpr uint8_t BASE_EN[NUM_BASES] = {23, 26, 32, 33, 13};

// Limit switches (input-only pins on ESP32 -> require ext pullups)
constexpr uint8_t LIMIT   = 34;
constexpr uint8_t RAISED  = 35;

// Motion config
constexpr uint8_t ENABLE_LEVEL  = LOW;
constexpr uint8_t DISABLE_LEVEL = HIGH;
constexpr bool    HOME_DIR_HIGH = false; // LOW in your code
constexpr float   STEPS_PER_MM  = 80.0f;
constexpr long    MIN_POS_STEPS = 0;
constexpr long    MAX_POS_STEPS = (long)(200.0f * STEPS_PER_MM);

// Step pulse timing
constexpr uint8_t STEP_PULSE_US = 4;
}
