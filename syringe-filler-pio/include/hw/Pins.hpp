#pragma once
#include <Arduino.h>

namespace Pins {

// =======================================================
// ==========  I2C BUS (for first PN532 + ADS/PCA)  =======
// =======================================================
constexpr int      I2C_SDA  = 21;
constexpr int      I2C_SCL  = 22;
constexpr uint32_t I2C_FREQ = 100000; // Hz (slow and safe for long bus)

// =======================================================
// ==========  STEPPER SIGNALS  ==========================
// =======================================================

// Shared step/dir between stepper #1 (gantry) and #2 (plunger)
// Only one will ever be active at a time.
constexpr uint8_t STEP12 = 4;
constexpr uint8_t DIR12  = 16;

// Separate enables let you select which motor is live
constexpr uint8_t EN1 = 17;  // enable for gantry
constexpr uint8_t EN2 = 5;   // enable for plunger

// Stepper #3 (toolhead syringe / base drive)
constexpr uint8_t STEP3 = 12;
constexpr uint8_t DIR3  = 14;
// constexpr uint8_t EN3 = — unused for now

// =======================================================
// ==========  BASE ENABLE LINES (active LOW)  ============
// =======================================================

constexpr uint8_t NUM_BASES = 5;
constexpr uint8_t BASE_EN[NUM_BASES] = {27, 26, 25, 33, 32};

// =======================================================
// ==========  LIMIT SWITCHES  ============================
// =======================================================

constexpr uint8_t LIMIT  = 34; // input-only
constexpr uint8_t RAISED = 35; // input-only

// =======================================================
// ==========  MOTION CONFIG  =============================
// =======================================================

constexpr uint8_t  ENABLE_LEVEL  = LOW;
constexpr uint8_t  DISABLE_LEVEL = HIGH;
constexpr bool     HOME_DIR_HIGH = false;
constexpr float    STEPS_PER_MM  = 80.0f;
constexpr long     MIN_POS_STEPS = 0;
constexpr long     MAX_POS_STEPS = (long)(250.0f * STEPS_PER_MM);

// =======================================================
// ==========  STEP PULSE TIMING  =========================
// =======================================================

constexpr uint8_t STEP_PULSE_US = 11;

// =======================================================
// ==========  PN532 READERS  =============================
// =======================================================

// PN532 #1  → I2C  (uses SDA=21, SCL=22; no extra pins needed)
constexpr int PN532_I2C_SDA = I2C_SDA;
constexpr int PN532_I2C_SCL = I2C_SCL;

// ---------- Secondary I2C bus (Wire1) for BaseRFID ----------
constexpr int      I2C2_SDA  = 18;   // choose free GPIOs
constexpr int      I2C2_SCL  = 19;
constexpr uint32_t I2C2_FREQ = 50000; // keep same 50 kHz for stability

// =======================================================
// ==========  LINEAR ENCODER (A/B/Index)  =================
// =======================================================
// Phidgets ENC4111_0 via 74LVC245 (5V → 3V3)
// ESP32 DevKitC pins:
//  - A  → GPIO 23
//  - B  → GPIO 13
//  - Z  → GPIO 39 (VN, input-only)
constexpr uint8_t ENC_A = 23;
constexpr uint8_t ENC_B = 13;
constexpr uint8_t ENC_Z = 39; // index / reference mark (input-only)

// =======================================================
// ==========  UNUSED / PLACEHOLDERS  =====================
// =======================================================

constexpr int PN532_IRQ = -1;
constexpr int PN532_RST = -1;

} // namespace Pins
