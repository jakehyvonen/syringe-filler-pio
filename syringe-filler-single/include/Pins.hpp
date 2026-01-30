/**
 * @file Pins.hpp
 * @brief Pin mapping for the single-syringe filler.
 */
#pragma once

namespace Pins {

// Stepper driver (A4988)
constexpr int STEPPER_STEP = 21;
constexpr int STEPPER_DIR  = 22;
// Optional enable pin for the stepper driver (set to -1 if unused)
constexpr int STEPPER_ENABLE = -1;

// Buttons (active LOW, internal pullups)
constexpr int BUTTON_WITHDRAW = 32;
constexpr int BUTTON_DISPENSE = 33;

// PN532 RFID (SPI)
constexpr int PN532_MISO = 19;
constexpr int PN532_MOSI = 23;
constexpr int PN532_SCK = 18;
constexpr int PN532_SS = 17;
constexpr int PN532_RST = 16;
}  // namespace Pins
