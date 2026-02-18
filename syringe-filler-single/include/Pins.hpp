/**
 * @file Pins.hpp
 * @brief Pin mapping for the single-syringe filler.
 */
#pragma once

namespace Pins {

// Stepper driver #1 (A4988)
constexpr int STEPPER1_STEP = 21;
constexpr int STEPPER1_DIR  = 22;
constexpr int STEPPER1_ENABLE = 25;

// Buttons for stepper #1 (active LOW, internal pullups)
constexpr int BUTTON1_WITHDRAW = 32;
constexpr int BUTTON1_DISPENSE = 33;

// Stepper driver #2 (A4988)
// NOTE: Pin choices avoid ESP32-WROOM strapping pins and use full digital I/O pins.
constexpr int STEPPER2_STEP = 26;
constexpr int STEPPER2_DIR  = 14;
// Shared enable line with stepper #1 to keep pin assignment on safe GPIOs.
constexpr int STEPPER2_ENABLE = STEPPER1_ENABLE;

// Buttons for stepper #2 (active LOW, internal pullups)
constexpr int BUTTON2_WITHDRAW = 13;
constexpr int BUTTON2_DISPENSE = 16;

// PN532 RFID (SPI)
constexpr int PN532_MISO = 19;
constexpr int PN532_MOSI = 23;
constexpr int PN532_SCK = 18;
constexpr int PN532_SS = 17;
constexpr int PN532_RST = 27;
}  // namespace Pins
