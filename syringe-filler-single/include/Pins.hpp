/**
 * @file Pins.hpp
 * @brief Pin mapping for the single-syringe filler.
 */
#pragma once

namespace Pins {

// Stepper driver (A4988)
constexpr int STEPPER_STEP = 21;
constexpr int STEPPER_DIR  = 20;

// Buttons (active LOW, internal pullups)
constexpr int BUTTON_WITHDRAW = 10;
constexpr int BUTTON_DISPENSE = 9;

// PN532 RFID (SPI)
constexpr int PN532_MISO = 8;
constexpr int PN532_MOSI = 7;
constexpr int PN532_SCK = 6;
constexpr int PN532_SS = 4;
constexpr int PN532_RST = 5;
}  // namespace Pins
