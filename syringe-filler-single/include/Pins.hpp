/**
 * @file Pins.hpp
 * @brief Pin mapping for the single-syringe filler.
 */
#pragma once

namespace Pins {

// Shared STEP/DIR bus for both stepper drivers (A4988)
constexpr int STEPPER_SHARED_STEP = 21;
constexpr int STEPPER_SHARED_DIR  = 22;

// Stepper driver #1 (A4988)
constexpr int STEPPER1_STEP = STEPPER_SHARED_STEP;
constexpr int STEPPER1_DIR  = STEPPER_SHARED_DIR;
constexpr int STEPPER1_ENABLE = 25;

// Buttons for stepper #1 (active LOW, internal pullups)
constexpr int BUTTON1_WITHDRAW = 32;
constexpr int BUTTON1_DISPENSE = 33;

// Stepper driver #2 (A4988)
// NOTE: Pin choices avoid ESP32-WROOM strapping pins and use full digital I/O pins.
constexpr int STEPPER2_STEP = STEPPER_SHARED_STEP;
constexpr int STEPPER2_DIR  = STEPPER_SHARED_DIR;
// Dedicated enable for stepper #2 on a safe output-capable GPIO.
constexpr int STEPPER2_ENABLE = 26;

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
