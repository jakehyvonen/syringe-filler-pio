/**
 * @file Pins.hpp
 * @brief Pin mapping for the single-syringe filler.
 */
#pragma once

namespace Pins {

// Stepper driver (A4988)
constexpr int STEPPER_STEP = 26;
constexpr int STEPPER_DIR  = 27;

// Buttons (active LOW, internal pullups)
constexpr int BUTTON_WITHDRAW = 32;
constexpr int BUTTON_DISPENSE = 33;

// PN532 RFID (I2C)
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
constexpr uint32_t I2C_FREQ = 400000;
constexpr int PN532_IRQ = 25;
constexpr int PN532_RST = 14;

}  // namespace Pins
