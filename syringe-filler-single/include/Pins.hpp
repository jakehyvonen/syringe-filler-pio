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
constexpr int BUTTON_WITHDRAW = 8;
constexpr int BUTTON_DISPENSE = 7;

// PN532 RFID (I2C)
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 20;
constexpr uint32_t I2C_FREQ = 400000;
constexpr int PN532_IRQ = 25;
constexpr int PN532_RST = 14;

}  // namespace Pins
