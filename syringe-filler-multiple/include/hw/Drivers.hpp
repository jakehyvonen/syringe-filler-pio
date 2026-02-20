/**
 * @file Drivers.hpp
 * @brief I2C driver initialization and device availability helpers.
 */
#pragma once
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23X17.h>
#include <stdint.h>
#include "hw/Pins.hpp"

namespace Drivers {
  extern Adafruit_MCP23X17 MCP;
  extern Adafruit_ADS1115 ADS;
  extern Adafruit_MCP23X17 BASE_EN_EXPANDER;

  // Uses Pins::I2C_SDA / Pins::I2C_SCL / Pins::I2C_FREQ
  bool initI2C();

  // Explicit pin/freq override
  bool initI2C(int sda, int scl, uint32_t freq);

  bool hasMCP();
  bool hasADS();
  bool hasBaseEnableExpander();
  bool i2cPresent(uint8_t addr);
  void i2cScanBoth();
}
