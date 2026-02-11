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
  extern Adafruit_MCP23X17    MCP;
  extern Adafruit_ADS1115       ADS;

  // Uses Pins::I2C_SDA / Pins::I2C_SCL / Pins::I2C_FREQ
  bool initI2C();
  bool initI2C2();

  // Explicit pin/freq override
  bool initI2C(int sda, int scl, uint32_t freq);
  bool initI2C2(int sda, int scl, uint32_t freq);
  //bool initI2C(int sda = Pins::I2C_SDA, int scl = Pins::I2C_SCL, uint32_t freq = 100000);
  void i2cRecover();        // attempt bus recovery on lockup

  bool hasMCP();
  bool hasADS();
  bool i2cPresent(uint8_t addr);
  bool i2c2Present(uint8_t addr);
  void i2cScanBoth();
}
