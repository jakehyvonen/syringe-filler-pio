#pragma once
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_ADS1X15.h>
#include <stdint.h>
#include "hw/Pins.hpp"

namespace Drivers {
  extern Adafruit_PWMServoDriver PCA;
  extern Adafruit_ADS1115       ADS;

  // Uses Pins::I2C_SDA / Pins::I2C_SCL / Pins::I2C_FREQ
  bool initI2C();

  // Explicit pin/freq override
  bool initI2C(int sda, int scl, uint32_t freq);
  //bool initI2C(int sda = Pins::I2C_SDA, int scl = Pins::I2C_SCL, uint32_t freq = 100000);
  void i2cRecover();        // new

  bool hasPCA();
  bool hasADS();
  bool i2cPresent(uint8_t addr);
}
