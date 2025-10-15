#pragma once
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_ADS1X15.h>

namespace Drivers {
  extern Adafruit_PWMServoDriver PCA;
  extern Adafruit_ADS1115 ADS;
  bool initI2C(int sda=21, int scl=22, uint32_t freq=400000);
  bool hasPCA();
  bool hasADS();
  bool i2cPresent(uint8_t addr);
}
